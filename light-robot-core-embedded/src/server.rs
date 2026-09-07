use light_robot_core_api::*;
use std::io;
use std::io::ErrorKind;

use anyhow::{Error, Result};
use esp_idf_hal::io::EspIOError;
use esp_idf_svc::http::server::{Connection, EspHttpServer, Request};
use esp_idf_svc::http::server::ws::EspHttpWsDetachedSender;
use esp_idf_svc::ws::FrameType;
use esp_idf_sys::EspError;
use include_dir::{include_dir, Dir};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

static DIST: Dir<'_> = include_dir!("$CARGO_MANIFEST_DIR/../light-robot-core-frontend/dist-gz/");

pub struct Server {
    server: EspHttpServer<'static>,
}

impl Server {
    pub fn new<F>(
        state: Arc<Mutex<State>>,
        test_data: Arc<Mutex<ServoTestResult>>,
        command_handler: F,
    ) -> Result<Self>
    where
        F: Fn(&Command) -> Result<()> + Send + Sync + 'static,
    {
        use esp_idf_svc::http::server::EspHttpServer;
        use esp_idf_svc::http::server::Method;
        use esp_idf_svc::io::Write;

        let mut conf = esp_idf_svc::http::server::Configuration::default();
        // WebSocket command decoding and the handshake run in ESP-IDF's HTTPD
        // task. Rust/serde needs substantially more headroom than its 6 KiB
        // default stack.
        conf.stack_size = 16 * 1024;
        conf.max_resp_headers = 100;
        conf.max_uri_handlers = 100;

        let mut server = EspHttpServer::new(&conf)?;
        let command_handler = Arc::new(command_handler);
        let rest_command_handler = command_handler.clone();
        let ws_sender = Arc::new(Mutex::new(None::<EspHttpWsDetachedSender>));
        let rest_state = state.clone();
        let rest_test_data = test_data.clone();

        fn serve_file<'a>(
            server: &'a mut EspHttpServer,
            path: &'static str,
            content: &'static [u8],
        ) -> Result<(), EspError> {
            server.fn_handler(
                format!("/{}", path).as_ref(),
                Method::Get,
                move |req| -> Result<(), EspIOError> {
                    let content_type = if path.ends_with(".js") {
                        "application/javascript"
                    } else if path.ends_with(".wasm") {
                        "application/wasm"
                    } else if path.ends_with(".html") {
                        "text/html"
                    } else if path.ends_with(".htm") {
                        "text/html"
                    } else if path.ends_with(".html") {
                        "text/html"
                    } else if path.ends_with(".css") {
                        "text/css"
                    } else {
                        "text/html"
                    };

                    req.into_response(
                        200,
                        None,
                        &[
                            ("Content-Type", content_type),
                            ("Content-Encoding", "gzip"),
                            ("Access-Control-Allow-Origin", "*"),
                        ],
                    )?
                    .write_all(content)?;
                    Ok(())
                },
            )?;
            Ok(())
        }

        server
            .fn_handler(
                "/state",
                Method::Get,
                move |req| -> Result<(), EspIOError> {
                    let state = rest_state.lock().unwrap().to_owned();

                    req.into_response(
                        200,
                        None,
                        &[
                            ("Content-Type", "application/json"),
                            ("Access-Control-Allow-Origin", "*"),
                        ],
                    )?
                    .write_all(serde_json::to_string(&state).unwrap().as_bytes())?;
                    Ok(())
                },
            )?
            .fn_handler(
                "/command",
                Method::Post,
                move |mut req| -> Result<(), EspIOError> {
                    struct ReqRead<'a, A> {
                        req: &'a mut Request<A>,
                    }

                    impl<'a, A> io::Read for ReqRead<'a, A>
                    where
                        A: Connection,
                    {
                        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
                            self.req
                                .read(buf)
                                .map_err(|e| io::Error::new(ErrorKind::BrokenPipe, ""))
                        }
                    }

                    let command = serde_json::from_reader::<_, Command>(ReqRead { req: &mut req });

                    //TODO headers (cross-origin, content-type)
                    match command {
                        Ok(command) => {
                            match rest_command_handler(&command) {
                                Ok(_) => {
                                    req.into_ok_response()?;
                                }
                                Err(_) => {
                                    req.into_status_response(500)?;
                                }
                            }

                            Ok(())
                        }
                        Err(_) => req
                            .into_response(400, Some("Unable to parse command"), &[])
                            .map(|_| ()),
                    }?;

                    Ok(())
                },
            )?
            .fn_handler(
                "/test_result",
                Method::Get,
                move |req| -> Result<(), EspIOError> {
                    let test_data = rest_test_data.lock().unwrap().to_owned();

                    req.into_response(
                        200,
                        None,
                        &[
                            ("Content-Type", "application/json"),
                            ("Access-Control-Allow-Origin", "*"),
                        ],
                    )?
                    .write_all(serde_json::to_string(&test_data).unwrap().as_bytes())?;
                    Ok(())
                },
            )?;

        let ws_command_handler = command_handler.clone();
        let ws_sender_for_handler = ws_sender.clone();
        let ws_state = state.clone();
        server.ws_handler("/ws", None, move |connection| -> Result<()> {
            if connection.is_new() {
                // Keep the HTTPD handshake small. The background publisher
                // sends the initial state within 50 ms after this sender is
                // registered, avoiding JSON serialization on HTTPD's stack.
                let sender = connection.create_detached_sender()?;
                *ws_sender_for_handler.lock().unwrap() = Some(sender);
            } else if !connection.is_closed() {
                let mut buffer = [0_u8; 4096];
                let (frame_type, length) = connection.recv(&mut buffer)?;
                if matches!(frame_type, FrameType::Text(_)) {
                    // EspHttpWsConnection reports text-frame length including
                    // its NUL terminator; serde_json requires the JSON bytes
                    // only, otherwise every socket command is rejected.
                    let json = &buffer[..length.saturating_sub(1)];
                    if let Ok(command) = serde_json::from_slice::<Command>(json) {
                        ws_command_handler(&command)?;
                    }
                }
            }
            Ok(())
        })?;

        let ws_state = state.clone();
        let ws_test_data = test_data.clone();
        thread::spawn(move || {
            let mut sent_test_revision = 0;
            loop {
                thread::sleep(Duration::from_millis(50));
                let state = ws_state.lock().unwrap().clone();
                let mut sender = match ws_sender.lock().unwrap().clone() { Some(sender) => sender, None => continue };
                // Do not compete with the time-sensitive CAN capture loop.
                // The final result and fresh state are sent immediately after
                // the test clears this flag.
                if state.servo_calibration.test_running {
                    continue;
                }
                if state.servo_calibration.test_result_revision != sent_test_revision {
                    let test = ws_test_data.lock().unwrap().clone();
                    if let Ok(message) = serde_json::to_vec(&SocketMessage::TestResult(test)) {
                        let _ = sender.send(FrameType::Text(false), &message);
                    }
                    sent_test_revision = state.servo_calibration.test_result_revision;
                }
                if let Ok(message) = serde_json::to_vec(&SocketMessage::State(state)) {
                    let _ = sender.send(FrameType::Text(false), &message);
                }
            }
        });

        let f = DIST.get_file("index.html").unwrap();
        serve_file(&mut server, "", f.contents())?;

        fn serve_dir(server: &mut EspHttpServer, dir: &'static Dir) -> Result<()> {
            dir.files().for_each(|f| {
                serve_file(server, f.path().to_str().unwrap(), f.contents()).unwrap();
            });
            if dir.dirs().next().is_some() {
                for dir in dir.dirs() {
                    serve_dir(server, dir).unwrap();
                }
            }
            Ok(())
        }

        DIST.files().for_each(|f| {
            serve_file(&mut server, f.path().to_str().unwrap(), f.contents()).unwrap();
        });
        for dir in DIST.dirs() {
            serve_dir(&mut server, dir).unwrap();
        }

        Ok(Self { server })
    }
}
