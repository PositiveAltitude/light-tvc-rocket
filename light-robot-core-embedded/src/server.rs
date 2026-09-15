use light_robot_core_api::*;
use anyhow::Result;
use esp_idf_hal::cpu::Core;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::io::EspIOError;
use esp_idf_svc::http::server::ws::EspHttpWsDetachedSender;
use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::ws::FrameType;
use esp_idf_sys::EspError;
use include_dir::{include_dir, Dir};
use log::info;
use std::ffi::CStr;
use std::sync::{Arc, Mutex};
use std::thread;

static DIST: Dir<'_> = include_dir!("$CARGO_MANIFEST_DIR/../light-robot-core-frontend/dist-gz/");

pub struct Server {
    server: EspHttpServer<'static>,
}

impl Server {
    pub fn new<F>(
        state: Arc<Mutex<State>>,
        test_data: Arc<Mutex<ServoTestResult>>,
        inertia_capture_result: Arc<Mutex<InertiaCaptureResult>>,
        hil_result: Arc<Mutex<HilSimulationResult>>,
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
        let ws_sender = Arc::new(Mutex::new(None::<EspHttpWsDetachedSender>));

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

        let ws_command_handler = command_handler.clone();
        let ws_sender_for_handler = ws_sender.clone();
        server.ws_handler("/ws", None, move |connection| -> Result<()> {
            if connection.is_new() {
                info!("UI WebSocket connected");
                // Keep the HTTPD handshake small. The background publisher
                // sends the initial state within 50 ms after this sender is
                // registered, avoiding JSON serialization on HTTPD's stack.
                let sender = connection.create_detached_sender()?;
                *ws_sender_for_handler.lock().unwrap() = Some(sender);
            } else if connection.is_closed() {
                info!("UI WebSocket disconnected");
            } else {
                let mut buffer = [0_u8; 4096];
                let (frame_type, length) = connection.recv(&mut buffer)?;
                if matches!(frame_type, FrameType::Text(_)) {
                    // EspHttpWsConnection reports text-frame length including
                    // its NUL terminator; serde_json requires the JSON bytes
                    // only, otherwise every socket command is rejected.
                    let json = &buffer[..length.saturating_sub(1)];
                    match serde_json::from_slice::<Command>(json) {
                        Ok(command) => {
                            // A rejected command must not silently close the
                            // UI socket. In particular, checklist safety gates
                            // need to be diagnosable at the bench.
                            if let Err(error) = ws_command_handler(&command) {
                                info!("UI command rejected: {:?}", error);
                            }
                        }
                        Err(error) => info!("Malformed UI command: {:?}", error),
                    }
                }
            }
            Ok(())
        })?;

        let ws_state = state.clone();
        let ws_test_data = test_data.clone();
        let ws_inertia_capture_result = inertia_capture_result.clone();
        let ws_hil_result = hil_result.clone();
        let default_publisher_thread_config =
            esp_idf_hal::task::thread::ThreadSpawnConfiguration::get();
        esp_idf_hal::task::thread::ThreadSpawnConfiguration {
            name: Some(CStr::from_bytes_with_nul(b"state-publisher\0").unwrap()),
            stack_size: 16 * 1024,
            pin_to_core: Some(Core::Core0),
            ..Default::default()
        }
        .set()
        .unwrap();
        thread::spawn(move || {
            let mut sent_test_revision = 0;
            let mut sent_inertia_result_revision = 0;
            let mut sent_hil_result_revision = 0;
            loop {
                FreeRtos::delay_ms(50);
                let state = ws_state.lock().unwrap().clone();
                let mut sender = match ws_sender.lock().unwrap().clone() {
                    Some(sender) => sender,
                    None => continue,
                };
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
                if state.inertia_capture.result_revision != sent_inertia_result_revision {
                    let result = ws_inertia_capture_result.lock().unwrap().clone();
                    if let Ok(message) =
                        serde_json::to_vec(&SocketMessage::InertiaCaptureResult(result))
                    {
                        let _ = sender.send(FrameType::Text(false), &message);
                    }
                    sent_inertia_result_revision = state.inertia_capture.result_revision;
                }
                if state.hil_simulation.result_revision != sent_hil_result_revision {
                    let result = ws_hil_result.lock().unwrap();
                    let total = result.samples.len().div_ceil(4) as u16;
                    for (index, samples) in result.samples.chunks(4).enumerate() {
                        let chunk = HilSimulationChunk { revision: state.hil_simulation.result_revision, index: index as u16, total, missed_deadlines: result.missed_deadlines, samples: samples.to_vec() };
                        if let Ok(message) = serde_json::to_vec(&SocketMessage::HilSimulationChunk(chunk)) { let _ = sender.send(FrameType::Text(false), &message); }
                        FreeRtos::delay_ms(2);
                    }
                    sent_hil_result_revision = state.hil_simulation.result_revision;
                }
                if let Ok(message) = serde_json::to_vec(&SocketMessage::State(state)) {
                    let _ = sender.send(FrameType::Text(false), &message);
                }
            }
        });
        if let Some(default_publisher_thread_config) = default_publisher_thread_config {
            default_publisher_thread_config.set()?;
        }

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
