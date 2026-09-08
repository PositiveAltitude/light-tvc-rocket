use anyhow::{anyhow, Result};
use esp_idf_hal::modem::WifiModemPeripheral;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::wifi::*;
use light_robot_core_api::*;
use log::info;
use std::sync::{Arc, Mutex};

/// One initial connection plus two retries before recovery AP mode.
const CLIENT_CONNECT_ATTEMPTS: usize = 3;
const CLIENT_RETRY_DELAY_MS: u32 = 1_000;

pub struct WiFi<'a> {
    _wifi: BlockingWifi<EspWifi<'a>>,
    _state: Arc<Mutex<State>>,
}

impl<'a> WiFi<'a> {
    pub fn new(
        configuration: WifiConnectionConfiguration,
        modem: impl WifiModemPeripheral + 'static,
        sysloop: EspSystemEventLoop,
        state: Arc<Mutex<State>>,
    ) -> Result<Self> {
        let esp_wifi = EspWifi::new(modem, sysloop.clone(), None)?;
        let mut wifi = BlockingWifi::wrap(esp_wifi, sysloop)?;

        let client_config = match configuration.clone() {
            WifiConnectionConfiguration {
                connection_type: WifiConnectionType::StartAccessPoint,
                ..
            } => return Self::start_access_point(wifi, state),
            WifiConnectionConfiguration {
                connection_type: WifiConnectionType::ConnectToExternal,
                credentials: WifiCredentials { ssid, password },
            } => Configuration::Client(ClientConfiguration {
                ssid: heapless::String::try_from(ssid.as_str())
                    .map_err(|_| anyhow!("Wi-Fi SSID is too long"))?,
                password: heapless::String::try_from(password.as_str())
                    .map_err(|_| anyhow!("Wi-Fi password is too long"))?,
                auth_method: if password.is_empty() {
                    AuthMethod::None
                } else {
                    AuthMethod::WPA2Personal
                },
                channel: None,
                ..Default::default()
            }),
        };

        for attempt in 1..=CLIENT_CONNECT_ATTEMPTS {
            let connection_result: Result<()> = (|| {
                wifi.set_configuration(&client_config)?;
                wifi.start()?;
                wifi.connect()?;
                wifi.wait_netif_up()?;
                Ok(())
            })();

            match connection_result {
                Ok(()) => {
                    info!(
                        "WIFI Connect -- OK on attempt {}/{}",
                        attempt,
                        CLIENT_CONNECT_ATTEMPTS
                    );
                    let mut configuration = configuration.clone();
                    configuration.credentials.password = "".into();
                    state.lock().unwrap().wifi_state = configuration;
                    let ip_info = wifi.wifi().sta_netif().get_ip_info()?;
                    info!("DHCP info: {:?}", ip_info);
                    return Ok(Self {
                        _wifi: wifi,
                        _state: state,
                    });
                }
                Err(error) => {
                    info!(
                        "WIFI Connect attempt {}/{} -- FAIL: {:?}",
                        attempt,
                        CLIENT_CONNECT_ATTEMPTS,
                        error
                    );
                    // Start every retry from a known driver state; a failed
                    // authentication attempt can otherwise leave the station
                    // still connecting when the next connect is requested.
                    let _ = wifi.stop();
                    if attempt < CLIENT_CONNECT_ATTEMPTS {
                        esp_idf_hal::delay::FreeRtos::delay_ms(CLIENT_RETRY_DELAY_MS);
                    }
                }
            }
        }

        info!("WIFI Connect -- all attempts failed; starting access point");
        Self::start_access_point(wifi, state)
    }

    fn start_access_point(
        mut wifi: BlockingWifi<EspWifi<'a>>,
        state: Arc<Mutex<State>>,
    ) -> Result<Self> {
        wifi.set_configuration(&Configuration::AccessPoint(AccessPointConfiguration {
            ssid: "LRC-wifi".try_into().unwrap(),
            channel: 1,
            ..Default::default()
        }))?;
        wifi.start()?;
        wifi.wait_netif_up()?;
        info!("WIFI AP Start -- OK");
        state.lock().unwrap().wifi_state = WifiConnectionConfiguration {
            connection_type: WifiConnectionType::StartAccessPoint,
            credentials: WifiCredentials {
                ssid: "LRC-wifi".into(),
                password: "".into(),
            },
        };
        let ip_info = wifi.wifi().ap_netif().get_ip_info()?;
        info!("DHCP -- OK");
        info!("DHCP info: {:?}", ip_info);
        Ok(Self {
            _wifi: wifi,
            _state: state,
        })
    }
}
