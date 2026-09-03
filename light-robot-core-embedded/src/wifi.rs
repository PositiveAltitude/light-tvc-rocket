use anyhow::Result;
use esp_idf_hal::modem::WifiModemPeripheral;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::wifi::*;
use light_robot_core_api::*;
use log::info;
use std::sync::{Arc, Mutex};

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

        let cfg = match configuration.clone() {
            WifiConnectionConfiguration {
                connection_type: WifiConnectionType::StartAccessPoint,
                credentials: WifiCredentials { ssid, password },
            } => Configuration::AccessPoint(AccessPointConfiguration {
                ssid: heapless::String::try_from(ssid.as_str()).unwrap(),
                channel: 1,
                password: heapless::String::try_from(password.as_str()).unwrap(),
                auth_method: AuthMethod::WPA2Personal,
                ..Default::default()
            }),
            WifiConnectionConfiguration {
                connection_type: WifiConnectionType::ConnectToExternal,
                credentials: WifiCredentials { ssid, password },
            } => Configuration::Client(ClientConfiguration {
                ssid: heapless::String::try_from(ssid.as_str()).unwrap(),
                password: heapless::String::try_from(password.as_str()).unwrap(),
                channel: None,
                ..Default::default()
            }),
        };

        let client_configuration_result = wifi.set_configuration(&cfg);

        let connection_result = client_configuration_result.and_then(|_| {
            wifi.start()?;
            wifi.connect()?;
            info!("WIFI Connect -- OK");
            let mut configuration = configuration.clone();
            configuration.credentials.password = "".into();
            state.lock().unwrap().wifi_state = configuration;
            Ok(())
        });

        match connection_result {
            Ok(_) => (),
            Err(_) => {
                info!("WIFI Connect -- FAIL");
                let ap_configuration_result =
                    wifi.set_configuration(&Configuration::AccessPoint(AccessPointConfiguration {
                        ssid: "LRC-wifi".try_into().unwrap(),
                        channel: 1,
                        ..Default::default()
                    }));
                ap_configuration_result.and_then(|_| {
                    wifi.start()?;
                    info!("WIFI AP Start -- OK");
                    state.lock().unwrap().wifi_state = WifiConnectionConfiguration {
                        connection_type: WifiConnectionType::StartAccessPoint,
                        credentials: WifiCredentials {
                            ssid: "LRC-wifi".into(),
                            password: "".into(),
                        },
                    };
                    Ok(())
                })?;
            }
        };

        wifi.wait_netif_up()?;
        let ip_info = wifi.wifi().ap_netif().get_ip_info()?;
        info!("DHCP -- OK");
        info!("DHCP info: {:?}", ip_info);
        Ok(Self {
            _wifi: wifi,
            _state: state,
        })
    }
}
