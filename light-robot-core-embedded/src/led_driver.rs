use esp_idf_hal::gpio::OutputPin;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::rmt::RmtChannel;
use ws2812_esp32_rmt_driver::*;

pub struct LedDriver<'a> {
    ws2812: Ws2812Esp32RmtDriver<'a>,
}

impl<'a> LedDriver<'a> {
    pub fn new(
        led_pin: impl Peripheral<P = impl OutputPin> + 'a,
        rmt_channel: impl Peripheral<P = impl RmtChannel> + 'a,
    ) -> Result<Self, Ws2812Esp32RmtDriverError> {
        let ws =
            Ws2812Esp32RmtDriver::new(rmt_channel, led_pin);

        ws.map(|ws| Self { ws2812: ws })
    }

    pub fn set_rgb(&mut self,r: u8, g: u8, b: u8) -> Result<(), Ws2812Esp32RmtDriverError> {
        self.ws2812.write_blocking(Box::new([g, r, b].into_iter()))
    }
    pub fn off(&mut self) -> Result<(), Ws2812Esp32RmtDriverError> {
        self.ws2812.write_blocking(Box::new([0, 0, 0].into_iter()))
    }
}