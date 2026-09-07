#[allow(deprecated)]
use esp_idf_hal::gpio::OutputPin;
#[allow(deprecated)]
use esp_idf_hal::rmt::RmtChannel;
use ws2812_esp32_rmt_driver::*;

pub struct LedDriver<'a> { ws2812: Ws2812Esp32RmtDriver<'a> }

impl<'a> LedDriver<'a> {
    #[allow(deprecated)]
    pub fn new(led_pin: impl OutputPin + 'a, rmt_channel: impl RmtChannel + 'a) -> Result<Self, Ws2812Esp32RmtDriverError> {
        Ws2812Esp32RmtDriver::new(rmt_channel, led_pin).map(|ws2812| Self { ws2812 })
    }
    pub fn set_rgb(&mut self, r: u8, g: u8, b: u8) -> Result<(), Ws2812Esp32RmtDriverError> {
        self.ws2812.write_blocking(Box::new([g, r, b].into_iter()))
    }
}
