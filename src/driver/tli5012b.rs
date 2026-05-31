use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::mode::Master;
use embassy_stm32::spi::{Config as SpiConfig, Direction, MODE_1, Spi};
use embassy_stm32::time::Hertz;

use crate::resources::AngleSensorResources;

const READ_AVAL: u16 = 0x8021;
const ANGLE_MASK: u16 = 0x7fff;

pub struct Tli5012b {
    spi: Spi<'static, Blocking, Master>,
    cs: Output<'static>,
}

impl Tli5012b {
    pub fn new(resources: AngleSensorResources) -> Self {
        let mut spi_config = SpiConfig::default();
        spi_config.mode = MODE_1;
        spi_config.frequency = Hertz(1_000_000);
        spi_config.gpio_speed = Speed::VeryHigh;

        let spi = Spi::new_blocking(
            resources.spi,
            resources.sck,
            resources.mosi,
            resources.miso,
            spi_config,
        );
        let cs = Output::new(resources.nss, Level::High, Speed::VeryHigh);

        Self { spi, cs }
    }

    pub fn read_angle(&mut self) -> Result<AngleSample, Error> {
        let mut frame = [0u16; 2];

        self.cs.set_low();
        self.spi.set_direction(Some(Direction::Transmit));
        let result = self.spi.blocking_write(&[READ_AVAL]);
        if result.is_ok() {
            for _ in 0..64 {
                cortex_m::asm::nop();
            }

            self.spi.set_direction(Some(Direction::Receive));
            let result = self.spi.blocking_read(&mut frame);
            self.spi.set_direction(None);
            self.cs.set_high();
            result.map_err(|_| Error::Spi)?;
        } else {
            self.spi.set_direction(None);
            self.cs.set_high();
            result.map_err(|_| Error::Spi)?;
        }

        let raw_angle = frame[0];
        let safety = frame[1];

        if raw_angle == 0xffff && safety == 0xffff {
            return Err(Error::AllOnes);
        }

        let raw_count = raw_angle & ANGLE_MASK;
        let angle_deg = raw_count as f32 * 360.0 / 32_768.0;

        Ok(AngleSample {
            raw_angle,
            safety,
            raw_count,
            angle_deg,
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AngleSample {
    pub raw_angle: u16,
    pub safety: u16,
    pub raw_count: u16,
    pub angle_deg: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Spi,
    AllOnes,
}
