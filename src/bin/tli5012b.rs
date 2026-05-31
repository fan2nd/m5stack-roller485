#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use m5stack_roller485::driver::tli5012b::{Error, Tli5012b};
use m5stack_roller485::resources::*;
use m5stack_roller485::{rcc, split_resources};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(rcc::config());
    let resources = split_resources!(p);
    let mut sensor = Tli5012b::new(resources.angle_sensor);

    info!("TLI5012B angle start");

    loop {
        match sensor.read_angle() {
            Ok(sample) => {
                info!(
                    "aval=0x{:04x} count={} safety=0x{:04x} angle={} deg",
                    sample.raw_angle, sample.raw_count, sample.safety, sample.angle_deg
                );
            }
            Err(Error::Spi) => {
                error!("tli5012b spi transfer failed");
            }
            Err(Error::AllOnes) => {
                warn!("tli5012b returned all ones; check wiring, CS, mode, and sensor power");
            }
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}
