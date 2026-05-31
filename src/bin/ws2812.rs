#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use m5stack_roller485::communication::ws2812::{self, Rgb};
use m5stack_roller485::resources::*;
use m5stack_roller485::task;
use m5stack_roller485::{rcc, split_resources};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(rcc::config());
    let resources = split_resources!(p);

    spawner.spawn(task::ws2812::task(resources.ws2812).unwrap());
    info!("WS2812 task demo start");

    let frames = [
        [Rgb::new(16, 0, 0), Rgb::new(0, 0, 16)],
        [Rgb::new(0, 16, 0), Rgb::new(16, 0, 0)],
        [Rgb::new(0, 0, 16), Rgb::new(0, 16, 0)],
        [Rgb::new(8, 8, 8), Rgb::new(8, 8, 8)],
        [Rgb::OFF, Rgb::OFF],
    ];

    loop {
        for colors in frames {
            ws2812::set(colors);
            Timer::after(Duration::from_millis(400)).await;
        }
    }
}
