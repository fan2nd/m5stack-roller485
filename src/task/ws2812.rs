use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::khz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::Timer;

use crate::communication::ws2812::{Command, LED_COUNT, Rgb, SIGNAL};
use crate::resources::Ws2812Resources;

const BITS_PER_LED: usize = 24;
const RESET_SLOTS: usize = 1;
const WAVEFORM_LEN: usize = LED_COUNT * BITS_PER_LED + RESET_SLOTS;

#[embassy_executor::task]
pub async fn task(resources: Ws2812Resources) {
    let mut pwm = SimplePwm::new(
        resources.timer,
        None,
        Some(PwmPin::new(resources.pin, OutputType::PushPull)),
        None,
        None,
        khz(800),
        CountingMode::EdgeAlignedUp,
    );
    let mut dma = resources.dma;

    let channel = Channel::Ch2;
    pwm.channel(channel).set_duty_cycle(0);

    let max_duty = pwm.max_duty_cycle() as u16;
    let n0 = 8 * max_duty / 25;
    let n1 = 2 * n0;
    let mut waveform = [0u16; WAVEFORM_LEN];

    loop {
        let colors = match SIGNAL.wait().await {
            Command::Set(colors) => colors,
            Command::Off => [Rgb::OFF; LED_COUNT],
        };

        encode_colors(&mut waveform, n0, n1, &colors);
        pwm.waveform_up(dma.reborrow(), channel, &waveform).await;
        Timer::after_micros(80).await;
    }
}

fn encode_colors(waveform: &mut [u16; WAVEFORM_LEN], n0: u16, n1: u16, colors: &[Rgb; LED_COUNT]) {
    let mut offset = 0;
    for color in colors {
        offset = encode_byte(waveform, offset, color.g, n0, n1);
        offset = encode_byte(waveform, offset, color.r, n0, n1);
        offset = encode_byte(waveform, offset, color.b, n0, n1);
    }

    waveform[offset] = 0;
}

fn encode_byte(
    waveform: &mut [u16; WAVEFORM_LEN],
    mut offset: usize,
    byte: u8,
    n0: u16,
    n1: u16,
) -> usize {
    for bit in (0..8).rev() {
        waveform[offset] = if byte & (1 << bit) != 0 { n1 } else { n0 };
        offset += 1;
    }
    offset
}
