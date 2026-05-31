use embassy_stm32::{Config, rcc};

pub fn config() -> Config {
    let mut config = Config::default();
    config.rcc.sys = rcc::Sysclk::PLL1_R;
    config.rcc.pll = Some(rcc::Pll {
        source: rcc::PllSource::HSI,
        prediv: rcc::PllPreDiv::DIV1,
        mul: rcc::PllMul::MUL21,
        divp: Some(rcc::PllPDiv::DIV2),
        divq: Some(rcc::PllQDiv::DIV2),
        divr: Some(rcc::PllRDiv::DIV2),
    });
    config.rcc.mux.adc12sel = rcc::mux::Adcsel::SYS;
    config
}
