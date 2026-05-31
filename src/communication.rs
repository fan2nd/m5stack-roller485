pub mod ws2812 {
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::signal::Signal;

    pub const LED_COUNT: usize = 2;

    pub static SIGNAL: Signal<CriticalSectionRawMutex, Command> = Signal::new();

    #[derive(Clone, Copy, Debug, Default)]
    pub struct Rgb {
        pub r: u8,
        pub g: u8,
        pub b: u8,
    }

    impl Rgb {
        pub const OFF: Self = Self::new(0, 0, 0);

        pub const fn new(r: u8, g: u8, b: u8) -> Self {
            Self { r, g, b }
        }
    }

    #[derive(Clone, Copy, Debug)]
    pub enum Command {
        Set([Rgb; LED_COUNT]),
        Off,
    }

    pub fn set(colors: [Rgb; LED_COUNT]) {
        SIGNAL.signal(Command::Set(colors));
    }

    pub fn off() {
        SIGNAL.signal(Command::Off);
    }
}

pub mod foc {
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::signal::Signal;

    use crate::control::{ControlMode, Effects, Gains, Limits};

    pub static COMMAND: Signal<CriticalSectionRawMutex, Command> = Signal::new();

    #[derive(Clone, Copy, Debug)]
    pub enum Command {
        Enable,
        Disable,
        ClearFault,
        SetMode(ControlMode),
        SetTarget(f32),
        SetGains(Gains),
        SetLimits(Limits),
        SetEffects(Effects),
    }

    pub fn send(command: Command) {
        COMMAND.signal(command);
    }
}

pub mod commissioning {
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::signal::Signal;

    pub static COMMAND: Signal<CriticalSectionRawMutex, Command> = Signal::new();

    #[derive(Clone, Copy, Debug)]
    pub enum Command {
        Start,
        Abort,
    }

    pub fn send(command: Command) {
        COMMAND.signal(command);
    }
}
