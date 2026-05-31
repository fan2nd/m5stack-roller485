use crate::communication::foc::{COMMAND, Command};
use crate::control::FocController;
use crate::driver::cordic::Cordic;
use crate::driver::current::{CURRENT_READ_WORDS, SyncedCurrentSampler};
use crate::task::angle;

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub dt_s: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            dt_s: 1.0 / 20_000.0,
        }
    }
}

#[embassy_executor::task]
pub async fn task(config: Config, mut current: SyncedCurrentSampler) {
    let mut controller = FocController::default();
    let mut cordic = Cordic::new();
    let mut current_buf = [0u16; CURRENT_READ_WORDS];

    loop {
        let current_sample = match current.read_latest(&mut current_buf).await {
            Ok(sample) => sample,
            Err(_) => {
                current.clear();
                continue;
            }
        };

        drain_commands(&mut controller);

        let theta = angle::predicted_now().await;
        let (_sin_t, _cos_t) = cordic.sin_cos(theta);

        let _ = (config.dt_s, current_sample);
    }
}

fn drain_commands(controller: &mut FocController) {
    while let Some(command) = COMMAND.try_take() {
        match command {
            Command::Enable => controller.request_enable(),
            Command::Disable => controller.request_disable(),
            Command::ClearFault => controller.clear_fault(),
            Command::SetMode(mode) => controller.set_mode(mode),
            Command::SetTarget(target) => controller.set_target(target),
            Command::SetGains(gains) => controller.set_gains(gains),
            Command::SetLimits(limits) => controller.set_limits(limits),
            Command::SetEffects(effects) => controller.effects = effects,
        }
    }
}
