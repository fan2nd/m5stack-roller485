use embassy_time::{Duration, Timer};

use crate::communication::commissioning::{COMMAND, Command};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Idle,
    CurrentOffset,
    AlignRotor,
    DragForward,
    DetectDirection,
    DetectPolePairs,
    MeasureOffset,
    Done,
    Failed,
}

#[derive(Clone, Copy, Debug)]
pub struct MotorCalibration {
    pub sensor_dir: f32,
    pub pole_pairs: u8,
    pub electrical_offset: f32,
    pub current_offsets: [f32; 3],
}

impl Default for MotorCalibration {
    fn default() -> Self {
        Self {
            sensor_dir: 1.0,
            pole_pairs: 1,
            electrical_offset: 0.0,
            current_offsets: [2048.0; 3],
        }
    }
}

#[embassy_executor::task]
pub async fn task() {
    let mut state = State::Idle;
    let mut _calibration = MotorCalibration::default();

    loop {
        if let Some(command) = COMMAND.try_take() {
            match command {
                Command::Start => state = State::CurrentOffset,
                Command::Abort => state = State::Idle,
            }
        }

        state = step_state(state);
        Timer::after(Duration::from_millis(10)).await;
    }
}

fn step_state(state: State) -> State {
    match state {
        State::Idle | State::Done | State::Failed => state,
        State::CurrentOffset => State::AlignRotor,
        State::AlignRotor => State::DragForward,
        State::DragForward => State::DetectDirection,
        State::DetectDirection => State::DetectPolePairs,
        State::DetectPolePairs => State::MeasureOffset,
        State::MeasureOffset => State::Done,
    }
}
