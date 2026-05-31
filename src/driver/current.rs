use embassy_stm32::adc::{
    Adc, AdcChannel as _, AdcConfig, ConversionTrigger, Exten, RegularConversionMode,
    RingBufferedAdc, SampleTime,
};
use embassy_stm32::pac::timer::vals::Mms2;
use embassy_stm32::peripherals::ADC1;

use crate::resources::CurrentResources;

pub const CURRENT_FRAME_WORDS: usize = 4;
pub const CURRENT_DMA_BUF_WORDS: usize = CURRENT_FRAME_WORDS * 80;
pub const CURRENT_READ_WORDS: usize = CURRENT_DMA_BUF_WORDS / 2;

const ADC1_REGULAR_TRIGGER_TIM1_TRGO2: u8 = 10;

#[derive(Clone, Copy, Debug, Default)]
pub struct CurrentRaw {
    pub ia: u16,
    pub ib: u16,
    pub ic: u16,
    pub vbus: u16,
}

pub struct SyncedCurrentSampler {
    adc: RingBufferedAdc<'static, ADC1>,
    decoder: CurrentDecoder,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CurrentError {
    Overrun,
}

impl SyncedCurrentSampler {
    pub fn new(
        resources: CurrentResources,
        dma_buf: &'static mut [u16; CURRENT_DMA_BUF_WORDS],
        config: CurrentConfig,
    ) -> Self {
        let adc = Adc::new(resources.adc, AdcConfig::default());
        let vbus = resources.vbus.degrade_adc();
        let ia = resources.phase_a_current.degrade_adc();
        let ib = resources.phase_b_current.degrade_adc();
        let ic = resources.phase_c_current.degrade_adc();

        let sequence = [
            (vbus, SampleTime::CYCLES12_5),
            (ia, SampleTime::CYCLES12_5),
            (ib, SampleTime::CYCLES12_5),
            (ic, SampleTime::CYCLES12_5),
        ]
        .into_iter();

        let adc = adc.into_ring_buffered(
            resources.adc_dma,
            dma_buf,
            sequence,
            RegularConversionMode::Triggered(tim1_trgo2_regular_trigger()),
        );

        Self {
            adc,
            decoder: CurrentDecoder::new(config),
        }
    }

    pub fn set_offsets(&mut self, offsets: [f32; 3]) {
        self.decoder.set_offsets(offsets);
    }

    pub async fn read_latest(
        &mut self,
        read_buf: &mut [u16; CURRENT_READ_WORDS],
    ) -> Result<CurrentSample, CurrentError> {
        let raw = self.read_latest_raw(read_buf).await?;
        Ok(self.decoder.decode(raw))
    }

    pub async fn read_latest_raw(
        &mut self,
        read_buf: &mut [u16; CURRENT_READ_WORDS],
    ) -> Result<CurrentRaw, CurrentError> {
        self.adc
            .read(read_buf)
            .await
            .map_err(|_| CurrentError::Overrun)?;

        let frame = &read_buf[CURRENT_READ_WORDS - CURRENT_FRAME_WORDS..CURRENT_READ_WORDS];
        Ok(CurrentRaw {
            vbus: frame[0],
            ia: frame[1],
            ib: frame[2],
            ic: frame[3],
        })
    }

    pub fn clear(&mut self) {
        self.adc.clear();
    }
}

pub fn tim1_trgo2_regular_trigger() -> ConversionTrigger {
    ConversionTrigger {
        channel: ADC1_REGULAR_TRIGGER_TIM1_TRGO2,
        edge: Exten::RISING_EDGE,
    }
}

pub fn configure_tim1_trgo2_update() {
    embassy_stm32::pac::TIM1
        .cr2()
        .modify(|reg| reg.set_mms2(Mms2::UPDATE));
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PhaseCurrents {
    pub ia: f32,
    pub ib: f32,
    pub ic: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CurrentSample {
    pub phase: PhaseCurrents,
    pub vbus: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct CurrentConfig {
    pub amp_per_lsb: f32,
    pub vbus_divider_ratio: f32,
    pub adc_vref: f32,
    pub adc_max: f32,
}

impl Default for CurrentConfig {
    fn default() -> Self {
        Self {
            amp_per_lsb: 0.002,
            vbus_divider_ratio: 2.2 / (1.2 + 12.0),
            adc_vref: 3.3,
            adc_max: 4095.0,
        }
    }
}

pub struct CurrentDecoder {
    config: CurrentConfig,
    offsets: [f32; 3],
}

impl CurrentDecoder {
    pub fn new(config: CurrentConfig) -> Self {
        Self {
            config,
            offsets: [2048.0; 3],
        }
    }

    pub fn set_offsets(&mut self, offsets: [f32; 3]) {
        self.offsets = offsets;
    }

    pub fn offsets(&self) -> [f32; 3] {
        self.offsets
    }

    pub fn decode(&self, raw: CurrentRaw) -> CurrentSample {
        let phase = PhaseCurrents {
            ia: (raw.ia as f32 - self.offsets[0]) * self.config.amp_per_lsb,
            ib: (raw.ib as f32 - self.offsets[1]) * self.config.amp_per_lsb,
            ic: (raw.ic as f32 - self.offsets[2]) * self.config.amp_per_lsb,
        };

        let v_adc = raw.vbus as f32 * (self.config.adc_vref / self.config.adc_max);
        let vbus = v_adc / self.config.vbus_divider_ratio.max(0.001);

        CurrentSample { phase, vbus }
    }
}

pub struct OffsetCalibrator {
    accum: [u32; 3],
    samples: u32,
    target_samples: u32,
}

impl OffsetCalibrator {
    pub fn new(target_samples: u32) -> Self {
        Self {
            accum: [0; 3],
            samples: 0,
            target_samples,
        }
    }

    pub fn push(&mut self, raw: CurrentRaw) -> Option<[f32; 3]> {
        self.accum[0] += raw.ia as u32;
        self.accum[1] += raw.ib as u32;
        self.accum[2] += raw.ic as u32;
        self.samples += 1;

        if self.samples >= self.target_samples {
            let n = self.samples as f32;
            Some([
                self.accum[0] as f32 / n,
                self.accum[1] as f32 / n,
                self.accum[2] as f32 / n,
            ])
        } else {
            None
        }
    }
}
