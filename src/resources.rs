use assign_resources::assign_resources;
use embassy_stm32::{Peri, peripherals};

assign_resources! {
    motor: MotorResources {
        adc: ADC1 = MotorAdc,
        adc_dma: DMA1_CH1 = MotorAdcDma,
        vbus: PA0 = VbusAdcPin,
        phase_a_current: PA1 = PhaseACurrentPin,
        phase_b_current: PA2 = PhaseBCurrentPin,
        phase_c_current: PA3 = PhaseCCurrentPin,
        pwm_timer: TIM1 = MotorPwmTimer,
        phase_c_pwm: PA8 = PhaseCPwmPin,
        phase_b_pwm: PA9 = PhaseBPwmPin,
        phase_a_pwm: PA10 = PhaseAPwmPin,
        drv_fault: PB0 = DrvFaultPin,
        drv_en: PB1 = DrvEnablePin,
        pwm_en: PB2 = PwmEnablePin,
    }

    rs485: Rs485Resources {
        usart: USART3 = Rs485Uart,
        tx: PC10 = Rs485TxPin,
        rx: PC11 = Rs485RxPin,
        tx_dma: DMA1_CH4 = Rs485TxDma,
        rx_dma: DMA1_CH2 = Rs485RxDma,
        dir: PB4 = Rs485DirPin,
    }

    angle_sensor: AngleSensorResources {
        spi: SPI1 = AngleSensorSpi,
        nss: PA4 = AngleSensorNssPin,
        sck: PA5 = AngleSensorSckPin,
        miso: PA6 = AngleSensorMisoPin,
        mosi: PA7 = AngleSensorMosiPin,
    }

    oled: OledResources {
        spi: SPI2 = OledSpi,
        nss: PB12 = OledNssPin,
        sck: PB13 = OledSckPin,
        mosi: PB15 = OledMosiPin,
        rst: PB11 = OledRstPin,
        dc: PB14 = OledDcPin,
    }

    i2c: I2cResources {
        i2c: I2C1 = BoardI2c,
        scl: PA15 = BoardI2cSclPin,
        sda: PB7 = BoardI2cSdaPin,
    }

    tim3_pwm: Tim3PwmResources {
        timer: TIM3 = Tim3PwmTimer,
        ch2: PB5 = Tim3Ch2Pin,
        dma: DMA1_CH3 = Tim3Ch2Dma,
    }

    board_io: BoardIoResources {
        sys_sw: PA12 = SysSwPin,
        debug: PB9 = DebugPin,
    }
}
