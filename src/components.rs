//! Basic component structs
use cortex_m::{prelude::_embedded_hal_PwmPin, singleton};
use critical_section::CriticalSection;
use defmt::{debug, error, info, warn};
use embedded_hal::digital::{OutputPin, PinState};
use rp2040_hal::{
    dma,
    dma::SingleChannel,
    gpio::{
        bank0::{Gpio6, Gpio7, Gpio8},
        FunctionNull, FunctionSio, Pin, PullDown, SioOutput,
    },
};

use crate::{
    buffer::DetectionMsg,
    interrupt::{READINGS_FIFO, SIGNAL_CONF, SIGNAL_GEN, STATUS_LEDS},
};

/// All states for LEDs
pub enum StatusLedStates {
    /// Green
    Normal,
    /// Yellow
    Alert,
    /// Red
    Error,
    /// None illuminated
    Disabled,
}

/// System status is communicated via a trio of LED colours (see [`StatusLedStates`]).
pub trait StatusLed {
    /// Panic message if no LEDs have been configured.
    const NO_LED_PANIC_MSG: &'static str =
        "Unable to display state due to non-configured LEDs, or not available in mutex";
    /// Message displayed if system enters [`StatusLedStates::Error`]
    const RESET_MSG: &'static str = "\nSystem must be power cycled to restore normal operation.";
    /// Message displayed if system enters [`StatusLedStates::Disabled`]
    const DISABLE_MSG: &'static str = "\nToggle the disable switch to resume normal operation.";

    /// Initialize LEDs
    fn init(
        normal_led: Pin<Gpio6, FunctionNull, PullDown>,
        alert_led: Pin<Gpio7, FunctionNull, PullDown>,
        error_led: Pin<Gpio8, FunctionNull, PullDown>,
    ) -> Option<&'static mut Self>;
    /// Set [`StatusLedStates::Normal`] within a [`CriticalSection`]
    fn set_normal(cs: CriticalSection, message: Option<&str>);
    /// Set [`StatusLedStates::Alert`] within a [`CriticalSection`]
    fn set_alert(cs: CriticalSection, message: Option<DetectionMsg>);
    /// Set [`StatusLedStates::Error`] within a [`CriticalSection`]
    fn set_error(cs: CriticalSection, message: Option<&str>);
    /// Set [`StatusLedStates::Disabled`] within a [`CriticalSection`]
    fn set_disabled(cs: CriticalSection, message: Option<&str>);
    /// Pause signal generation, readings, and interrupts when disabled or error raised.
    fn pause_detection(cs: CriticalSection);
    /// Resume components with normal operation
    fn resume_detection(cs: CriticalSection);
}

/// Controls the status LEDs on separate pins. Intended for operation with three separate LEDs
pub struct StatusLedMulti {
    /// Current LED state
    pub state: StatusLedStates,
    /// Typically green
    normal_led: Pin<Gpio6, FunctionSio<SioOutput>, PullDown>,
    /// Typically yellow
    alert_led: Pin<Gpio7, FunctionSio<SioOutput>, PullDown>,
    /// Typically red
    error_led: Pin<Gpio8, FunctionSio<SioOutput>, PullDown>,
}

impl StatusLed for StatusLedMulti {
    fn init(
        normal_led: Pin<Gpio6, FunctionNull, PullDown>,
        alert_led: Pin<Gpio7, FunctionNull, PullDown>,
        error_led: Pin<Gpio8, FunctionNull, PullDown>,
    ) -> Option<&'static mut Self> {
        singleton!(: StatusLedMulti = Self {
            state: StatusLedStates::Alert,
            normal_led: normal_led.into_push_pull_output_in_state(PinState::Low),
            alert_led: alert_led.into_push_pull_output_in_state(PinState::High),
            error_led: error_led.into_push_pull_output_in_state(PinState::Low),
        })
    }

    fn set_normal(cs: CriticalSection, message: Option<&str>) {
        let status = STATUS_LEDS.take(cs).expect(Self::NO_LED_PANIC_MSG);
        if let Some(detection_msg) = message {
            info!("Resuming normal detection: {}", detection_msg);
        } else {
            warn!("State changed to normal");
        }

        match status.state {
            StatusLedStates::Alert => {
                status.alert_led.set_low().unwrap();
                info!("Previous detection event cleared")
            }
            StatusLedStates::Error => {
                status.error_led.set_low().unwrap();
                Self::resume_detection(cs);
            }
            StatusLedStates::Disabled => Self::resume_detection(cs),
            StatusLedStates::Normal => {}
        };
        status.normal_led.set_high().unwrap();
        status.state = StatusLedStates::Normal;
        STATUS_LEDS.replace(cs, Some(status));
    }

    fn set_alert(cs: CriticalSection, message: Option<DetectionMsg>) {
        let status = STATUS_LEDS.take(cs).expect(Self::NO_LED_PANIC_MSG);
        if let Some(detection_msg) = message {
            info!("{}", detection_msg);
        } else {
            warn!("Unknown alert raised!");
        }

        match status.state {
            StatusLedStates::Normal => status.normal_led.set_low().unwrap(),
            StatusLedStates::Error => {
                status.error_led.set_low().unwrap();
                Self::resume_detection(cs);
            }
            StatusLedStates::Disabled => Self::resume_detection(cs),
            StatusLedStates::Alert => {}
        };
        status.alert_led.set_high().unwrap();
        status.state = StatusLedStates::Alert;
        STATUS_LEDS.replace(cs, Some(status));
    }

    fn set_error(cs: CriticalSection, message: Option<&str>) {
        let status = STATUS_LEDS.take(cs).expect(Self::NO_LED_PANIC_MSG);
        if let Some(msg_text) = message {
            error!(
                "Error encountered during operation:\n{=str}{=str}",
                msg_text,
                Self::RESET_MSG
            );
        } else {
            error!(
                "Unknown error encountered during operation.{=str}",
                Self::RESET_MSG
            );
        }

        match status.state {
            StatusLedStates::Normal => {
                status.normal_led.set_low().unwrap();
                Self::pause_detection(cs);
            }
            StatusLedStates::Alert => {
                status.alert_led.set_low().unwrap();
                Self::pause_detection(cs);
            }
            StatusLedStates::Error | StatusLedStates::Disabled => {}
        };
        status.error_led.set_high().unwrap();
        status.state = StatusLedStates::Error;
        STATUS_LEDS.replace(cs, Some(status));
    }

    fn set_disabled(cs: CriticalSection, message: Option<&str>) {
        let status = STATUS_LEDS.take(cs).expect(Self::NO_LED_PANIC_MSG);
        if let Some(msg_text) = message {
            info!(
                "System has been disabled:\n{=str}{=str}",
                msg_text,
                Self::DISABLE_MSG
            );
        } else {
            info!("System has been disabled.{=str}", Self::DISABLE_MSG);
        }

        match status.state {
            StatusLedStates::Normal => {
                status.normal_led.set_low().unwrap();
                Self::pause_detection(cs);
            }
            StatusLedStates::Alert => {
                status.alert_led.set_low().unwrap();
                Self::pause_detection(cs);
            }
            StatusLedStates::Error => status.error_led.set_low().unwrap(),
            StatusLedStates::Disabled => {}
        };
        status.state = StatusLedStates::Disabled;
        STATUS_LEDS.replace(cs, Some(status));
    }

    fn pause_detection(cs: CriticalSection) {
        debug!("Disabling signal generation");
        let mut signal_pwm = SIGNAL_GEN.take(cs).expect("Unable to access PWM controls");
        signal_pwm.disable();
        SIGNAL_GEN.replace(cs, Some(signal_pwm));

        debug!("Disabling FIFO readings/interrupts");
        let fifo_transfer = READINGS_FIFO.take(cs).expect("Unable to access ADC FIFO");
        SIGNAL_CONF.replace(cs, Some(fifo_transfer.wait()));
    }

    fn resume_detection(cs: CriticalSection) {
        debug!("Restoring signal generation");
        let mut signal_pwm = SIGNAL_GEN.take(cs).expect("Unable to access PWM controls");
        signal_pwm.enable();
        SIGNAL_GEN.replace(cs, Some(signal_pwm));

        debug!("Restoring ADC readings and interrupts");
        let config = SIGNAL_CONF.replace(cs, None);
        if let Some(mut inner) = config {
            inner.0.enable_irq0();
            let new_transfer = dma::single_buffer::Config::new(inner.0, inner.1, inner.2);
            READINGS_FIFO.replace(cs, Some(new_transfer.start()));
        } else {
            warn!("Failed to restore FIFO config");
            READINGS_FIFO.replace(cs, None);
        }
    }
}
