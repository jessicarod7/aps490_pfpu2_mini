//! Configuration for system state and status LED control

// SPDX-License-Identifier: Apache-2.0

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

/// System states, expressed by LEDs colours
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

    /// Set [`StatusLedStates::Normal`] within a [`CriticalSection`]
    fn set_normal(cs: CriticalSection, message: Option<&str>);
    /// Set [`StatusLedStates::Alert`] within a [`CriticalSection`]
    fn set_alert(cs: CriticalSection, message: Option<DetectionMsg>);
    /// Set [`StatusLedStates::Error`] within a [`CriticalSection`]
    fn set_error(cs: CriticalSection, message: Option<&str>);
    /// Set [`StatusLedStates::Disabled`] within a [`CriticalSection`]
    fn set_disabled(cs: CriticalSection, message: Option<&str>);
    /// Pause signal generation, readings, and interrupts when disabled or error raised
    fn pause_detection(cs: CriticalSection);
    /// Resume components with normal operation
    fn resume_detection(cs: CriticalSection);
}

/// Directly controls the LEDs.
pub trait LedControl {
    /// Initialize LEDs
    fn init(
        gpio6: Pin<Gpio6, FunctionNull, PullDown>,
        gpio7: Pin<Gpio7, FunctionNull, PullDown>,
        gpio8: Pin<Gpio8, FunctionNull, PullDown>,
    ) -> Option<&'static mut impl StatusLed>;
    /// Set the LEDs to match the current state. Any internal state should also be set.
    ///
    /// Returns `new_state` for convenience.
    fn set_led(
        &mut self,
        old_state: &StatusLedStates,
        new_state: StatusLedStates,
    ) -> StatusLedStates;
}

/// Controls the status LEDs on separate pins
pub struct StatusLedBase<C>
where
    C: LedControl,
{
    /// Current LED state
    pub state: StatusLedStates,
    /// Controller for lights
    pub ctrl: C,
}

impl<C: LedControl> StatusLed for StatusLedBase<C> {
    fn set_normal(cs: CriticalSection, message: Option<&str>) {
        let status = STATUS_LEDS.take(cs).expect(Self::NO_LED_PANIC_MSG);
        if let Some(detection_msg) = message {
            info!("Resuming normal detection: {}", detection_msg);
        } else {
            warn!("State changed to normal");
        }

        match status.state {
            StatusLedStates::Error | StatusLedStates::Disabled => Self::resume_detection(cs),
            StatusLedStates::Normal | StatusLedStates::Alert => {}
        }
        status.state = status.ctrl.set_led(&status.state, StatusLedStates::Normal);
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
            StatusLedStates::Error | StatusLedStates::Disabled => Self::resume_detection(cs),
            StatusLedStates::Normal | StatusLedStates::Alert => {}
        };
        status.state = status.ctrl.set_led(&status.state, StatusLedStates::Alert);
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
            StatusLedStates::Normal | StatusLedStates::Alert => Self::pause_detection(cs),
            StatusLedStates::Error | StatusLedStates::Disabled => {}
        };
        status.state = status.ctrl.set_led(&status.state, StatusLedStates::Error);
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
            StatusLedStates::Normal | StatusLedStates::Alert => Self::pause_detection(cs),
            StatusLedStates::Error | StatusLedStates::Disabled => {}
        };
        status.state = status
            .ctrl
            .set_led(&status.state, StatusLedStates::Disabled);
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

/// Common anode RGB, mapped as follows:
/// - [`Gpio6`] is the red control
/// - [`Gpio7`] is the green control
/// - [`Gpio8`] is the blue control (initialized high but otherwise unused)
#[cfg(any(doc, feature = "rgba_status"))]
pub struct Rgba {
    /// Used in [`StatusLedStates::Alert`] and [`StatusLedStates::Error`]
    red_led: Pin<Gpio6, FunctionSio<SioOutput>, PullDown>,
    /// Used in [`StatusLedStates::Normal`] and [`StatusLedStates::Error`]
    green_led: Pin<Gpio7, FunctionSio<SioOutput>, PullDown>,
    /// Initialized but unused
    #[allow(dead_code)]
    blue_led: Pin<Gpio8, FunctionSio<SioOutput>, PullDown>,
}

#[cfg(any(doc, feature = "rgba_status"))]
impl LedControl for Rgba {
    fn init(
        gpio6: Pin<Gpio6, FunctionNull, PullDown>,
        gpio7: Pin<Gpio7, FunctionNull, PullDown>,
        gpio8: Pin<Gpio8, FunctionNull, PullDown>,
    ) -> Option<&'static mut StatusLedBase<Rgba>> {
        singleton!(: StatusLedBase<Rgba> = StatusLedBase {
            state: StatusLedStates::Alert,
            ctrl: Rgba {
                red_led: gpio6.into_push_pull_output_in_state(PinState::Low),
                green_led: gpio7.into_push_pull_output_in_state(PinState::Low),
                blue_led: gpio8.into_push_pull_output_in_state(PinState::High),
            }
        })
    }

    fn set_led(
        &mut self,
        old_state: &StatusLedStates,
        new_state: StatusLedStates,
    ) -> StatusLedStates {
        match old_state {
            StatusLedStates::Normal => self.green_led.set_high().unwrap(),
            StatusLedStates::Alert => {
                self.red_led.set_high().unwrap();
                self.green_led.set_high().unwrap();
            }
            StatusLedStates::Error => self.red_led.set_high().unwrap(),
            StatusLedStates::Disabled => {}
        }

        match new_state {
            StatusLedStates::Normal => self.green_led.set_low().unwrap(),
            StatusLedStates::Alert => {
                self.red_led.set_low().unwrap();
                self.green_led.set_low().unwrap();
            }
            StatusLedStates::Error => self.green_led.set_low().unwrap(),
            StatusLedStates::Disabled => {}
        }

        new_state
    }
}

/// Triple LED status, mapped as follows:
/// - [`Gpio6`] is a green LED
/// - [`Gpio7`] is a yellow LED
/// - [`Gpio8`] is a red LED
#[cfg(any(doc, feature = "triple_status"))]
pub struct Triple {
    normal_led: Pin<Gpio6, FunctionSio<SioOutput>, PullDown>,
    alert_led: Pin<Gpio7, FunctionSio<SioOutput>, PullDown>,
    error_led: Pin<Gpio8, FunctionSio<SioOutput>, PullDown>,
}

#[cfg(any(doc, feature = "triple_status"))]
impl LedControl for Triple {
    fn init(
        gpio6: Pin<Gpio6, FunctionNull, PullDown>,
        gpio7: Pin<Gpio7, FunctionNull, PullDown>,
        gpio8: Pin<Gpio8, FunctionNull, PullDown>,
    ) -> Option<&'static mut StatusLedBase<Self>> {
        singleton!(: StatusLedBase<Triple> = StatusLedBase {
            state: StatusLedStates::Alert,
            ctrl: Triple {
                normal_led: gpio6.into_push_pull_output_in_state(PinState::Low),
                alert_led: gpio7.into_push_pull_output_in_state(PinState::High),
                error_led: gpio8.into_push_pull_output_in_state(PinState::Low),
            }
        })
    }

    fn set_led(
        &mut self,
        old_state: &StatusLedStates,
        new_state: StatusLedStates,
    ) -> StatusLedStates {
        match old_state {
            StatusLedStates::Normal => self.normal_led.set_low().unwrap(),
            StatusLedStates::Alert => self.alert_led.set_low().unwrap(),
            StatusLedStates::Error => self.error_led.set_low().unwrap(),
            StatusLedStates::Disabled => {}
        }
        match new_state {
            StatusLedStates::Normal => self.normal_led.set_high().unwrap(),
            StatusLedStates::Alert => self.alert_led.set_high().unwrap(),
            StatusLedStates::Error => self.error_led.set_high().unwrap(),
            StatusLedStates::Disabled => {}
        }

        new_state
    }
}
