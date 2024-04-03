//! Basic component structs
use embedded_hal::digital::PinState;
use rp2040_hal::gpio::{bank0::{Gpio6, Gpio7, Gpio8}, FunctionNull, FunctionSio, Pin, PullDown, SioOutput};
use crate::components::StatusLedStates::{Alert, Normal};

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

pub struct StatusLed {
    pub state: StatusLedStates,
    normal_led: Pin<Gpio6, FunctionSio<SioOutput>, PullDown>,
    alert_led: Pin<Gpio7, FunctionSio<SioOutput>, PullDown>,
    error_led: Pin<Gpio8, FunctionSio<SioOutput>, PullDown>,
}

/// Init LED GPIO pins for hnadling via interrupt
impl StatusLed {
    pub fn init(
        normal_led: Pin<Gpio6, FunctionNull, PullDown>,
        alert_led: Pin<Gpio7, FunctionNull, PullDown>,
        error_led: Pin<Gpio8, FunctionNull, PullDown>,
    ) -> Self {
        Self {
            state: Alert,
            normal_led: normal_led.into_push_pull_output_in_state(PinState::Low),
            alert_led: alert_led.into_push_pull_output_in_state(PinState::High),
            error_led: error_led.into_push_pull_output_in_state(PinState::Low),
        }
    }
}
