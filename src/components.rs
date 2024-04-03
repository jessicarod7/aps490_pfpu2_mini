//! Basic component structs
use cortex_m::singleton;
use embedded_hal::digital::PinState;
use rp2040_hal::gpio::{
    bank0::{Gpio6, Gpio7, Gpio8},
    FunctionNull, FunctionSio, Pin, PullDown, SioOutput,
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

/// Controls the status LEDs on separate pins
pub struct StatusLedMulti {
    /// Current LED state
    pub state: StatusLedStates,
    normal_led: Pin<Gpio6, FunctionSio<SioOutput>, PullDown>,
    alert_led: Pin<Gpio7, FunctionSio<SioOutput>, PullDown>,
    error_led: Pin<Gpio8, FunctionSio<SioOutput>, PullDown>,
}

/// Init LED GPIO pins for hnadling via interrupt
impl StatusLedMulti {
    pub fn init(
        normal_led: Pin<Gpio6, FunctionNull, PullDown>,
        alert_led: Pin<Gpio7, FunctionNull, PullDown>,
        error_led: Pin<Gpio8, FunctionNull, PullDown>,
    ) -> Self {
        Self {
            state: StatusLedStates::Alert,
            normal_led: normal_led.into_push_pull_output_in_state(PinState::Low),
            alert_led: alert_led.into_push_pull_output_in_state(PinState::High),
            error_led: error_led.into_push_pull_output_in_state(PinState::Low),
        }
    }
}

pub fn create_avg_buffer() -> Option<&'static mut [u8; 4000]> {
    singleton!(: [u8; 4000] = [0u8; 4000])
}