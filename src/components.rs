//! Basic component structs
use cortex_m::singleton;
use critical_section::CriticalSection;
use defmt::{debug, error, warn};
use embedded_hal::digital::{OutputPin, PinState};
use rp2040_hal::gpio::{
    bank0::{Gpio6, Gpio7, Gpio8},
    FunctionNull, FunctionSio, Pin, PullDown, SioOutput,
};

use crate::interrupt::{BUFFERS, STATUS_LEDS};

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
    /// Panic message if no LEDs have been configured.
    pub const NO_LED_PANIC_MSG: &'static str =
        "Unable to display state due to non-configured LEDs, or not available in mutex";
    const RESET_MSG: &'static str = "\nSystem must be power cycled to restore normal operation.";

    pub fn init(
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

    /// Set error state within a [`CriticalSection`]
    pub fn set_error(cs: CriticalSection, message: Option<&str>) {
        let status = STATUS_LEDS.take(cs).expect(Self::NO_LED_PANIC_MSG);

        if let Some(msg_text) = message {
            error!(
                "Error encountered during operation:\n{}{}",
                msg_text,
                Self::RESET_MSG
            );
        } else {
            error!(
                "Unknown error encountered during operation.{}",
                Self::RESET_MSG
            );
        }

        match status.state {
            StatusLedStates::Normal => status.normal_led.set_low().unwrap(),
            StatusLedStates::Alert => status.alert_led.set_low().unwrap(),
            StatusLedStates::Error | StatusLedStates::Disabled => {}
        };

        status.error_led.set_high().unwrap();
        STATUS_LEDS.replace(cs, Some(status));
    }
}

/// Monotonic counter indicating the position of averaged samples.
#[derive(Default, Debug, Ord, PartialOrd, Eq, PartialEq, Copy, Clone)]
pub struct SampleCounter(u32);
impl SampleCounter {
    /// Get current counter value
    pub fn get(&self) -> u32 {
        self.0
    }

    /// Increment counter (mainly used by [`Buffers.current_sample`](Buffers). An error will be
    /// raised if any counter reaches [`u32::MAX`].
    pub fn increment(&mut self) {
        if self.0.checked_add(1).is_none() {
            critical_section::with(|cs| {
                debug!("critical_section: counter set_error overflow");
                StatusLedMulti::set_error(
                    cs,
                    Some("No ADC transfer in progress! Unable to collect latest readings"),
                )
            })
        }
    }
}

/// Various buffers used for managing signal samples
pub struct Buffers {
    /// Records up to 45k averaged samples (90 s with 2 ms averaging) to determine if a detection event occurred
    longterm_buffer: [u8; 45000],
    /// Counter for the most recent sample added to
    current_sample: SampleCounter,
    /// Indicates position time stamps for up to 10 recent detection events, comparable with `current_sample`
    detection_events: [SampleCounter; 10],
    /// A potential detection event has been recorded, and the system is awaiting a second average sample
    await_confirm: bool,
}

impl Buffers {
    /// Initial averaged difference used for detecting contact.
    ///
    /// Ex. a trigger delta of 128 on a 3.3V signal requires that the average voltage range has
    /// decreased by approximately 1.65V.
    const INIT_TRIGGER_DELTA: u8 = 160;
    /// Initial averaged difference to restore [`StatusLedStates::Normal`].
    /// 
    /// This is the increase in voltage relative to the last detection event.
    const INIT_RESET_DELTA: u8 = 100; 
    /// Panic message raised if buffers are not available
    pub const NO_BUFFER_PANIC_MSG: &'static str =
        "Buffers have not been initialized or are not currently available in mutex";

    /// Initialize [`BUFFERS`]
    pub fn init() {
        match singleton!(:Buffers = Self {
            longterm_buffer: [0u8; 45000],
            current_sample: SampleCounter::default(),
            detection_events: [SampleCounter::default(); 10],
            await_confirm: false
        }) {
            Some(init_buffers) => {
                debug!("critical_section: init buffers");
                critical_section::with(|cs| BUFFERS.replace(cs, Some(init_buffers)));
            }
            None => warn!("Buffers have already been initiated"),
        }
    }

    /// Insert a new sample at the head
    pub fn insert(&mut self, sample: u8) {
        let new_head = (self.current_sample.get() + 1) as usize % self.longterm_buffer.len();
        self.longterm_buffer[new_head] = sample;
    }

    /// Analyze the most recent data to determine if a contact event has occurred.
    ///
    /// Also updates the record of recent detection events
    pub fn contact_detected(&mut self) -> bool {
        
        
        todo!()
    }
    
    /// Analyze the most recent data and contact events to determine when contact ends
    pub fn end_contact(&mut self) -> bool {
        todo!()
    }
}

/// Creates a buffer for ADC DMA transfers
pub fn create_avg_buffer() -> Option<&'static mut [u8; 4000]> {
    singleton!(: [u8; 4000] = [0u8; 4000])
}
