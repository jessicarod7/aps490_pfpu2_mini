use cortex_m::singleton;
use defmt::{debug, warn, Format, Formatter};
#[allow(unused_imports)]
use defmt::trace;

use crate::{
    components::{StatusLed, StatusLedMulti},
    interrupt::BUFFERS,
};

/// Number of samples stored in the long-term buffer. Should be a multiple of 250 for tracing purposes
///
/// Currently set to 45k averaged samples (90 s with 2 ms averaging)
pub const LONGTERM_SIZE: usize = 45000;

/// Index of a detection event, combined with voltage difference
pub type DetectionEvent = (SampleCounter, u8);

/// Monotonic counter indicating the position of averaged samples.
#[derive(Default, Debug, Ord, PartialOrd, Eq, PartialEq, Copy, Clone)]
pub struct SampleCounter(usize);

impl SampleCounter {
    /// Get current counter value
    pub fn get_counter(&self) -> usize {
        self.0
    }

    /// Increment counter (mainly used by [`Buffers.current_sample`](Buffers)). An error will be
    /// raised if any counter reaches [`u32::MAX`].
    pub fn increment(&mut self) {
        if self.0.checked_add(1).is_none() {
            critical_section::with(|cs| {
                debug!("critical_section: counter set_error overflow");
                #[cfg(feature = "multi_status")]
                StatusLedMulti::set_error(
                    cs,
                    Some("No ADC transfer in progress! Unable to collect latest readings"),
                )
            })
        }
    }

    /// Add with defined wrapping. Result will be within range \[0, `limit` - 1\].
    ///
    /// Used to index [`Buffers.longterm_buffer`](Buffers)
    pub fn wrapping_counter_add(&self, rhs: usize, limit: usize) -> usize {
        if self.0 + rhs >= limit {
            rhs - (limit - self.0)
        } else {
            self.0 + rhs
        }
    }

    /// Subtract with defined wrapping. Limit will be within range \[0, `limit` - 1\].
    ///
    /// Used to index [`Buffers.longterm_buffer`](Buffers)
    pub fn wrapping_counter_sub(&self, rhs: usize, limit: usize) -> usize {
        if self.0.wrapping_sub(rhs) >= limit {
            limit - (rhs - self.0)
        } else {
            self.0 - rhs
        }
    }
}

/// Various buffers used for managing signal samples
pub struct Buffers {
    /// Records samples for long-term and adaptive detection.
    longterm_buffer: [u8; LONGTERM_SIZE],
    /// Counter for the most recent sample added to
    current_sample: SampleCounter,
    /// Rotates position time stamps for up to 10 recent detection events, comparable with `current_sample`.
    /// Most recent event is stored at index 0
    detection_events: [Option<DetectionEvent>; 10],
    /// A potential detection event or event clear has been recorded, and the system is awaiting a
    /// second sample
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
    const INIT_RESTORE_DELTA: u8 = 100;
    /// Panic message raised if buffers are not available
    pub const NO_BUFFER_PANIC_MSG: &'static str =
        "Buffers have not been initialized or are not currently available in mutex";

    /// Initialize [`BUFFERS`]
    pub fn init() {
        match singleton!(:Buffers = Self {
            longterm_buffer: [0u8; LONGTERM_SIZE],
            current_sample: SampleCounter::default(),
            detection_events: [None; 10],
            await_confirm: false
        }) {
            Some(init_buffers) => {
                debug!("critical_section: init buffers");
                critical_section::with(|cs| BUFFERS.replace(cs, Some(init_buffers)));
            }
            None => warn!("Buffers have already been initiated"),
        }
    }

    /// Returns [`SampleCounter::get_counter`] wrapped to [`LONGTERM_SIZE`]
    pub fn current_wrapped(&self) -> SampleCounter {
        SampleCounter(self.current_sample.get_counter() % LONGTERM_SIZE) 
    }
    
    /// Insert a new sample at the head
    pub fn insert(&mut self, sample: u8) {
        let new_head = self.current_wrapped().wrapping_counter_add(1, LONGTERM_SIZE);
        self.longterm_buffer[new_head] = sample;
        self.current_sample.increment();
        
        // Comment out cfg to trace all samples
        #[cfg(feature = "")]
        if self.current_sample.get_counter() % 250 == 0 {
            let first_sample = self.current_wrapped().wrapping_counter_sub(250, LONGTERM_SIZE);
            let new_samples = self.longterm_buffer.get(first_sample..first_sample+250).unwrap();
            trace!("Here are the last 250 samples:\n{}", new_samples)
        }
    }

    /// Analyze the most recent data to determine if a contact event has occurred.
    ///
    /// Also updates the record of recent detection events
    pub fn detect_contact(&mut self) {
        if !self.await_confirm {
            // First contact check
            let prev_sample = self.current_sample.wrapping_counter_sub(1, LONGTERM_SIZE);
            if self.longterm_buffer[prev_sample]
                - self.longterm_buffer[self.current_sample.get_counter()]
                >= Self::INIT_TRIGGER_DELTA
            {
                self.await_confirm = true;
            }
        } else {
            // Validation contact check
            let prev_high_sample = self.current_sample.wrapping_counter_sub(2, LONGTERM_SIZE);
            if self.longterm_buffer[prev_high_sample]
                - self.longterm_buffer[self.current_sample.get_counter()]
                >= Self::INIT_TRIGGER_DELTA
            {
                // Contact detected!
                critical_section::with(|cs| {
                    #[cfg(feature = "multi_status")]
                    StatusLedMulti::set_alert(cs, Some(DetectionMsg::create(self)))
                });
                self.add_detection_event();
            }

            // Always reset on validation check
            self.await_confirm = false;
        }
    }

    /// Analyze the most recent data and contact events to determine when contact ends
    ///
    /// A detection [`StatusLedStates::Alert`] will not clear until at least 150 samples (300 milliseconds with 2 ms
    /// averaging) have been recorded. This ensures the operator will see the LED light up.
    pub fn detect_end_contact(&mut self) {
        if let Some(last_detection) = self.detection_events[0] {
            if self
                .current_sample
                .wrapping_counter_sub(last_detection.0.get_counter(), LONGTERM_SIZE)
                >= 300
            {
                if !self.await_confirm
                    && self.longterm_buffer[self.current_sample.get_counter()] - last_detection.1
                        >= Self::INIT_RESTORE_DELTA
                {
                    // First clear check
                    self.await_confirm = true;
                }
            } else if self.await_confirm {
                // Validation clear check
                if self.longterm_buffer[self.current_sample.get_counter()] - last_detection.1
                    >= Self::INIT_RESTORE_DELTA
                {
                    // Contact cleared!
                    critical_section::with(|cs| {
                        #[cfg(feature = "multi_status")]
                        StatusLedMulti::set_normal(cs, None)
                    })
                }

                self.await_confirm = false;
            }
        } else {
            warn!("End contact detection was called before any detection events have occurred.");
        }
    }

    /// Shortcut to return index of a successful detection sample.
    ///
    ///```
    /// static BUFFERS = Buffers::init();
    ///
    /// BUFFERS.insert(12);
    /// assert_eq!(BUFFERS.detection_idx(), BUFFERS.current_sample.get_counter() - 1)
    ///```
    pub fn detection_idx(&self) -> usize {
        self.current_sample.get_counter() - 1
    }

    /// Add an entry to the `detection_events` array, based on the penultimate sample.
    fn add_detection_event(&mut self) {
        self.detection_events.rotate_right(1);
        self.detection_events[0] = Some((
            self.current_sample,
            self.longterm_buffer[self.current_sample.get_counter()],
        ));
    }
}

/// Newtype to send formatted error messages when [`Buffers::detect_contact`] is successful.
pub struct DetectionMsg(SampleCounter);

impl DetectionMsg {
    /// Create a detection message:
    ///
    /// > "contact detected on sample {[`Buffers::detection_idx`]}! Adding to detection events"`
    fn create(buffer: &Buffers) -> Self {
        Self(SampleCounter(buffer.detection_idx()))
    }
}

impl Format for DetectionMsg {
    fn format(&self, fmt: Formatter) {
        defmt::write!(
            fmt,
            "contact detected on sample {}! Adding to detection events",
            self.0.get_counter()
        )
    }
}

/// Creates a buffer for ADC DMA transfers
pub fn create_avg_buffer() -> Option<&'static mut [u8; 4000]> {
    singleton!(: [u8; 4000] = [0u8; 4000])
}
