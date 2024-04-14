//! Interrupt and exception handlers, plus static [`Mutex`].

// Copyright 2024 Cameron Rodriguez
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use core::{cell::RefCell, cmp::Ordering};

use cortex_m_rt::exception;
use critical_section::Mutex;
#[allow(unused_imports)]
use defmt::trace;
use defmt::{debug, Format};
use embedded_hal::digital::InputPin;
use rp2040_hal::{
    adc::DmaReadTarget,
    dma::{single_buffer, single_buffer::Transfer, Channel, CH0},
    gpio::{bank0::Gpio9, FunctionSio, Pin, PullDown, SioInput},
    pac::interrupt,
    pwm,
    pwm::{FreeRunning, Pwm3, Slice},
};

#[cfg(feature = "rgba_status")]
use crate::components::Rgba;
#[cfg(feature = "triple_status")]
use crate::components::Triple;
use crate::{
    buffer::{Buffers, DetectionMsg},
    components::{StatusLed, StatusLedBase, StatusLedStates},
};

/// Wrapper for [DMA `Transfer`](Transfer)
pub type ReadingsDma = Transfer<Channel<CH0>, DmaReadTarget<u8>, &'static mut [u8; 4000]>;
/// Wrapper for [`DISABLE_SWITCH`]
pub type DisableSwitch = Pin<Gpio9, FunctionSio<SioInput>, PullDown>;
/// Wrapper for [`SIGNAL_GEN`]
pub type SignalPwm = pwm::Channel<Slice<Pwm3, FreeRunning>, pwm::A>;
/// Wrapper for [`SIGNAL_CONF`]
pub type SignalGenConfig = (Channel<CH0>, DmaReadTarget<u8>, &'static mut [u8; 4000]);

/// Status LEDs for access in interrupts. Implementation for feature `rgba_status`.
#[cfg(feature = "rgba_status")]
pub static STATUS_LEDS: Mutex<RefCell<Option<&'static mut StatusLedBase<Rgba>>>> =
    Mutex::new(RefCell::new(None));
/// Status LEDs for access in interrupts. There is a nearly identical implementation for feature
/// `rgba_status`, which does not appear here.
#[cfg(any(doc, feature = "triple_status"))]
pub static STATUS_LEDS: Mutex<RefCell<Option<&'static mut StatusLedBase<Triple>>>> =
    Mutex::new(RefCell::new(None));

///  access in interrupts
pub static READINGS_FIFO: Mutex<RefCell<Option<ReadingsDma>>> = Mutex::new(RefCell::new(None));

/// access when disabling system/ in error state
pub static SIGNAL_GEN: Mutex<RefCell<Option<SignalPwm>>> = Mutex::new(RefCell::new(None));

/// Stores signal config when detection is paused
pub static SIGNAL_CONF: Mutex<RefCell<Option<SignalGenConfig>>> = Mutex::new(RefCell::new(None));

/// Global buffers for analyzing readings
pub static BUFFERS: Mutex<RefCell<Option<&'static mut Buffers>>> = Mutex::new(RefCell::new(None));

/// Global disable switch
pub static DISABLE_SWITCH: Mutex<RefCell<Option<DisableSwitch>>> = Mutex::new(RefCell::new(None));

/// Calculates proper averages aligned with signal timing
#[derive(Copy, Clone, Debug, Default, Hash, Ord, PartialOrd, Eq, PartialEq, Format)]
pub struct AlignedAverages {
    /// The average voltage from the "higher" 2000 measurements
    avg_high: i32,
    /// The average voltage from the "lower" 2000 measurements
    avg_low: i32,
}

impl AlignedAverages {
    /// Takes the partial sums of the samples to calculate the high and low averages for contact
    /// detection.
    ///
    /// Sorting requires an allocator, so this implementation identifies the highest partial sums.
    /// Although this implementation technically allows for non-adjacent partial sums to be matched,
    /// in effect this has little impact as those scenarios result in low overall deltas.
    ///
    /// This would be a good section to rewrite :)
    pub fn align_signal_timing(partial_sums: &[i32; 4]) -> Self {
        let mut avg_high_idx = [4usize; 2];
        let mut avg_high = 0i32;

        let match_sum = partial_sums
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.cmp(b.1))
            .unwrap();
        avg_high_idx[0] = match_sum.0;
        avg_high += match_sum.1;

        let match_sum = partial_sums
            .iter()
            .enumerate()
            .max_by(|a, b| match a.1.cmp(b.1) {
                Ordering::Less | Ordering::Equal => {
                    if avg_high_idx.contains(&b.0) {
                        Ordering::Greater
                    } else {
                        Ordering::Less
                    }
                }
                Ordering::Greater => {
                    if avg_high_idx.contains(&a.0) {
                        Ordering::Less
                    } else {
                        Ordering::Greater
                    }
                }
            })
            .unwrap();
        avg_high_idx[1] = match_sum.0;
        avg_high += match_sum.1;
        avg_high /= 2000;
        #[cfg(feature = "trace_indiv_samples")]
        Self::trace_high_index(&avg_high_idx);

        let avg_low = partial_sums
            .iter()
            .enumerate()
            .filter_map(|(idx, sum)| {
                if !avg_high_idx.contains(&idx) {
                    Some(sum)
                } else {
                    None
                }
            })
            .sum::<i32>()
            / 2000;

        Self { avg_low, avg_high }
    }

    /// Records the two highest measurements from the first four of a 2 ms sample.
    #[cfg(any(doc, feature = "trace_indiv_samples"))]
    pub fn trace_high_index(avg_high_idx: &[usize; 2]) {
        trace!("high indices (mod 4): {}", avg_high_idx);
    }

    /// Calculates the average range of the sample interval
    fn get_delta(&self) -> u8 {
        u8::try_from(self.avg_high - self.avg_low).map_or(255, |avg| avg)
    }
}

/// ISR for reading ADC values and calculating averages
#[interrupt]
fn DMA_IRQ_0() {
    let mut readings_isr: Option<ReadingsDma> = None;
    if readings_isr.is_none() {
        debug!("critical_section: DMA take readings");
        critical_section::with(|cs| readings_isr = READINGS_FIFO.take(cs));
    }

    if let Some(adc_dma_transfer) = readings_isr {
        let (dma_ch, dma_from, avg_buffer) = adc_dma_transfer.wait();

        // Align averages with incoming signals
        let mut partial_sums = [0i32; 4]; // 1000 samples each
        for (idx, partial) in partial_sums.iter_mut().enumerate() {
            *partial = avg_buffer
                .iter()
                .skip(idx)
                .step_by(4)
                .map(|i| *i as i32)
                .sum::<i32>();
        }
        let avgs = AlignedAverages::align_signal_timing(&partial_sums);

        #[cfg(feature = "trace_indiv_samples")]
        trace_indiv_samples(avg_buffer, &avgs);

        // Determine if enough low sample events have occurred
        let sample_avg = avgs.get_delta();
        let mut contact_detected = false;
        let mut reset_detected = false;
        critical_section::with(|cs| {
            debug!("critical_section: dma update and check longterm buffers");
            let buffers = BUFFERS.take(cs).expect(Buffers::NO_BUFFER_PANIC_MSG);
            buffers.insert(sample_avg);

            debug!("critical_section: match status for correct buffer logic");
            let status_leds = STATUS_LEDS.borrow_ref(cs);
            if status_leds.is_some() {
                match status_leds.as_ref().unwrap().state {
                    StatusLedStates::Normal => {
                        if buffers.detect_contact() {
                            contact_detected = true
                        }
                    }
                    StatusLedStates::Alert => {
                        if buffers.detect_end_contact() {
                            reset_detected = true
                        }
                    }
                    StatusLedStates::Error | StatusLedStates::Disabled => {}
                }
            }

            BUFFERS.replace(cs, Some(buffers));
            debug!("exit buffer critical section");
        });
        if contact_detected {
            critical_section::with(|cs| {
                let buffers = BUFFERS.take(cs).unwrap();
                #[cfg(feature = "rgba_status")]
                StatusLedBase::<Rgba>::set_alert(cs, Some(DetectionMsg::create(buffers)));
                #[cfg(feature = "triple_status")]
                StatusLedBase::<Triple>::set_alert(cs, Some(DetectionMsg::create(buffers)));
                BUFFERS.replace(cs, Some(buffers));
            });
        } else if reset_detected {
            critical_section::with(|cs| {
                #[cfg(feature = "rgba_status")]
                StatusLedBase::<Rgba>::set_normal(cs, None);
                #[cfg(feature = "triple_status")]
                StatusLedBase::<Triple>::set_normal(cs, None);
            })
        }

        let new_dma_transfer = single_buffer::Config::new(dma_ch, dma_from, avg_buffer);
        debug!("critical_section: start new DMA transfer");
        critical_section::with(|cs| READINGS_FIFO.replace(cs, Some(new_dma_transfer.start())));
    } else {
        // Report error if FIFO is not active
        critical_section::with(|cs| {
            debug!("critical_section: dma set_error for no active FIFO");
            #[cfg(feature = "rgba_status")]
            StatusLedBase::<Rgba>::set_error(
                cs,
                Some("No ADC transfer in progress! Unable to collect latest readings"),
            );
            #[cfg(feature = "triple_status")]
            StatusLedBase::<Triple>::set_error(
                cs,
                Some("No ADC transfer in progress! Unable to collect latest readings"),
            );
        });
    }
}

/// Records the following information about a 2 ms sample (note all measurements are 8 bits on a
/// <span style="white-space:nowrap;">3.3 V</span> signal):
/// - Maximum voltage recorded
/// - Minimum voltage recorded
/// - Average voltage from higher half
/// - Average voltage from lower half
/// - The first 20 measurements
/// - All unique measurements seen
///
/// Example of a trace:
///
/// ```shell
/// [TRACE] interrupt.rs:59    => max: Some(255) // min: Some(0) // avg1: 127 // avg2: 127
/// -> all_unique samples: [Some(0), Some(1), Some(2), Some(3), None, None, None, None, None, None, None, None, None, None, None, None, Some(16), Some(17), None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, Some(95), None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, Some(140), Some(141), None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, Some(231), Some(232), Some(233), Some(234), Some(235), None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, Some(253), Some(254), Some(255)]
/// ```
#[cfg(any(doc, feature = "trace_indiv_samples"))]
pub fn trace_indiv_samples(avg_buffer: &[u8; 4000], avgs: &AlignedAverages) {
    let unique_samples = avg_buffer.iter().fold([None; 256], |mut acc, s| {
        acc[*s as usize] = Some(s);
        acc
    });
    trace!(
                "max: {} // min: {} // avg_high: {} // avg_low: {} // 20 samples: {}\n-> all_unique samples: {}",
                avg_buffer.iter().max(),
                avg_buffer.iter().min(),
                avgs.avg_high,
                avgs.avg_low,
                avg_buffer.get(0..20).unwrap(),
                unique_samples
            );
}

/// ISR for SysTick, used for checking [`DisableSwitch`]
///
/// Lazily takes ownership of [`DISABLE_SWITCH`] as it will not be used again in the main runtime
/// again.
#[exception]
fn SysTick() {
    static mut DISABLE_SWITCH_ISR: Option<DisableSwitch> = None;

    if DISABLE_SWITCH_ISR.is_none() {
        critical_section::with(|cs| {
            *DISABLE_SWITCH_ISR = DISABLE_SWITCH.take(cs);
        })
    }

    if let Some(switch) = DISABLE_SWITCH_ISR {
        if switch
            .is_high()
            .expect("Unable to check disable switch state")
        {
            critical_section::with(|cs| {
                debug!("critical_section: system disabled by switch");
                #[cfg(feature = "rgba_status")]
                StatusLedBase::<Rgba>::set_disabled(cs, Some("System disabled by switch."));
                #[cfg(feature = "triple_status")]
                StatusLedBase::<Triple>::set_disabled(cs, Some("System disabled by switch."));
            });
        } else if switch.is_low().unwrap() {
            critical_section::with(|cs| {
                debug!("critical_section: system disabled by switch");
                #[cfg(feature = "rgba_status")]
                StatusLedBase::<Rgba>::set_normal(cs, Some("System disabled by switch."));
                #[cfg(feature = "triple_status")]
                StatusLedBase::<Triple>::set_normal(cs, Some("System disabled by switch."));
            });
        }
    }
}
