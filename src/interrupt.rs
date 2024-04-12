//! Interrupt and exception handlers, plus static [`Mutex`].

use core::{cell::RefCell, cmp::Ordering};

use cortex_m_rt::exception;
use critical_section::Mutex;
use defmt::debug;
#[allow(unused_imports)]
use defmt::trace;
use embedded_hal::digital::InputPin;
use rp2040_hal::{
    adc::DmaReadTarget,
    dma,
    dma::{single_buffer::Transfer, Channel, CH0},
    gpio::{bank0::Gpio9, FunctionSio, Pin, PullDown, SioInput},
    pac::interrupt,
    pwm,
    pwm::{FreeRunning, Pwm3, Slice},
};

use crate::{
    buffer::{Buffers, DetectionMsg},
    components::{StatusLed, StatusLedMulti, StatusLedStates},
};

/// Status LEDs for access in interrupts
#[cfg(any(doc, feature = "led_status"))]
pub static STATUS_LEDS: Mutex<RefCell<Option<&'static mut StatusLedMulti>>> =
    Mutex::new(RefCell::new(None));

/// Wrapper for [DMA `Transfer`](Transfer)
pub type ReadingsDma = Transfer<Channel<CH0>, DmaReadTarget<u8>, &'static mut [u8; 4000]>;
pub type DisableSwitch = Pin<Gpio9, FunctionSio<SioInput>, PullDown>;
pub type SignalPwm = pwm::Channel<Slice<Pwm3, FreeRunning>, pwm::A>;
pub type SignalGenConfig = (Channel<CH0>, DmaReadTarget<u8>, &'static mut [u8; 4000]);

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
struct AlignedAverages {
    avg_high: i32,
    avg_low: i32,
}

impl AlignedAverages {
    /// Takes the partial sums of the samples to calculate the high and low averages for contact
    /// detection.
    ///
    /// Sorting requires an allocator, so this implementation identifies the highest partial sums.
    /// Although this implementation technically allows for non-adjacent partial sums to be matched,
    /// in effect this has little impact as those scenarios result in low overall deltas.
    fn align_signal_timing(partial_sums: &[i32; 4]) -> Self {
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
        {
            trace!("high indices (mod 4): {}", avg_high_idx);
        }

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
        {
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
                /*unique_samples*/ 0
            );
        }
        let sample_avg = avgs.get_delta();

        // Determine if enough low sample events have occurred
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
                #[cfg(feature = "led_status")]
                let buffers = BUFFERS.take(cs).unwrap();
                StatusLedMulti::set_alert(cs, Some(DetectionMsg::create(buffers)));
                BUFFERS.replace(cs, Some(buffers));
            });
        } else if reset_detected {
            critical_section::with(|cs| {
                #[cfg(feature = "led_status")]
                StatusLedMulti::set_normal(cs, None)
            })
        }

        let new_dma_transfer = dma::single_buffer::Config::new(dma_ch, dma_from, avg_buffer);
        debug!("critical_section: start new DMA transfer");
        critical_section::with(|cs| READINGS_FIFO.replace(cs, Some(new_dma_transfer.start())));
    } else {
        // Report error if FIFO is not active
        critical_section::with(|cs| {
            debug!("critical_section: dma set_error for no active FIFO");
            #[cfg(feature = "led_status")]
            StatusLedMulti::set_error(
                cs,
                Some("No ADC transfer in progress! Unable to collect latest readings"),
            )
        });
    }
}

/// ISR for SysTick, used for checking [`DisableSwitch`]
///
/// Lazily takes ownership of [`DISABLE_SWITCH`] as it will not be used again in the main runtime again.
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
                #[cfg(feature = "led_status")]
                StatusLedMulti::set_disabled(cs, Some("System disabled by switch."))
            });
        } else if switch.is_low().unwrap() {
            critical_section::with(|cs| {
                debug!("critical_section: system disabled by switch");
                #[cfg(feature = "led_status")]
                StatusLedMulti::set_normal(cs, Some("System disabled by switch."))
            });
        }
    }
}
