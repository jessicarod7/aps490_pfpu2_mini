//! Interrupt and exception handlers, plus static [`Mutex`].

use core::cell::RefCell;

use critical_section::Mutex;
use defmt::debug;
use rp2040_hal::{
    adc::DmaReadTarget,
    dma,
    dma::{CH0, Channel, single_buffer::Transfer},
    pac::interrupt,
};
use crate::buffer::Buffers;

use crate::components::{StatusLed, StatusLedMulti, StatusLedStates};

/// Status LEDs for access in interrupts
#[cfg(feature = "multi_status")]
pub static STATUS_LEDS: Mutex<RefCell<Option<&'static mut StatusLedMulti>>> =
    Mutex::new(RefCell::new(None));

/// Wrapper for [DMA `Transfer`](Transfer)
pub type ReadingsDma = Transfer<Channel<CH0>, DmaReadTarget<u8>, &'static mut [u8; 4000]>;
///  access in interrupts
pub static READINGS_FIFO: Mutex<RefCell<Option<ReadingsDma>>> = Mutex::new(RefCell::new(None));

/// Global buffers for analyzing readings
pub static BUFFERS: Mutex<RefCell<Option<&'static mut Buffers>>> = Mutex::new(RefCell::new(None));

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

        let avg1: i32 = avg_buffer.iter().step_by(2).map(|i| *i as i32).sum::<i32>() / 2000;
        let avg2: i32 = avg_buffer
            .iter()
            .skip(1)
            .step_by(2)
            .map(|i| *i as i32)
            .sum::<i32>()
            / 2000;
        let sample_avg = u8::try_from((avg1 - avg2).abs()).map_or(255, |avg| avg);

        // Determine if enough low sample events have occurred
        critical_section::with(|cs| {
            debug!("critical_section: dma update and check longterm buffers");
            let buffers = BUFFERS.take(cs).expect(Buffers::NO_BUFFER_PANIC_MSG);
            buffers.insert(sample_avg);

            debug!("critical_section: match status for correct buffer logic");
            let status_leds = STATUS_LEDS.borrow_ref(cs);
            if status_leds.is_some() {
                match status_leds.as_ref().unwrap().state {
                    StatusLedStates::Normal => buffers.detect_contact(),
                    StatusLedStates::Alert => buffers.detect_end_contact(),
                    StatusLedStates::Error | StatusLedStates::Disabled => {}
                }
            }
            
            BUFFERS.replace(cs, Some(buffers));
            debug!("exit buffer critical section");
        });

        let new_dma_transfer = dma::single_buffer::Config::new(dma_ch, dma_from, avg_buffer);
        debug!("critical_section: start new DMA transfer");
        critical_section::with(|cs| READINGS_FIFO.replace(cs, Some(new_dma_transfer.start())));
    } else {
        // Report error if FIFO is not active
        critical_section::with(|cs| {
            debug!("critical_section: dma set_error for no active FIFO");
            #[cfg(feature = "multi_status")]
            StatusLedMulti::set_error(
                cs,
                Some("No ADC transfer in progress! Unable to collect latest readings"),
            )
        });
    }
}
