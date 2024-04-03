//! Interrupt and exception handlers, plus static [`Mutex`].

use core::cell::RefCell;

use critical_section::Mutex;
use defmt::debug;
use rp2040_hal::{
    adc::DmaReadTarget,
    dma::{single_buffer::Transfer, Channel, CH0},
    pac::interrupt,
};

use crate::components::{Buffers, StatusLedMulti};

/// Status LEDs for access in interrupts
pub static STATUS_LEDS: Mutex<RefCell<Option<&'static mut StatusLedMulti>>> =
    Mutex::new(RefCell::new(None));

/// Wrappper for [`AdcFifo`] and [DMA `Transfer`](Transfer)
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
        
        // Detetmine if enough low sample events have occurred
        critical_section::with(|cs| {
            debug!("critical_section: dma update and check longterm buffers");
            let buffers = BUFFERS.take(cs).expect(Buffers::NO_BUFFER_PANIC_MSG);
            buffers.insert(sample_avg);
            
            BUFFERS.replace(cs, Some(buffers));
            debug!("exit buffer critical section");
        })
    } else {
        // Report error if FIFO is not active
        critical_section::with(|cs| {
            debug!("critical_section: dma set_error for no active FIFO");
            StatusLedMulti::set_error(
                cs,
                Some("No ADC transfer in progress! Unable to collect latest readings"),
            )
        });
    }
}
