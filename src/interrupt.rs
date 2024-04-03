//! Interrupt and exception handlers, plus static [`Mutex`].

use core::cell::RefCell;

use critical_section::Mutex;
use rp2040_hal::{
    adc::{AdcFifo, DmaReadTarget},
    dma::{single_buffer::Transfer, Channel, CH0},
};

use crate::components::StatusLedMulti;

/// Status LEDs for access in interrupts
pub static STATUS_LEDS: Mutex<RefCell<Option<StatusLedMulti>>> = Mutex::new(RefCell::new(None));

/// Wrappper for [`AdcFifo`] and [DMA `Transfer`](Transfer)
pub type ReadingsDma = Transfer<Channel<CH0>, DmaReadTarget<u8>, &'static mut [u8; 4000]>;
///  access in interrupts
pub static READINGS_FIFO: Mutex<RefCell<Option<ReadingsDma>>> = Mutex::new(RefCell::new(None));
