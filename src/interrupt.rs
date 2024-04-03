//! Interrupt and exception handlers, plus static [`Mutex`].

use core::cell::RefCell;
use critical_section::Mutex;
use crate::components::StatusLed;

/// Status LEDs for access in interrupts
pub static STATUS_LEDS: Mutex<RefCell<Option<StatusLed>>> = Mutex::new(RefCell::new(None));

