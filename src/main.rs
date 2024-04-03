//! Provides contact detection with a high-conductivity surface (such as brain tissue)
//! for an autopsy saw.
#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::Mutex;
use defmt::{info, warn};
#[allow(unused_imports)]
use defmt_rtt as _;
#[allow(unused_imports)]
use panic_probe as _;
use rp2040_hal::{entry, pac, prelude::*, Sio, Watchdog};
use rp2040_hal::clocks::init_clocks_and_plls;
use rp2040_hal::fugit::RateExtU32;
use rp2040_hal::gpio::Pins;
use crate::components::StatusLed;

mod components;
mod interrupt;

/// Second-stage bootloader, from [rp2040-boot2](https://docs.rs/rp2040-boot2)
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;
/// External high-speed crystal on the pico board is 12Mhz
pub const XOSC_FREQ_HZ: u32 = 12_000_000;
/// Attempt to run system clock at 24 MHz
pub const SYS_CLOCK_FREQ: u32 = 24_000_000;
/// Frequency of detection signal is 100 kHz
pub static SIGNAL_GEN_FREQ_HZ: u32 = 100_000;

/// Main operation loop
#[entry]
fn main() -> ! {
    info!("Detection system startup");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let mut clocks = init_clocks_and_plls(
        XOSC_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();
    // Attempt to switch system to 24 MHz for efficiency
    clocks
        .system_clock
        .configure_clock(&clocks.reference_clock, SYS_CLOCK_FREQ.Hz())
        .unwrap_or_else(|err| {
            warn!(
                "Unable to downscale clock speed: {}\nClocks will continue to run at {}",
                err,
                clocks.system_clock.freq().to_Hz()
            )
        });
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    
    /// Setup LED pins
    let status_leds = StatusLed::init(pins.gpio6, pins.gpio7, pins.gpio8);
    
    loop {}
}
