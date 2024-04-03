//! Provides contact detection with a high-conductivity surface (such as brain tissue)
//! for an autopsy saw.
#![no_std]
#![no_main]
#![doc(html_playground_url = "https://play.rust-lang.org/")]

use defmt::{debug, info, warn};
#[allow(unused_imports)]
use defmt_rtt as _;
#[allow(unused_imports)]
use panic_probe as _;
use rp2040_hal::{
    clocks::init_clocks_and_plls,
    dma,
    dma::{DMAExt, SingleChannel},
    entry,
    fugit::RateExtU32,
    gpio::Pins,
    pac,
    prelude::*,
    Adc, Sio, Watchdog,
};

use crate::{
    components::{create_avg_buffer, Buffers, StatusLedMulti},
    interrupt::{READINGS_FIFO, STATUS_LEDS},
};

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
    let _core = pac::CorePeripherals::take().unwrap();
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
                "Unable to downscale clock speed: {}\nClocks will continue to run at {=u32}",
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

    // Setup status LEDs
    debug!("critical_section: init status LEDs");
    critical_section::with(|cs| {
        STATUS_LEDS.replace(cs, StatusLedMulti::init(pins.gpio6, pins.gpio7, pins.gpio8))
    });

    // Setup ADC pins, DMA, buffers
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pin0 = rp2040_hal::adc::AdcPin::new(pins.gpio26.into_floating_input()).unwrap();
    let mut dma = pac.DMA.split(&mut pac.RESETS);
    Buffers::init();

    // Setup first transfer
    let avg_buffer = create_avg_buffer().unwrap();
    let mut readings_fifo = adc
        .build_fifo()
        .set_channel(&mut adc_pin0)
        // Ex. 24 MHz clock at 200 ksamples/s (2x SIGNAL_FREQ_KHZ) -> sample every 120 clk cycles
        .clock_divider(
            ((clocks.system_clock.freq().to_Hz() / (2 * SIGNAL_GEN_FREQ_HZ)) - 1) as u16,
            0,
        )
        .shift_8bit()
        .enable_dma()
        .start_paused();
    dma.ch0.enable_irq0();
    let adc_dma_transfer =
        dma::single_buffer::Config::new(dma.ch0, readings_fifo.dma_read_target(), avg_buffer);
    debug!("critical_section: transfer readings FIFO to mutex");
    critical_section::with(|cs| READINGS_FIFO.replace(cs, Some(adc_dma_transfer.start())));
    readings_fifo.resume();

    loop {}
}
