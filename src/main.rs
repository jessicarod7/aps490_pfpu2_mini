//! Provides contact detection with a high-conductivity surface (such as brain tissue)
//! for an autopsy saw.
#![no_std]
#![no_main]
#![warn(missing_docs)]
#![cfg_attr(
    doc_cfg,
    feature(doc_cfg),
    feature(doc_auto_cfg),
    feature(doc_cfg_hide)
)]

use buffer::{create_avg_buffer, Buffers};
use cortex_m::peripheral::syst::SystClkSource;
use defmt::{debug, info, warn};
#[allow(unused_imports)]
use defmt_rtt as _;
use embedded_hal::pwm::SetDutyCycle;
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
    pwm::Slices,
    Adc, Sio, Watchdog,
};

use crate::{
    components::{StatusLed, StatusLedMulti},
    interrupt::{DISABLE_SWITCH, READINGS_FIFO, SIGNAL_GEN, STATUS_LEDS},
};

mod buffer;
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
pub static SIGNAL_GEN_FREQ_HZ: f32 = 100_000.0;

/// Main operation loop
#[entry]
fn main() -> ! {
    #[cfg(all(feature = "multi_status", feature = "rgba_status"))]
    panic!("Please select one LED status feature: `multi_status` or `rgba_status`");

    info!("Detection system startup");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // Rescale other calculations based on system clock
    let mut sysclk_rescale = 1f32;
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
            );
            sysclk_rescale = clocks.system_clock.freq().to_Hz() as f32 / SYS_CLOCK_FREQ as f32;
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
        #[cfg(feature = "multi_status")]
        STATUS_LEDS.replace(cs, StatusLedMulti::init(pins.gpio6, pins.gpio7, pins.gpio8))
    });

    // Initialize and start signal generator
    let mut pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
    pwm_slices
        .pwm3
        // Ex. 24 MHz clock generates 100 kHz signal ->  240 clk cycles per PWM cycle (`top`)
        // with 50% duty cycle
        .set_top(
            ((clocks.system_clock.freq().to_Hz() as f32 / (SIGNAL_GEN_FREQ_HZ * sysclk_rescale))
                - 1.0) as u16,
        );
    pwm_slices.pwm3.enable();
    let mut signal_gen = pwm_slices.pwm3.channel_a;
    signal_gen.output_to(pins.gpio22);
    signal_gen.set_duty_cycle_percent(50).unwrap();
    debug!("critical_section: transfer PWM control to mutex");
    critical_section::with(|cs| SIGNAL_GEN.replace(cs, Some(signal_gen)));

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
            ((clocks.system_clock.freq().to_Hz() as f32
                / (2.0 * (SIGNAL_GEN_FREQ_HZ * sysclk_rescale)))
                - 1.0) as u16,
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

    // Configure and enable SysTick for disable switch
    let disable_switch = pins.gpio9.into_pull_down_input();
    disable_switch.set_schmitt_enabled(true); // Debouncing
    debug!("critical_section: init disable switch");
    critical_section::with(|cs| DISABLE_SWITCH.replace(cs, Some(disable_switch)));

    let mut syst = core.SYST;
    syst.set_clock_source(SystClkSource::Core); // 1 us per tick
    syst.set_reload(20_000);
    syst.clear_current();
    syst.enable_interrupt();

    // Begin normal system operation
    critical_section::with(|cs| {
        StatusLedMulti::set_normal(cs, Some("System initialization complete"))
    });
    unsafe { pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0) }
    loop {
        // All functionality in interrupts
        cortex_m::asm::wfi();
    }
}
