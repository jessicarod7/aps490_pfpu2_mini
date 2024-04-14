//! This [RP2040](rp2040_hal) project provides contact detection with a
//! highly-conductivity/highly-capacitive surface (such as brain tissue) for an autopsy saw. For
//! more information, check out [the repo](https://github.com/cam-rod/aps490_pfpu2_mini).
//!
//! ## Crate features
//!
//! - `triple_status`: Enables the use of 3 LEDs to provide system status. This is the main user
//!   interface for the tool, and is enabled by default.
//! - `rgba_status`: Alternate configuration which uses a single common-anode RGB LED. This is the design which appears in
//!   [our schematic](https://github.com/cam-rod/aps490_retraction_fsm/blob/hardware/aps490_detection/aps490_detection-schematic.pdf).
//! - `trace_avg_samples`: Logs the average voltage difference measured, 250 samples at a time. See
//!   [`buffer::Buffers::trace_avg_samples`].
//! - `trace_indiv_samples` Logs information on every sample recorded. Very noisy! See
//!   [`interrupt::AlignedAverages::trace_high_index`] and [`interrupt::trace_indiv_samples`]
//! - `disable_switch`: Starts the SysTick timer to check the disable switch status. Never tested
//!   this feature, and I'm pretty sure my implementation will cause the system to panic due to poor
//!   synchronization. This functionality should be redesigned before enabling the feature.
//!
//! <div class="warning">Features <code>triple_status</code> and <code>rgba_status</code> are
//! mutually exclusive.</div>
//!
//! ## Demo
//!
//! The following is a simplified (including [`Rgba`](components::Rgba)-only lights) implementation of the [binary crate](https://github.com/cam-rod/aps490_pfpu2_mini/blob/main/src/main.rs)
//! used on our proof-of-concept.
//!
//! ```no_run
//! #![no_std]
//! #![no_main]
//!
//! use aps490_pfpu2_mini::{
//!     buffer::{create_avg_buffer, Buffers},
//!     components::{LedControl, Rgba, StatusLed, StatusLedBase},
//!     interrupt::{DISABLE_SWITCH, READINGS_FIFO, SIGNAL_GEN, STATUS_LEDS},
//! };
//! use cortex_m::peripheral::syst::SystClkSource;
//! use defmt::{debug, warn};
//! #[allow(unused_imports)]
//! use defmt_rtt as _;
//! use embedded_hal::pwm::SetDutyCycle;
//! #[allow(unused_imports)]
//! use panic_probe as _;
//! use rp2040_hal::{
//!     adc::{Adc, AdcPin},
//!     clocks::init_clocks_and_plls,
//!     dma::{single_buffer, DMAExt, SingleChannel},
//!     entry,
//!     gpio::Pins,
//!     pac,
//!     prelude::*,
//!     pwm::Slices,
//!     Sio, Watchdog,
//! };
//!
//! #[link_section = ".boot2"]
//! #[used]
//! pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;
//! pub const XOSC_FREQ_HZ: u32 = 12_000_000;
//! const SYS_CLOCK_FREQ: u32 = 24_000_000;
//! pub static SIGNAL_GEN_FREQ_HZ: f32 = 100_000.0;
//!
//! #[entry]
//! fn main() -> ! {
//!     let mut pac = pac::Peripherals::take().unwrap();
//!     let core = pac::CorePeripherals::take().unwrap();
//!     let mut watchdog = Watchdog::new(pac.WATCHDOG);
//!     let sio = Sio::new(pac.SIO);
//!
//!     let mut clocks = init_clocks_and_plls(
//!         XOSC_FREQ_HZ,
//!         pac.XOSC,
//!         pac.CLOCKS,
//!         pac.PLL_SYS,
//!         pac.PLL_USB,
//!         &mut pac.RESETS,
//!         &mut watchdog,
//!     )
//!     .ok()
//!     .unwrap();
//!     let mut sysclk_rescale = clocks.system_clock.freq().to_Hz() as f32 / SYS_CLOCK_FREQ as f32;
//!
//!     // Setup status LEDs
//!     let pins = Pins::new(
//!         pac.IO_BANK0,
//!         pac.PADS_BANK0,
//!         sio.gpio_bank0,
//!         &mut pac.RESETS,
//!     );
//!     critical_section::with(|cs| {
//!         STATUS_LEDS.replace(cs, Rgba::init(pins.gpio6, pins.gpio7, pins.gpio8));
//!     });
//!
//!     // Initialize and start signal generator
//!     let mut pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
//!     pwm_slices.pwm3.set_top(
//!         ((clocks.system_clock.freq().to_Hz() as f32 / (SIGNAL_GEN_FREQ_HZ * sysclk_rescale))
//!             - 1.0) as u16,
//!     );
//!     pwm_slices.pwm3.enable();
//!     let mut signal_gen = pwm_slices.pwm3.channel_a;
//!     signal_gen.output_to(pins.gpio22);
//!     signal_gen.set_duty_cycle_percent(50).unwrap();
//!     critical_section::with(|cs| SIGNAL_GEN.replace(cs, Some(signal_gen)));
//!
//!     // Setup ADC pins, DMA, buffers
//!     let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
//!     let mut adc_pin0 = AdcPin::new(pins.gpio26.into_floating_input()).unwrap();
//!     let mut dma = pac.DMA.split(&mut pac.RESETS);
//!     Buffers::init();
//!
//!     // Setup first transfer
//!     let avg_buffer = create_avg_buffer().unwrap();
//!     let mut readings_fifo = adc
//!         .build_fifo()
//!         .set_channel(&mut adc_pin0)
//!         .clock_divider(
//!             ((clocks.system_clock.freq().to_Hz() as f32
//!                 / (2.0 * (SIGNAL_GEN_FREQ_HZ * sysclk_rescale)))
//!                 - 1.0) as u16,
//!             0,
//!         )
//!         .shift_8bit()
//!         .enable_dma()
//!         .start_paused();
//!     dma.ch0.enable_irq0();
//!     let adc_dma_transfer =
//!         single_buffer::Config::new(dma.ch0, readings_fifo.dma_read_target(), avg_buffer);
//!     debug!("critical_section: transfer readings FIFO to mutex");
//!     critical_section::with(|cs| READINGS_FIFO.replace(cs, Some(adc_dma_transfer.start())));
//!     readings_fifo.resume();
//!
//!     // Configure and enable SysTick for disable switch
//!     let disable_switch = pins.gpio9.into_pull_down_input();
//!     disable_switch.set_schmitt_enabled(true); // Debouncing
//!     debug!("critical_section: init disable switch");
//!     critical_section::with(|cs| DISABLE_SWITCH.replace(cs, Some(disable_switch)));
//!
//!     let mut syst = core.SYST;
//!     syst.set_clock_source(SystClkSource::Core); // 1 us per tick
//!     syst.set_reload(20_000);
//!     syst.clear_current();
//!     #[cfg(feature = "disable_switch")]
//!     syst.enable_interrupt();
//!
//!     // Begin normal system operation
//!     critical_section::with(|cs| {
//!         #[cfg(feature = "rgba_status")]
//!         StatusLedBase::<Rgba>::set_normal(cs, Some("System initialization complete"));
//!         #[cfg(feature = "triple_status")]
//!         StatusLedBase::<Triple>::set_normal(cs, Some("System initialization complete"));
//!     });
//!     unsafe { pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0) }
//!     loop {
//!         // All functionality in interrupts
//!         cortex_m::asm::wfi();
//!     }
//! }
//! ```

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

#![no_std]
#![no_main]
#![warn(missing_docs)]
#![cfg_attr(docsrs, feature(doc_cfg), feature(doc_auto_cfg), feature(doc_cfg_hide))]

pub mod buffer;
pub mod components;
pub mod interrupt;

#[cfg(all(feature = "triple_status", feature = "rgba_status"))]
compile_error!("Features `triple_status` and `rgba_status` cannot be enabled at the same time in crate aps490_pfpu2_mini");
