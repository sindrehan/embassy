use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::{env, fs};

use cfg_aliases::cfg_aliases;
use nxp_pac::metadata;
use nxp_pac::metadata::{METADATA, Peripheral};
#[allow(unused)]
use proc_macro2::Literal;
#[allow(unused)]
use proc_macro2::TokenStream;
use proc_macro2::{Ident, Span};
use quote::format_ident;
#[allow(unused)]
use quote::quote;

#[path = "./build_common.rs"]
mod common;

fn main() {
    let mut cfgs = common::CfgSet::new();
    common::set_target_cfgs(&mut cfgs);

    let chip_name = match env::vars()
        .map(|(a, _)| a)
        .filter(|x| {
            x.starts_with("CARGO_FEATURE_MIMXRT")
                || x.starts_with("CARGO_FEATURE_LPC")
                || x.starts_with("CARGO_FEATURE_MKL")
        })
        .get_one()
    {
        Ok(x) => x,
        Err(GetOneError::None) => panic!("No mimxrt/lpc/mkl Cargo feature enabled"),
        Err(GetOneError::Multiple) => panic!("Multiple mimxrt/lpc/mkl Cargo features enabled"),
    }
    .strip_prefix("CARGO_FEATURE_")
    .unwrap()
    .to_ascii_lowercase();

    let singletons = singletons(&mut cfgs);

    cfg_aliases! {
        rt1xxx: { any(feature = "mimxrt1011", feature = "mimxrt1062") },
    }

    cfg_aliases! {
        lpc55: { any(feature = "lpc55s16", feature = "lpc55-core0") },
    }

    cfg_aliases! {
        kinetis: { feature = "mkl82z7" },
    }

    eprintln!("chip: {chip_name}");

    generate_code(&mut cfgs, &singletons);
}

/// The instance suffix of a DMA controller: `"0"` for `DMA0`, `""` for the single unnumbered
/// Kinetis `DMA`. `None` for anything else that merely starts with `DMA`, such as `DMAMUX`.
fn dma_instance(name: &str) -> Option<&str> {
    let instance = name.strip_prefix("DMA")?;
    (instance.is_empty() || instance.parse::<u8>().is_ok()).then_some(instance)
}

/// A peripheral singleton returned by `embassy_nxp::init`.
struct Singleton {
    name: String,

    /// A cfg guard which indicates whether the `Peripherals` struct will give the user this singleton.
    cfg: Option<TokenStream>,
}

fn singletons(cfgs: &mut common::CfgSet) -> Vec<Singleton> {
    let mut singletons = Vec::new();

    for peripheral in METADATA.peripherals {
        // GPIO and DMA are generated in a 2nd pass.
        let skip_singleton = peripheral.name.starts_with("GPIO") || dma_instance(peripheral.name).is_some();

        // Kinetis time drivers own their timer peripherals.
        let skip_singleton = skip_singleton || (cfg!(feature = "time-driver-tpm") && peripheral.name == "TPM2");

        if !skip_singleton {
            singletons.push(Singleton {
                name: peripheral.name.into(),
                cfg: (cfg!(feature = "time-driver-lptmr") && matches!(peripheral.name, "LPTMR0" | "LPTMR1"))
                    .then(|| quote! { #[cfg(not(feature = "time-driver-lptmr"))] }),
            });
        }
    }

    cfgs.declare_all(&[
        "gpio1",
        "gpio1_hi",
        "gpio2",
        "gpio2_hi",
        "gpio3",
        "gpio3_hi",
        "gpio4",
        "gpio4_hi",
        "gpio5",
        "gpio5_hi",
        "gpio10",
        "gpio10_hi",
    ]);

    for peripheral in METADATA.peripherals.iter().filter(|p| p.name.starts_with("GPIO")) {
        let instance = peripheral.name.strip_prefix("GPIO").unwrap();
        // RT1xxx and LPC55 number their GPIO banks, Kinetis letters them (GPIOA..GPIOE).
        let numbered = instance.parse::<u8>().is_ok();
        assert!(numbered || (!instance.is_empty() && instance.chars().all(|c| c.is_ascii_uppercase())));
        if numbered {
            cfgs.enable(format!("gpio{}", instance));
        }

        for signal in peripheral.signals.iter() {
            let pin_number = signal.name.parse::<u8>().unwrap();

            if numbered && pin_number > 15 {
                cfgs.enable(format!("gpio{}_hi", instance));
            }

            // GPIO signals only defined a single signal, on a single pin.
            assert_eq!(signal.pins.len(), 1);

            singletons.push(Singleton {
                name: signal.pins[0].pin.into(),
                cfg: None,
            });
        }
    }

    // Kinetis parts also expose dedicated analog pins which are not part of a GPIO bank.
    #[cfg(feature = "_kinetis")]
    for pin in METADATA.pins {
        if !singletons.iter().any(|singleton| singleton.name == pin.name) {
            singletons.push(Singleton {
                name: pin.name.into(),
                cfg: None,
            });
        }
    }

    for (peripheral, instance) in METADATA
        .peripherals
        .iter()
        .filter_map(|p| dma_instance(p.name).map(|i| (p, i)))
    {
        for signal in peripheral.signals.iter() {
            let channel_number = signal.name.parse::<u8>().unwrap();
            let name = format!("DMA{instance}_CH{channel_number}");

            // DMA has no pins.
            assert!(signal.pins.is_empty());

            singletons.push(Singleton { name, cfg: None });
        }
    }

    for peripheral in METADATA.peripherals.iter().filter(|p| p.name.starts_with("SCT")) {
        let instance = peripheral.name.strip_prefix("SCT").unwrap();
        assert!(instance.parse::<u8>().is_ok());

        for signal in peripheral.signals.iter() {
            if !signal.name.starts_with("OUT") {
                continue;
            }

            let channel_number = signal.name.strip_prefix("OUT").unwrap().parse::<u8>().unwrap();
            let name = format!("SCT{instance}_OUT{channel_number}");

            singletons.push(Singleton { name, cfg: None });
        }
    }

    singletons
}

#[cfg(feature = "_rt1xxx")]
fn generate_iomuxc() -> TokenStream {
    let iomuxc_pad_impls = metadata::METADATA
        .pins
        .iter()
        .filter(|p| p.iomuxc.as_ref().filter(|i| i.mux.is_some()).is_some())
        .map(|pin| {
            let Some(ref iomuxc) = pin.iomuxc else {
                panic!("Pin {} has no IOMUXC definitions", pin.name);
            };

            let name = Ident::new(pin.name, Span::call_site());
            let mux = iomuxc.mux.unwrap();
            let pad = iomuxc.pad;

            quote! {
                impl_iomuxc_pad!(#name, #pad, #mux);
            }
        });

    let base_match_arms = metadata::METADATA
        .peripherals
        .iter()
        .filter(|p| p.name.starts_with("GPIO"))
        .map(|peripheral| {
            peripheral.signals.iter().map(|signal| {
                // All GPIO signals have a single pin.
                let pin = &signal.pins[0];
                let instance = peripheral.name.strip_prefix("GPIO").unwrap();
                let bank_match = format_ident!("Gpio{}", instance);
                let pin_number = signal.name.parse::<u8>().unwrap();
                let pin_ident = Ident::new(pin.pin, Span::call_site());

                quote! {
                    (Bank::#bank_match, #pin_number) => <crate::peripherals::#pin_ident as crate::iomuxc::SealedPad>
                }
            })
        })
        .flatten()
        .collect::<Vec<_>>();

    let pad_match_arms = base_match_arms.iter().map(|arm| {
        quote! { #arm::PAD }
    });

    let mux_match_arms = base_match_arms.iter().map(|arm| {
        quote! { #arm::MUX }
    });

    quote! {
        #(#iomuxc_pad_impls)*

        pub(crate) fn iomuxc_pad(bank: crate::gpio::Bank, pin: u8) -> *mut () {
            use crate::gpio::Bank;

            match (bank, pin) {
                #(#pad_match_arms),*,
                _ => unreachable!()
            }
        }

        pub(crate) fn iomuxc_mux(bank: crate::gpio::Bank, pin: u8) -> Option<*mut ()> {
            use crate::gpio::Bank;

            match (bank, pin) {
                #(#mux_match_arms),*,
                _ => unreachable!()
            }
        }
    }
}

fn generate_code(cfgs: &mut common::CfgSet, singletons: &[Singleton]) {
    #[allow(unused)]
    use std::fmt::Write;

    let out_dir = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    #[allow(unused_mut)]
    let mut output = String::new();

    writeln!(&mut output, "{}", peripherals(singletons)).unwrap();

    #[cfg(feature = "_rt1xxx")]
    writeln!(&mut output, "{}", generate_iomuxc()).unwrap();

    writeln!(&mut output, "{}", interrupts()).unwrap();
    writeln!(&mut output, "{}", impl_peripherals(cfgs, singletons)).unwrap();

    let out_file = out_dir.join("_generated.rs").to_string_lossy().to_string();
    fs::write(&out_file, output).unwrap();
    rustfmt(&out_file);
}

fn interrupts() -> TokenStream {
    let interrupts = METADATA.interrupts.iter().map(|(name, _)| format_ident!("{name}"));

    quote! {
        embassy_hal_internal::interrupt_mod!(#(#interrupts),*);
    }
}

fn peripherals(singletons: &[Singleton]) -> TokenStream {
    let defs = singletons.iter().map(|s| {
        let ident = Ident::new(&s.name, Span::call_site());
        quote! { #ident }
    });

    let peripherals = singletons.iter().map(|s| {
        let ident = Ident::new(&s.name, Span::call_site());
        let cfg = s.cfg.clone().unwrap_or_else(|| quote! {});
        quote! {
            #cfg
            #ident
        }
    });

    quote! {
        embassy_hal_internal::peripherals_definition!(#(#defs),*);
        embassy_hal_internal::peripherals_struct!(#(#peripherals),*);
    }
}

#[cfg(not(feature = "_kinetis"))]
fn impl_adc(impls: &mut Vec<TokenStream>, peripheral: &Peripheral) {
    for signal in peripheral.signals.iter() {
        let (ch_num, ch_side) = signal.name.rsplit_once("_").unwrap();
        let ch_num = ch_num.strip_prefix("CH").unwrap().parse::<u8>().unwrap();
        let ch_side = match ch_side {
            "A" => format_ident!("SideA"),
            "B" => format_ident!("SideB"),
            side => panic!("Invalid ADC channel side: {}", side),
        };

        assert_eq!(signal.pins.len(), 1);
        let pin = format_ident!("{}", signal.pins[0].pin);
        let ch_num = Literal::u8_unsuffixed(ch_num);

        impls.push(quote! {
            impl_adc_pin!(#pin, #ch_num, crate::adc::ChannelSide::#ch_side);
        })
    }
}

#[cfg(feature = "_kinetis")]
fn impl_adc16(impls: &mut Vec<TokenStream>, peripheral: &Peripheral) {
    let instance = Ident::new(peripheral.name, Span::call_site());
    impls.push(quote! {
        impl_adc_instance!(#instance);
    });

    if let Some(args) = interrupt_args(peripheral) {
        impls.push(quote! {
            impl_adc_interrupt!(#args);
        });
    }

    for signal in peripheral.signals {
        let (channel, mux_b) = match signal.name {
            "DP0" => (0, false),
            "DP1" => (1, false),
            "DM0" => (17, false),
            "DM1" => (18, false),
            name if name.starts_with("SE") => {
                let suffix = name.strip_prefix("SE").unwrap();
                let mux_b = suffix.ends_with('B');
                let number = suffix.trim_end_matches('B').parse::<u8>().unwrap();
                (number, mux_b)
            }
            _ => continue,
        };
        let channel = Literal::u8_unsuffixed(channel);

        for pin in signal.pins {
            let pin = format_ident!("{}", pin.pin);
            if pin.to_string().starts_with("PT") {
                impls.push(quote! {
                    impl_adc_gpio_pin!(#pin, #instance, #channel, #mux_b);
                });
            } else {
                impls.push(quote! {
                    impl_adc_fixed_pin!(#pin, #instance, #channel, #mux_b);
                });
            }
        }
    }
}

fn impl_gpio_pin(impls: &mut Vec<TokenStream>, peripheral: &Peripheral) {
    let instance = peripheral.name.strip_prefix("GPIO").unwrap();
    let bank = format_ident!("Gpio{}", instance);
    // let pin =

    for signal in peripheral.signals.iter() {
        let pin_number = signal.name.parse::<u8>().unwrap();
        let pin = Ident::new(signal.pins[0].pin, Span::call_site());

        impls.push(quote! {
            impl_pin!(#pin, #bank, #pin_number);
        });
    }
}

fn impl_dma_channel(impls: &mut Vec<TokenStream>, peripheral: &Peripheral) {
    let instance = Ident::new(peripheral.name, Span::call_site());

    for signal in peripheral.signals.iter() {
        let channel_number = signal.name.parse::<u8>().unwrap();
        let channel_name = format_ident!("{instance}_CH{channel_number}");

        impls.push(quote! {
            impl_dma_channel!(#instance, #channel_name, #channel_number);
        });
    }
}

#[cfg(not(feature = "_kinetis"))]
fn impl_usart(cfgs: &mut common::CfgSet, impls: &mut Vec<TokenStream>, peripheral: &Peripheral) {
    cfgs.declare_all(&[
        "has_usart_txd_pins",
        "has_usart_rxd_pins",
        "has_usart_cts_pins",
        "has_usart_rts_pins",
        "has_usart_sck_pins",
    ]);

    let instance = Ident::new(peripheral.name, Span::call_site());
    let flexcomm = Ident::new(
        peripheral.flexcomm.expect("LPC55 must specify FLEXCOMM instance"),
        Span::call_site(),
    );
    let number = Literal::u8_unsuffixed(peripheral.name.strip_prefix("USART").unwrap().parse::<u8>().unwrap());

    impls.push(quote! {
        impl_usart_instance!(#instance, #flexcomm, #number);
    });

    for signal in peripheral.signals {
        let r#macro = match signal.name {
            "TXD" => {
                cfgs.enable("has_usart_txd_pins");
                format_ident!("impl_usart_txd_pin")
            }
            "RXD" => {
                cfgs.enable("has_usart_rxd_pins");
                format_ident!("impl_usart_rxd_pin")
            }
            "CTS" => {
                cfgs.enable("has_usart_cts_pins");
                format_ident!("impl_usart_cts_pin")
            }
            "RTS" => {
                cfgs.enable("has_usart_rts_pins");
                format_ident!("impl_usart_rts_pin")
            }
            "SCK" => {
                cfgs.enable("has_usart_sck_pins");
                format_ident!("impl_usart_sck_pin")
            }
            _ => unreachable!(),
        };

        for pin in signal.pins {
            let alt = format_ident!("Alt{}", pin.alt);
            let pin = format_ident!("{}", pin.pin);

            impls.push(quote! {
                #r#macro!(#pin, #instance, #alt);
            });
        }
    }

    for dma_mux in peripheral.dma_muxing {
        assert_eq!(dma_mux.mux, "DMA0", "TODO: USART for more than LPC55");

        let r#macro = match dma_mux.signal {
            "TX" => format_ident!("impl_usart_tx_channel"),
            "RX" => format_ident!("impl_usart_rx_channel"),
            _ => unreachable!(),
        };

        let channel = format_ident!("DMA0_CH{}", dma_mux.request);

        impls.push(quote! {
            #r#macro!(#instance, #channel);
        });
    }
}

#[cfg(not(feature = "_kinetis"))]
fn impl_sct(impls: &mut Vec<TokenStream>, peripheral: &Peripheral) {
    let instance = Ident::new(peripheral.name, Span::call_site());

    impls.push(quote! {
        impl_sct_instance!(#instance);
    });

    for signal in peripheral.signals.iter() {
        if signal.name.starts_with("OUT") {
            let channel_number = signal.name.strip_prefix("OUT").unwrap().parse::<u8>().unwrap();

            let channel_name = format_ident!("{instance}_OUT{channel_number}");

            impls.push(quote! {
                impl_sct_output_instance!(#instance, #channel_name, #channel_number);
            });

            if signal.name.starts_with("OUT") {
                for pin in signal.pins {
                    let pin_name = format_ident!("{}", pin.pin);
                    let alt = format_ident!("Alt{}", pin.alt);

                    impls.push(quote! {
                        impl_sct_output_pin!(#instance, #channel_name, #pin_name, #alt);
                    });
                }
            }
        }
    }
}

#[cfg(feature = "_kinetis")]
fn impl_pwm(impls: &mut Vec<TokenStream>, peripheral: &Peripheral) {
    let instance = Ident::new(peripheral.name, Span::call_site());
    let channels = peripheral
        .signals
        .iter()
        .filter_map(|signal| signal.name.strip_prefix("CH")?.parse::<usize>().ok())
        .max()
        .map_or(0, |channel| channel + 1);

    impls.push(quote! {
        impl_pwm_instance!(#instance, #channels);
    });

    for signal in peripheral.signals.iter().filter(|signal| signal.name.starts_with("CH")) {
        let channel = Literal::usize_unsuffixed(signal.name.strip_prefix("CH").unwrap().parse().unwrap());
        for pin in signal.pins {
            let alt = format_ident!("Mux{}", pin.alt);
            let pin = format_ident!("{}", pin.pin);
            impls.push(quote! {
                impl_pwm_pin!(#pin, #instance, #channel, #alt);
            });
        }
    }
}

#[cfg(feature = "_kinetis")]
fn impl_flexio_pwm(impls: &mut Vec<TokenStream>, peripheral: &Peripheral) {
    let instance = Ident::new(peripheral.name, Span::call_site());
    impls.push(quote! {
        impl_flexio_pwm_instance!(#instance, 8);
    });

    for signal in peripheral.signals.iter().filter(|signal| signal.name.starts_with('D')) {
        let number = Literal::u8_unsuffixed(signal.name.strip_prefix('D').unwrap().parse().unwrap());
        for pin in signal.pins {
            let alt = format_ident!("Mux{}", pin.alt);
            let pin = format_ident!("{}", pin.pin);
            impls.push(quote! {
                impl_flexio_pwm_pin!(#pin, #instance, #number, #alt);
            });
        }
    }
}

#[cfg(not(feature = "_kinetis"))]
fn impl_spi(cfgs: &mut common::CfgSet, impls: &mut Vec<TokenStream>, peripheral: &Peripheral) {
    cfgs.declare_all(&["has_spi_sck_pins", "has_spi_mosi_pins", "has_spi_miso_pins"]);

    let instance = Ident::new(peripheral.name, Span::call_site());
    let flexcomm = Ident::new(
        peripheral.flexcomm.expect("LPC55 must specify FLEXCOMM instance"),
        Span::call_site(),
    );
    let number = Literal::u8_unsuffixed(peripheral.name.strip_prefix("SPI").unwrap().parse::<u8>().unwrap());

    impls.push(quote! {
        impl_spi_instance!(#instance, #flexcomm, #number);
    });

    for signal in peripheral.signals {
        let r#macro = match signal.name {
            "SCK" => {
                cfgs.enable("has_spi_sck_pins");
                format_ident!("impl_spi_sck_pin")
            }
            "MOSI" => {
                cfgs.enable("has_spi_mosi_pins");
                format_ident!("impl_spi_mosi_pin")
            }
            "MISO" => {
                cfgs.enable("has_spi_miso_pins");
                format_ident!("impl_spi_miso_pin")
            }
            _ => unreachable!(),
        };

        for pin in signal.pins {
            let alt = format_ident!("Alt{}", pin.alt);
            let pin = format_ident!("{}", pin.pin);

            impls.push(quote! {
                #r#macro!(#pin, #instance, #alt);
            });
        }
    }
}

/// Kinetis clock gating: one `SIM_SCGCx` bit per peripheral.
#[cfg(feature = "_kinetis")]
fn impl_clock_gate(impls: &mut Vec<TokenStream>, peripheral: &Peripheral) {
    let Some(gate) = peripheral.gate.as_ref() else {
        return;
    };

    let instance = Ident::new(peripheral.name, Span::call_site());
    let reg = format_ident!("{}", gate.enable);
    let getter = format_ident!("{}", gate.bit);
    let setter = format_ident!("set_{}", gate.bit);

    impls.push(quote! {
        impl_clock_gate!(#instance, #reg, #getter, #setter);
    });
}

/// INTMUX0 inputs of the peripherals without an NVIC line of their own (MKL82Z7 reference
/// manual table 3-3; the input number is the SDK's IRQn minus 32). Routed through channel 0.
#[cfg(feature = "_kinetis")]
const INTMUX_SOURCES: &[(&str, u8)] = &[
    ("LPTMR1", 0),
    ("SPI1", 4),
    ("LPUART2", 5),
    ("EMVSIM1", 6),
    ("I2C1", 7),
    ("TSI0", 8),
    ("PMC", 9),
    ("FTFA", 10),
    ("MCG", 11),
    ("WDOG", 12),
    ("DAC0", 13),
    ("TRNG0", 14),
    ("CMP0", 16),
];

/// DSPI FIFO depths (MKL82Z7 SDK `FSL_FEATURE_DSPI_FIFO_SIZEn`).
#[cfg(feature = "_kinetis")]
const DSPI_FIFO_DEPTH: &[(&str, u8)] = &[("SPI0", 4), ("SPI1", 1)];

/// The tokens for a driver's `impl_*_interrupt!` invocation: the peripheral's own NVIC line, or
/// the INTMUX channel line plus the source number, or nothing.
#[cfg(feature = "_kinetis")]
fn interrupt_args(peripheral: &Peripheral) -> Option<TokenStream> {
    let instance = Ident::new(peripheral.name, Span::call_site());
    if METADATA.interrupts.iter().any(|(name, _)| *name == peripheral.name) {
        return Some(quote! { #instance, #instance });
    }
    INTMUX_SOURCES
        .iter()
        .find(|(name, _)| *name == peripheral.name)
        .map(|(_, source)| {
            let channel = format_ident!("INTMUX0_0");
            let source = Literal::u8_unsuffixed(*source);
            quote! { #instance, #channel, #source }
        })
}

/// Kinetis LPUART: instance, interrupt (own NVIC line or INTMUX channel), pins.
#[cfg(feature = "_kinetis")]
fn impl_lpuart(impls: &mut Vec<TokenStream>, peripheral: &Peripheral) {
    let instance = Ident::new(peripheral.name, Span::call_site());
    let request = |suffix: &str| {
        let name = format!("{}{}", peripheral.name, suffix);
        let dma = peripheral
            .dma_muxing
            .iter()
            .find(|d| d.signal == name)
            .unwrap_or_else(|| panic!("{} has no DMA request", name));
        Literal::u8_unsuffixed(dma.request)
    };
    let (rx_request, tx_request) = (request("Rx"), request("Tx"));

    impls.push(quote! {
        impl_lpuart_instance!(#instance, #rx_request, #tx_request);
    });

    if let Some(args) = interrupt_args(peripheral) {
        impls.push(quote! {
            impl_lpuart_interrupt!(#args);
        });
    }

    for signal in peripheral.signals {
        let r#macro = match signal.name {
            "TX" => format_ident!("impl_lpuart_tx_pin"),
            "RX" => format_ident!("impl_lpuart_rx_pin"),
            _ => continue,
        };

        for pin in signal.pins {
            let alt = Literal::u8_unsuffixed(pin.alt);
            let pin = format_ident!("{}", pin.pin);
            impls.push(quote! {
                #r#macro!(#pin, #instance, #alt);
            });
        }
    }
}

/// Kinetis DSPI: instance with FIFO depth, interrupt (own NVIC line or INTMUX channel), pins.
#[cfg(feature = "_kinetis")]
fn impl_dspi(impls: &mut Vec<TokenStream>, peripheral: &Peripheral) {
    let instance = Ident::new(peripheral.name, Span::call_site());
    let depth = DSPI_FIFO_DEPTH
        .iter()
        .find(|(name, _)| *name == peripheral.name)
        .map(|(_, depth)| *depth)
        .unwrap_or(1);
    let depth = Literal::u8_unsuffixed(depth);
    let request = |suffix: &str| {
        let name = format!("{}{}", peripheral.name, suffix);
        let dma = peripheral
            .dma_muxing
            .iter()
            .find(|d| d.signal == name)
            .unwrap_or_else(|| panic!("{} has no DMA request", name));
        Literal::u8_unsuffixed(dma.request)
    };
    let (rx_request, tx_request) = (request("Rx"), request("Tx"));

    impls.push(quote! {
        impl_spi_instance!(#instance, #depth, #rx_request, #tx_request);
        impl_spis_instance!(#instance, #depth);
    });

    if let Some(args) = interrupt_args(peripheral) {
        impls.push(quote! {
            impl_spi_interrupt!(#args);
            impl_spis_interrupt!(#args);
        });
    }

    for signal in peripheral.signals {
        let macros = match signal.name {
            "PCS0" => vec![format_ident!("impl_spis_pcs_pin")],
            "SCK" => vec![format_ident!("impl_spi_sck_pin"), format_ident!("impl_spis_sck_pin")],
            "SOUT" => vec![format_ident!("impl_spi_mosi_pin"), format_ident!("impl_spis_sout_pin")],
            "SIN" => vec![format_ident!("impl_spi_miso_pin"), format_ident!("impl_spis_sin_pin")],
            _ => continue,
        };

        for pin in signal.pins {
            let alt = Literal::u8_unsuffixed(pin.alt);
            let pin = format_ident!("{}", pin.pin);
            for r#macro in &macros {
                impls.push(quote! {
                    #r#macro!(#pin, #instance, #alt);
                });
            }
        }
    }
}

/// Kinetis I2C: instance, interrupt (own NVIC line or INTMUX channel), pins.
#[cfg(feature = "_kinetis")]
fn impl_i2c(impls: &mut Vec<TokenStream>, peripheral: &Peripheral) {
    let instance = Ident::new(peripheral.name, Span::call_site());
    let request = peripheral
        .dma_muxing
        .iter()
        .find(|d| d.signal == peripheral.name)
        .unwrap_or_else(|| panic!("{} has no DMA request", peripheral.name));
    let request = Literal::u8_unsuffixed(request.request);

    impls.push(quote! {
        impl_i2c_instance!(#instance, #request);
    });

    if let Some(args) = interrupt_args(peripheral) {
        impls.push(quote! {
            impl_i2c_interrupt!(#args);
        });
    }

    for signal in peripheral.signals {
        let r#macro = match signal.name {
            "SCL" => format_ident!("impl_i2c_scl_pin"),
            "SDA" => format_ident!("impl_i2c_sda_pin"),
            _ => continue,
        };

        for pin in signal.pins {
            let alt = Literal::u8_unsuffixed(pin.alt);
            let pin = format_ident!("{}", pin.pin);
            impls.push(quote! {
                #r#macro!(#pin, #instance, #alt);
            });
        }
    }
}

/// Kinetis LLWU: (bank, pin, LLWU input) for every wakeup capable pin, from the LLWU signals.
#[cfg(feature = "_kinetis")]
fn llwu_pins(peripheral: &Peripheral) -> TokenStream {
    let entries = peripheral.signals.iter().flat_map(|signal| {
        let input = signal.name.strip_prefix("P").unwrap().parse::<u8>().unwrap();
        signal.pins.iter().map(move |pin| {
            let name = pin.pin;
            let bank = format_ident!("Gpio{}", &name[2..3]);
            let number = name[3..].parse::<u8>().unwrap();
            let number = Literal::u8_unsuffixed(number);
            let input = Literal::u8_unsuffixed(input);
            quote! { (crate::gpio::Bank::#bank, #number, #input) }
        })
    });
    quote! {
        pub(crate) const LLWU_PINS: &[(crate::gpio::Bank, u8, u8)] = &[#(#entries),*];
    }
}

fn impl_peripherals(cfgs: &mut common::CfgSet, singletons: &[Singleton]) -> TokenStream {
    let mut impls = Vec::new();

    for peripheral in metadata::METADATA.peripherals.iter() {
        let is_singleton = singletons.iter().any(|s| s.name == peripheral.name);

        if peripheral.name.starts_with("GPIO") {
            impl_gpio_pin(&mut impls, peripheral);
        }

        #[cfg(feature = "_kinetis")]
        if dma_instance(peripheral.name).is_some() {
            impl_dma_channel(&mut impls, peripheral);
        }

        #[cfg(feature = "_kinetis")]
        if is_singleton {
            impl_clock_gate(&mut impls, peripheral);

            if peripheral.name.starts_with("LPUART") {
                impl_lpuart(&mut impls, peripheral);
            }

            if peripheral.name.starts_with("I2C") {
                impl_i2c(&mut impls, peripheral);
            }

            if peripheral.name.starts_with("SPI") {
                impl_dspi(&mut impls, peripheral);
            }

            if peripheral.name.starts_with("TPM") {
                impl_pwm(&mut impls, peripheral);
            }

            if peripheral.name.starts_with("FLEXIO") {
                impl_flexio_pwm(&mut impls, peripheral);
            }

            if peripheral.name.starts_with("ADC") {
                impl_adc16(&mut impls, peripheral);
            }

            if peripheral.name == "LLWU" {
                impls.push(llwu_pins(peripheral));
            }
        }

        // LPC55 peripherals use a different signal naming and FLEXCOMM model.
        #[cfg(not(feature = "_kinetis"))]
        {
            let _ = is_singleton;

            if peripheral.name.starts_with("ADC") {
                impl_adc(&mut impls, peripheral);
            }

            if dma_instance(peripheral.name).is_some() {
                impl_dma_channel(&mut impls, peripheral);
            }

            if peripheral.name.starts_with("USART") {
                impl_usart(cfgs, &mut impls, peripheral);
            }

            if peripheral.name.starts_with("SPI") {
                impl_spi(cfgs, &mut impls, peripheral);
            }

            if peripheral.name.starts_with("SCT") {
                impl_sct(&mut impls, peripheral);
            }
        }
    }

    #[cfg(feature = "_kinetis")]
    let _ = cfgs;

    quote! {
        #(#impls)*
    }
}

/// rustfmt a given path.
/// Failures are logged to stderr and ignored.
fn rustfmt(path: impl AsRef<Path>) {
    let path = path.as_ref();
    match Command::new("rustfmt").args([path]).output() {
        Err(e) => {
            eprintln!("failed to exec rustfmt {:?}: {:?}", path, e);
        }
        Ok(out) => {
            if !out.status.success() {
                eprintln!("rustfmt {:?} failed:", path);
                eprintln!("=== STDOUT:");
                std::io::stderr().write_all(&out.stdout).unwrap();
                eprintln!("=== STDERR:");
                std::io::stderr().write_all(&out.stderr).unwrap();
            }
        }
    }
}

enum GetOneError {
    None,
    Multiple,
}

trait IteratorExt: Iterator {
    fn get_one(self) -> Result<Self::Item, GetOneError>;
}

impl<T: Iterator> IteratorExt for T {
    fn get_one(mut self) -> Result<Self::Item, GetOneError> {
        match self.next() {
            None => Err(GetOneError::None),
            Some(res) => match self.next() {
                Some(_) => Err(GetOneError::Multiple),
                None => Ok(res),
            },
        }
    }
}
