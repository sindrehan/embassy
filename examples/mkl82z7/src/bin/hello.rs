//! Prints chip identification registers over RTT and exits.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embassy_nxp::pac::{MCG, RCM, SIM};
use embassy_nxp_mkl82z7_examples as _;

#[entry]
fn main() -> ! {
    let _p = embassy_nxp::init(Default::default());

    let sdid = SIM.sdid().read();
    defmt::info!(
        "SIM_SDID = {:#010x}: family {} sub-family {} rev {} pins {}",
        sdid.0,
        sdid.familyid().to_bits(),
        sdid.subfamid().to_bits(),
        sdid.revid(),
        sdid.pinid().to_bits(),
    );
    defmt::info!(
        "UID = {:08x}{:08x}{:08x}{:08x}",
        SIM.uidh().read().0,
        SIM.uidmh().read().0,
        SIM.uidml().read().0,
        SIM.uidl().read().0
    );
    defmt::info!(
        "SIM_FCFG1 = {:#010x}, SIM_FCFG2 = {:#010x}",
        SIM.fcfg1().read().0,
        SIM.fcfg2().read().0
    );
    defmt::info!(
        "reset reasons: SRS0 = {:#04x}, SRS1 = {:#04x}",
        RCM.srs0().read().0,
        RCM.srs1().read().0
    );
    defmt::info!(
        "MCG: C1 = {:#04x} C2 = {:#04x} S = {:#04x}",
        MCG.c1().read().0,
        MCG.c2().read().0,
        MCG.s().read().0
    );

    embassy_nxp_mkl82z7_examples::exit()
}
