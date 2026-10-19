//! Ownership of I/O pins and UDMA peripherals by process.
//!
//! The HAL service keeps one of these tables. A process claims the pins it drives or reads
//! (`HalOpcode::ClaimIox`); from then on the service refuses to reconfigure, write or watch
//! (edge interrupts) those pins for any other process and hides their input values from it.
//! A process also claims the UDMA peripherals it drives (`HalOpcode::ClaimPeriph`); the
//! service then refuses to reset them, gate their clock or remap their events for any other
//! process. A claim lasts for the life of the process; nothing releases it. The table is
//! pure so that it can be tested on the host.

use crate::iox::IoxPort;
use crate::udma::PeriphId;

/// Ports in the I/O crossbar (`IoxPort::PA` to `IoxPort::PF`).
pub const PORTS: usize = 6;
/// Pins per port.
pub const PINS_PER_PORT: usize = 16;
/// UDMA peripherals, one per bit of `PeriphId` (`Uart0` = bit 0 to `Adc` = bit 19).
pub const PERIPHS: usize = 20;

/// Why a claim was refused.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum ClaimError {
    /// These pins of the requested mask are owned by another process. Nothing was claimed.
    Owned { pins: u16 },
}

/// Why a peripheral claim was refused.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum PeriphClaimError {
    /// Another process owns the peripheral.
    Owned,
}

/// The owner, by PID, of every pin and every UDMA peripheral. 0 means unclaimed.
pub struct IoxClaims {
    owners: [[u8; PINS_PER_PORT]; PORTS],
    periph_owners: [u8; PERIPHS],
}

impl IoxClaims {
    pub const fn new() -> Self {
        IoxClaims { owners: [[0; PINS_PER_PORT]; PORTS], periph_owners: [0; PERIPHS] }
    }

    /// Claims every pin set in `pins` for `pid`. Pins already owned by `pid` stay owned; a
    /// pin owned by another process refuses the whole claim and leaves the table unchanged.
    /// Bits above `PINS_PER_PORT` are ignored.
    pub fn claim(&mut self, port: IoxPort, pins: u16, pid: u8) -> Result<(), ClaimError> {
        let contested = pins & self.others_mask(port, pid);
        if contested != 0 {
            return Err(ClaimError::Owned { pins: contested });
        }
        let owners = &mut self.owners[port as usize];
        for pin in 0..PINS_PER_PORT {
            if pins & (1 << pin) != 0 {
                owners[pin] = pid;
            }
        }
        Ok(())
    }

    /// `true` when `pid` may configure or write `pin`: it owns it, or nobody does.
    pub fn allows(&self, port: IoxPort, pin: u8, pid: u8) -> bool {
        if pin as usize >= PINS_PER_PORT {
            return true;
        }
        let owner = self.owners[port as usize][pin as usize];
        owner == 0 || owner == pid
    }

    /// `pins` with the bits owned by other processes cleared: what a write from `pid` may touch.
    pub fn write_mask(&self, port: IoxPort, pins: u16, pid: u8) -> u16 { pins & !self.others_mask(port, pid) }

    /// The bits `pid` may read: everything but the pins owned by other processes.
    pub fn read_mask(&self, port: IoxPort, pid: u8) -> u16 { !self.others_mask(port, pid) }

    /// Claims `periph` for `pid`. A repeated claim by the owner succeeds; a claim of a
    /// peripheral owned by another process is refused. PID 0 never owns anything.
    pub fn claim_periph(&mut self, periph: PeriphId, pid: u8) -> Result<(), PeriphClaimError> {
        if !self.periph_allows(periph, pid) {
            return Err(PeriphClaimError::Owned);
        }
        self.periph_owners[periph_index(periph)] = pid;
        Ok(())
    }

    /// `true` when `pid` may reset `periph`, gate its clock or remap its events: it owns
    /// it, or nobody does.
    pub fn periph_allows(&self, periph: PeriphId, pid: u8) -> bool {
        let owner = self.periph_owners[periph_index(periph)];
        owner == 0 || owner == pid
    }

    fn others_mask(&self, port: IoxPort, pid: u8) -> u16 {
        let mut mask = 0u16;
        for (pin, &owner) in self.owners[port as usize].iter().enumerate() {
            if owner != 0 && owner != pid {
                mask |= 1 << pin;
            }
        }
        mask
    }
}

/// The bit position of `periph` in `PeriphId`, which is its slot in the table.
fn periph_index(periph: PeriphId) -> usize { (periph as u32).trailing_zeros() as usize }

#[cfg(test)]
mod tests {
    use super::*;

    const TUI: u8 = 8;
    const UI: u8 = 10;

    #[test]
    fn a_claim_records_the_owner_and_may_be_repeated() {
        let mut claims = IoxClaims::new();
        assert_eq!(claims.claim(IoxPort::PB, 0b11, TUI), Ok(()));
        assert_eq!(claims.claim(IoxPort::PB, 0b11, TUI), Ok(()));
        assert!(claims.allows(IoxPort::PB, 0, TUI));
        assert!(!claims.allows(IoxPort::PB, 0, UI));
        assert!(!claims.allows(IoxPort::PB, 1, UI));
    }

    #[test]
    fn another_process_cannot_claim_an_owned_pin_and_nothing_is_claimed() {
        let mut claims = IoxClaims::new();
        claims.claim(IoxPort::PB, 0b0010, TUI).unwrap();
        assert_eq!(claims.claim(IoxPort::PB, 0b0110, UI), Err(ClaimError::Owned { pins: 0b0010 }));
        assert!(claims.allows(IoxPort::PB, 2, TUI), "the uncontested pin of the refused claim stays free");
        assert_eq!(claims.claim(IoxPort::PB, 0b0100, TUI), Ok(()));
    }

    #[test]
    fn unclaimed_pins_and_other_ports_stay_open() {
        let mut claims = IoxClaims::new();
        claims.claim(IoxPort::PB, 0b1, TUI).unwrap();
        assert!(claims.allows(IoxPort::PB, 5, UI));
        assert!(claims.allows(IoxPort::PC, 0, UI));
        assert_eq!(claims.write_mask(IoxPort::PC, 0xffff, UI), 0xffff);
        assert_eq!(claims.read_mask(IoxPort::PC, UI), 0xffff);
    }

    #[test]
    fn a_write_mask_drops_the_pins_of_other_owners() {
        let mut claims = IoxClaims::new();
        claims.claim(IoxPort::PB, 0b1001, TUI).unwrap();
        claims.claim(IoxPort::PB, 0b0100, UI).unwrap();
        assert_eq!(claims.write_mask(IoxPort::PB, 0b1111, UI), 0b0110);
        assert_eq!(claims.write_mask(IoxPort::PB, 0b1111, TUI), 0b1011);
        assert_eq!(claims.write_mask(IoxPort::PB, 0b1001, UI), 0);
    }

    #[test]
    fn a_read_mask_hides_the_pins_of_other_owners() {
        let mut claims = IoxClaims::new();
        claims.claim(IoxPort::PB, 0b1001, TUI).unwrap();
        assert_eq!(claims.read_mask(IoxPort::PB, UI), !0b1001);
        assert_eq!(claims.read_mask(IoxPort::PB, TUI), 0xffff);
        assert_eq!(0xffff & claims.read_mask(IoxPort::PB, UI), 0xfff6);
    }

    #[test]
    fn a_pin_beyond_the_port_is_ignored() {
        let mut claims = IoxClaims::new();
        assert_eq!(claims.claim(IoxPort::PA, 0xffff, TUI), Ok(()));
        assert!(claims.allows(IoxPort::PA, 16, UI));
        assert!(claims.allows(IoxPort::PA, 255, UI));
    }

    #[test]
    fn a_zero_pid_never_owns_anything() {
        let mut claims = IoxClaims::new();
        claims.claim(IoxPort::PA, 0b1, 0).unwrap();
        assert!(claims.allows(IoxPort::PA, 0, UI));
        assert_eq!(claims.read_mask(IoxPort::PA, UI), 0xffff);
    }

    #[test]
    fn an_edge_interrupt_on_a_claimed_pin_is_refused_to_others() {
        let mut claims = IoxClaims::new();
        claims.claim(IoxPort::PB, 1 << 4 | 1 << 5, TUI).unwrap();
        assert!(!claims.allows(IoxPort::PB, 4, UI), "the UI may not watch a button");
        assert!(claims.allows(IoxPort::PB, 4, TUI), "the owner may");
        assert!(claims.allows(IoxPort::PB, 6, UI), "an unclaimed pin stays open");
    }

    #[test]
    fn an_edge_interrupt_registered_before_a_claim_is_no_longer_allowed_after_it() {
        let mut claims = IoxClaims::new();
        assert!(claims.allows(IoxPort::PB, 4, UI), "before the claim the UI may register");
        claims.claim(IoxPort::PB, 1 << 4, TUI).unwrap();
        assert!(!claims.allows(IoxPort::PB, 4, UI), "the HAL revokes what the table refuses");
    }

    #[test]
    fn a_peripheral_claim_records_the_owner_and_may_be_repeated() {
        let mut claims = IoxClaims::new();
        assert_eq!(claims.claim_periph(PeriphId::Spim2, TUI), Ok(()));
        assert_eq!(claims.claim_periph(PeriphId::Spim2, TUI), Ok(()));
        assert!(claims.periph_allows(PeriphId::Spim2, TUI));
        assert!(!claims.periph_allows(PeriphId::Spim2, UI));
    }

    #[test]
    fn another_process_cannot_claim_an_owned_peripheral() {
        let mut claims = IoxClaims::new();
        claims.claim_periph(PeriphId::Spim2, TUI).unwrap();
        assert_eq!(claims.claim_periph(PeriphId::Spim2, UI), Err(PeriphClaimError::Owned));
        assert!(claims.periph_allows(PeriphId::Spim2, TUI), "the refused claim changes nothing");
    }

    #[test]
    fn other_peripherals_stay_open() {
        let mut claims = IoxClaims::new();
        claims.claim_periph(PeriphId::Spim2, TUI).unwrap();
        for periph in [PeriphId::Uart0, PeriphId::Spim1, PeriphId::Spim3, PeriphId::I2c0, PeriphId::Adc] {
            assert!(claims.periph_allows(periph, UI), "{periph:?} is not claimed");
        }
        assert_eq!(claims.claim_periph(PeriphId::Adc, UI), Ok(()));
        assert!(!claims.periph_allows(PeriphId::Adc, TUI));
    }

    #[test]
    fn a_zero_pid_never_owns_a_peripheral() {
        let mut claims = IoxClaims::new();
        claims.claim_periph(PeriphId::Spim2, 0).unwrap();
        assert!(claims.periph_allows(PeriphId::Spim2, UI));
        assert_eq!(claims.claim_periph(PeriphId::Spim2, TUI), Ok(()));
    }

    #[test]
    fn every_peripheral_has_its_own_slot() {
        let all = [
            PeriphId::Uart0,
            PeriphId::Uart1,
            PeriphId::Uart2,
            PeriphId::Uart3,
            PeriphId::Spim0,
            PeriphId::Spim1,
            PeriphId::Spim2,
            PeriphId::Spim3,
            PeriphId::I2c0,
            PeriphId::I2c1,
            PeriphId::I2c2,
            PeriphId::I2c3,
            PeriphId::Sdio,
            PeriphId::I2s,
            PeriphId::Cam,
            PeriphId::Filter,
            PeriphId::Scif,
            PeriphId::Spis0,
            PeriphId::Spis1,
            PeriphId::Adc,
        ];
        assert_eq!(all.len(), PERIPHS);
        for (slot, periph) in all.iter().enumerate() {
            assert_eq!(periph_index(*periph), slot);
        }
    }
}
