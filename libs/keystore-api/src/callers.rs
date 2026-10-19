//! First-caller gating of the keystore's application resources.
//!
//! The keystore serves application key slots and application one-way counters to any
//! process that can reach it. A build with `caller-gating` remembers the PID of the first
//! process that touches a gated slot or a gated counter and refuses every other PID from
//! then on; the owner is fixed for the life of the keystore. The gate is pure so that it is
//! unit-tested on the host.

use core::ops::Range;

/// The caller is not the process that first used the resource.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct CallerRefused {
    /// The PID that owns the resource, 0 when the refused call had no sender PID.
    pub owner: u8,
}

/// Which process may use the gated slots and the gated counters.
pub struct CallerGate {
    /// Application key slot indices (relative to the application range) that are gated.
    slots: Range<usize>,
    /// One-way counter indices at or above this are gated.
    counter_floor: usize,
    slot_owner: Option<u8>,
    counter_owner: Option<u8>,
}

impl CallerGate {
    pub const fn new(slots: Range<usize>, counter_floor: usize) -> Self {
        CallerGate { slots, counter_floor, slot_owner: None, counter_owner: None }
    }

    /// Checks `pid` against the owner of the gated slots before an access to slot `index`.
    /// A slot outside the gated range is open to everyone. The first caller with a PID
    /// becomes the owner.
    pub fn check_slot(&mut self, index: usize, pid: u8) -> Result<(), CallerRefused> {
        if !self.slots.contains(&index) {
            return Ok(());
        }
        Self::check(&mut self.slot_owner, pid)
    }

    /// Checks `pid` against the owner of the gated counters before an access to counter
    /// `index`. A counter below the floor is open to everyone (the keystore has its own
    /// rules for those). The first caller with a PID becomes the owner.
    pub fn check_counter(&mut self, index: usize, pid: u8) -> Result<(), CallerRefused> {
        if index < self.counter_floor {
            return Ok(());
        }
        Self::check(&mut self.counter_owner, pid)
    }

    fn check(owner: &mut Option<u8>, pid: u8) -> Result<(), CallerRefused> {
        if pid == 0 {
            return Err(CallerRefused { owner: owner.unwrap_or(0) });
        }
        match *owner {
            None => {
                *owner = Some(pid);
                Ok(())
            }
            Some(current) if current == pid => Ok(()),
            Some(current) => Err(CallerRefused { owner: current }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const KEYS: u8 = 9;
    const UI: u8 = 10;

    fn gate() -> CallerGate { CallerGate::new(0..256, 128) }

    #[test]
    fn the_first_caller_of_a_gated_slot_owns_it() {
        let mut gate = gate();
        assert_eq!(gate.check_slot(0, KEYS), Ok(()));
        assert_eq!(gate.check_slot(255, KEYS), Ok(()));
        assert_eq!(gate.check_slot(17, UI), Err(CallerRefused { owner: KEYS }));
        assert_eq!(gate.check_slot(17, KEYS), Ok(()), "the owner keeps its access after a refusal");
    }

    #[test]
    fn a_slot_outside_the_gated_range_is_open_and_claims_nothing() {
        let mut gate = gate();
        assert_eq!(gate.check_slot(256, UI), Ok(()));
        assert_eq!(gate.check_slot(1535, UI), Ok(()));
        assert_eq!(gate.check_slot(0, KEYS), Ok(()), "the UI's open access did not make it the owner");
        assert_eq!(gate.check_slot(256, UI), Ok(()));
    }

    #[test]
    fn the_first_caller_of_a_gated_counter_owns_the_counters() {
        let mut gate = gate();
        assert_eq!(gate.check_counter(128, KEYS), Ok(()));
        assert_eq!(gate.check_counter(255, UI), Err(CallerRefused { owner: KEYS }));
        assert_eq!(gate.check_counter(200, KEYS), Ok(()));
    }

    #[test]
    fn counters_below_the_floor_are_not_gated() {
        let mut gate = gate();
        assert_eq!(gate.check_counter(85, UI), Ok(()));
        assert_eq!(gate.check_counter(127, UI), Ok(()));
        assert_eq!(gate.check_counter(128, KEYS), Ok(()), "the open access did not make the UI the owner");
    }

    #[test]
    fn slots_and_counters_have_owners_of_their_own() {
        let mut gate = gate();
        assert_eq!(gate.check_slot(0, KEYS), Ok(()));
        assert_eq!(gate.check_counter(128, UI), Ok(()));
        assert_eq!(gate.check_counter(128, KEYS), Err(CallerRefused { owner: UI }));
    }

    #[test]
    fn a_call_without_a_sender_pid_is_refused_and_owns_nothing() {
        let mut gate = gate();
        assert_eq!(gate.check_slot(0, 0), Err(CallerRefused { owner: 0 }));
        assert_eq!(gate.check_slot(0, KEYS), Ok(()));
        assert_eq!(gate.check_slot(0, 0), Err(CallerRefused { owner: KEYS }));
    }
}
