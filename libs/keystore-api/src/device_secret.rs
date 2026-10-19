//! A device-bound secret, derived per caller.
//!
//! The keystore keeps 32 random bytes in a data slot that only its own partition (`Fw0`)
//! can read. `Opcode::DerivedSecret` answers the caller
//! `HKDF-SHA256(ikm = slot, info = caller PID ‖ domain)`, so that every process gets a
//! secret of its own for each domain and none learns the slot. The derivation is pure so
//! that it is unit-tested on the host against the `hkdf` crate.

use hkdf::Hkdf;
use sha2::Sha256;
use zeroize::Zeroize;

/// Length of the slot and of every derived secret.
pub const DEVICE_SECRET_LEN: usize = 32;

/// `HKDF-SHA256` of the slot with no salt; the info is the caller's PID as a 32-bit
/// little-endian integer followed by the domain bytes, so a PID and a domain can never be
/// confused for another pair.
pub fn derive_device_secret(ikm: &[u8; DEVICE_SECRET_LEN], pid: u32, domain: &[u8]) -> [u8; DEVICE_SECRET_LEN] {
    let mut info = Vec::with_capacity(4 + domain.len());
    info.extend_from_slice(&pid.to_le_bytes());
    info.extend_from_slice(domain);
    let mut okm = [0u8; DEVICE_SECRET_LEN];
    Hkdf::<Sha256>::new(None, ikm)
        .expand(&info, &mut okm)
        .expect("32 is a valid length for Sha256 to output");
    info.zeroize();
    okm
}

/// The outcome of a `DerivedSecret` request.
#[derive(Debug, PartialEq, Eq, Clone, Copy, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
pub enum DerivedSecretStatus {
    /// The request has not been answered.
    Pending,
    /// `secret` holds the derivation.
    Ok,
    /// The keystore has no device-bound secret (the slot is empty).
    NotProvisioned,
    /// The request could not be processed.
    InternalError,
}

/// The `DerivedSecret` message: the caller fills `domain`, the keystore fills `secret` and
/// `status`. Zeroed on drop; the caller copies the secret out before dropping it.
#[derive(Debug, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize, Zeroize)]
#[zeroize(drop)]
pub struct DerivedSecret {
    pub domain: String,
    pub secret: [u8; DEVICE_SECRET_LEN],
    #[zeroize(skip)]
    pub status: DerivedSecretStatus,
}

impl DerivedSecret {
    pub fn request(domain: &str) -> Self {
        DerivedSecret { domain: domain.to_owned(), secret: [0u8; DEVICE_SECRET_LEN], status: DerivedSecretStatus::Pending }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ikm() -> [u8; DEVICE_SECRET_LEN] {
        let mut ikm = [0u8; DEVICE_SECRET_LEN];
        for (i, b) in ikm.iter_mut().enumerate() {
            *b = i as u8;
        }
        ikm
    }

    /// Computed once with `hkdf` 0.12.4 / `sha2` 0.10.8 on the host for ikm = 00..1f,
    /// pid = 9, domain = "zsigner-record".
    const VECTOR: [u8; 32] = [
        0x19, 0x2f, 0x34, 0xdf, 0xc9, 0xdd, 0x78, 0xe5, 0xde, 0xce, 0x7f, 0xff, 0xca, 0xe0, 0x5f, 0x04, 0x9f,
        0x8c, 0x67, 0x38, 0x24, 0xee, 0x80, 0x8c, 0x86, 0x07, 0x07, 0xe3, 0x31, 0x6e, 0x72, 0xce,
    ];

    #[test]
    fn matches_the_recorded_vector() {
        assert_eq!(derive_device_secret(&ikm(), 9, b"zsigner-record"), VECTOR);
    }

    #[test]
    fn matches_the_hkdf_crate_with_the_pid_in_front_of_the_domain() {
        let mut expected = [0u8; 32];
        Hkdf::<Sha256>::new(None, &ikm()).expand(b"\x09\x00\x00\x00zsigner-record", &mut expected).unwrap();
        assert_eq!(derive_device_secret(&ikm(), 9, b"zsigner-record"), expected);
    }

    #[test]
    fn another_pid_or_domain_derives_another_secret() {
        let base = derive_device_secret(&ikm(), 9, b"zsigner-record");
        assert_ne!(derive_device_secret(&ikm(), 10, b"zsigner-record"), base);
        assert_ne!(derive_device_secret(&ikm(), 9, b"zsigner-counter"), base);
        assert_ne!(derive_device_secret(&[0xa5; 32], 9, b"zsigner-record"), base);
    }

    #[test]
    fn a_request_starts_pending_and_zeroed() {
        let req = DerivedSecret::request("d");
        assert_eq!(req.status, DerivedSecretStatus::Pending);
        assert_eq!(req.secret, [0u8; 32]);
        assert_eq!(req.domain, "d");
    }
}
