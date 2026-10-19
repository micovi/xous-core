use crate::offsets::{PartitionAccess, RwPerms, SlotIndex};

// no external SPI
pub const SPI_FLASH_LEN: usize = 0;
pub const SPI_FLASH_ID_MASK: u32 = 0xff_ff_ff;
pub const SPI_FLASH_IDS: [u32; 0] = [];

// no external RAM
pub const SWAP_RAM_LEN: usize = 0;
pub const SWAP_RAM_ID_MASK: u32 = 0xff_ff;
// KGD 5D, mfg ID 9D; remainder of bits are part of the EID
pub const SWAP_RAM_IDS: [u32; 0] = [];

// "Partition table" of external SPI FLASH
pub const SWAP_FLASH_ORIGIN: usize = 0x0000_0000;
pub const SWAP_FLASH_RESERVED_LEN: usize = 0;
pub const APP_FLASH_ORIGIN: usize = 0;
pub const APP_FLASH_RESERVED_LEN: usize = 0;

// No PDDB, because no FLASH
pub const PDDB_ORIGIN: usize = 0;
pub const PDDB_LEN: usize = 0;

// Location of on-chip application segment, as offset from RRAM start
pub const APP_RRAM_OFFSET: usize = 0x20_0000;
pub const APP_RRAM_START: usize = 0x6000_0000 + APP_RRAM_OFFSET - crate::signatures::SIGBLOCK_LEN;
pub const APP_RRAM_LEN: usize = crate::offsets::RRAM_STORAGE_LEN - APP_RRAM_OFFSET + crate::signatures::SIGBLOCK_LEN;

// Regulator voltage target at boot
pub const CPU_VDD_LDO_BOOT_MV: u32 = 810;
pub const DEFAULT_FCLK_FREQUENCY: u32 = 700_000_000;

/// All the slots of concern located in a single iterator. The idea is that everything is
/// condensed here and used to check for access integrity using the array below.
pub const DATA_SLOTS: [SlotIndex; 14] = [
    crate::offsets::SERIAL_NUMBER,
    crate::offsets::UUID,
    crate::offsets::IFR_HASH,
    crate::offsets::CP_ID,
    crate::offsets::BAO1_PUBKEY,
    crate::offsets::BAO2_PUBKEY,
    crate::offsets::BETA_PUBKEY,
    crate::offsets::DEV_PUBKEY,
    crate::offsets::BOOT1_PK_RECEIPT_SLOT0,
    crate::offsets::BOOT1_PK_RECEIPT_SLOT1,
    crate::offsets::BOOT1_PK_RECEIPT_SLOT2,
    crate::offsets::BOOT1_PK_RECEIPT_SLOT3,
    crate::offsets::COLLATERAL_SECRET,
    crate::offsets::COLLATERAL_PUBLIC,
];

pub use crate::baosec::ERASE_PROOF;
pub use crate::baosec::{
    CHAFF_KEYS, NUISANCE_KEYS, NUISANCE_KEYS_0, NUISANCE_KEYS_1, RMA_KEY, ROOT_SEED, THE_FLAG_1,
};

pub const KEY_SLOTS: [SlotIndex; 6] =
    [THE_FLAG_1, ROOT_SEED, RMA_KEY, NUISANCE_KEYS_0, NUISANCE_KEYS_1, CHAFF_KEYS];

/// A device-bound secret the keystore fills from the TRNG on its first run and serves only
/// as per-caller HKDF derivations (`keystore-api`'s `DerivedSecret`, feature `device-secret`).
///
/// Why this slot: it is the last of the `[274..=383]` range the common page leaves for
/// future data slots, as far as possible from a provisioning block that would grow up from
/// 274; it is outside `APPLICATION` (384 and up), and it is deliberately NOT in `KEY_SLOTS`:
/// boot0's `erase_secrets` (`bao1x-hal/src/sigcheck.rs`) iterates `KEY_SLOTS` only, so the
/// per-boot erase of developer mode leaves this slot, and its ACL, as they are. The ACL
/// admits the keystore's own partition (`Fw0`) alone; boot1's console and a raw read of the
/// slot page from any other user are refused by the hardware.
///
/// <slot-map targets="dabao" registered="primary" in-key-slots="no" in-data-slots="no"/>
pub const DEVICE_BOUND_SECRET: SlotIndex = SlotIndex::Data(383, PartitionAccess::Fw0, RwPerms::ReadWrite);

// offsets into the application slot range for storing BIO config
pub const APP_BIO_CLK_INDEX: usize = 0;
pub const APP_BIO_PINS_INDEX: usize = 1;
pub const APP_BIO_CODE_VALID: usize = 2;
// total length is 0xf00 = 120 slots
pub const APP_BIO_CODE_INDICES: core::ops::Range<usize> = 3..123;
