//! The register pages of the memory-mastering blocks that nothing in the Dabao image
//! programs, mapped by this process at its start and kept for its life (feature `mmio-hold`).
//!
//! The kernel gives a physical page of a declared memory region to the first process that
//! maps it and refuses it to every other process while it stays mapped. For each block below,
//! every register that sets a DMA address, a length or a start lies in the one page listed
//! for it (in the SVD, each of these blocks ends below offset 0x1000), so a process that
//! cannot map that page cannot point the block at memory.
//!
//! Not held here, because a process of the image maps them: UART2 (xous-log, at its start),
//! SPIM2 (the application's trusted UI, after this service starts), I2C0 and the ADC (the I2C
//! and peripheral threads of this process), the UDMA global control page (this process), the
//! USB controller (usb-bao1x) and the BIO (this process under `bio-locked`, or the BIO server).

use bao1x_api::PeriphId;
use utralib::generated::*;

#[cfg(not(feature = "board-dabao"))]
compile_error!("mmio-hold lists the blocks the Dabao image leaves unused; build it with board-dabao");

const PAGE: usize = 4096;
/// The UDMA peripherals, one bit each in `PeriphId` (the clock-gate register's layout).
const UDMA_PERIPHERALS: u32 = (1 << 20) - 1;

/// The UDMA peripherals nothing in the image uses, each with its register page. A UDMA
/// channel's buffer address, size and start are written in its peripheral's page
/// (`udma::Udma::udma_enqueue`); the global control page only gates clocks, resets and
/// routes events.
const UNUSED_UDMA: [(PeriphId, usize); 16] = [
    (PeriphId::Uart0, HW_UDMA_UART_0_BASE),
    (PeriphId::Uart1, HW_UDMA_UART_1_BASE),
    (PeriphId::Uart3, HW_UDMA_UART_3_BASE),
    (PeriphId::Spim0, HW_UDMA_SPIM_0_BASE),
    (PeriphId::Spim1, HW_UDMA_SPIM_1_BASE),
    (PeriphId::Spim3, HW_UDMA_SPIM_3_BASE),
    (PeriphId::I2c1, HW_UDMA_I2C_1_BASE),
    (PeriphId::I2c2, HW_UDMA_I2C_2_BASE),
    (PeriphId::I2c3, HW_UDMA_I2C_3_BASE),
    (PeriphId::Sdio, HW_UDMA_SDIO_BASE),
    (PeriphId::I2s, HW_UDMA_I2S_BASE),
    (PeriphId::Cam, HW_UDMA_CAMERA_BASE),
    (PeriphId::Filter, HW_UDMA_FILTER_BASE),
    (PeriphId::Scif, HW_UDMA_SCIF_BASE),
    (PeriphId::Spis0, HW_UDMA_SPIS_0_BASE),
    (PeriphId::Spis1, HW_UDMA_SPIS_1_BASE),
];

/// The UDMA peripherals a process of the image maps, so left alone.
const USED_UDMA: [(PeriphId, usize); 4] = [
    (PeriphId::Uart2, HW_UDMA_UART_2_BASE),
    (PeriphId::Spim2, HW_UDMA_SPIM_2_BASE),
    (PeriphId::I2c0, HW_UDMA_I2C_0_BASE),
    (PeriphId::Adc, HW_UDMA_ADC_BASE),
];

/// The other memory masters nothing in the image uses, with their register pages: the PL230
/// (its control-structure pointers, channel enables and software requests), the MDMA (the
/// event selection that raises PL230 requests), the SCE DMA (the AXI start addresses, sizes
/// and start of its channels; the SCE engines themselves only take segment pointers) and the
/// SD device controller (its base-address registers).
const UNUSED_MASTERS: [(&str, usize); 4] = [
    ("PL230 DMA", HW_PL230_BASE),
    ("MDMA", HW_MDMA_BASE),
    ("SCE DMA", HW_SCEDMA_BASE),
    ("SD device controller", HW_SDDC_BASE),
];

/// The register page of a UDMA peripheral: the pages follow the global control page in the
/// order of the `PeriphId` bits.
const fn udma_page(periph: PeriphId) -> usize {
    HW_UDMA_CTRL_BASE + PAGE * ((periph as u32).trailing_zeros() as usize + 1)
}

/// Every UDMA peripheral is either held or used, none twice, and each listed page is the
/// peripheral's own.
const fn udma_lists_cover_every_peripheral() -> bool {
    let mut seen = 0u32;
    let mut i = 0;
    while i < UNUSED_UDMA.len() + USED_UDMA.len() {
        let (periph, page) =
            if i < UNUSED_UDMA.len() { UNUSED_UDMA[i] } else { USED_UDMA[i - UNUSED_UDMA.len()] };
        let bit = periph as u32;
        if seen & bit != 0 || udma_page(periph) != page {
            return false;
        }
        seen |= bit;
        i += 1;
    }
    seen == UDMA_PERIPHERALS
}

/// The other masters' pages are page-aligned and outside the UDMA block.
const fn other_masters_are_pages_outside_udma() -> bool {
    let mut i = 0;
    while i < UNUSED_MASTERS.len() {
        let page = UNUSED_MASTERS[i].1;
        if page % PAGE != 0 || (page >= HW_UDMA_CTRL_BASE && page < HW_UDMA_CTRL_BASE + HW_UDMA_MEM_LEN) {
            return false;
        }
        i += 1;
    }
    true
}

const _: () =
    assert!(udma_lists_cover_every_peripheral(), "a UDMA peripheral is missing, repeated or misplaced");
const _: () = assert!(other_masters_are_pages_outside_udma(), "a held page is misplaced");

/// Maps every page listed above into this process. The returned ranges are never unmapped,
/// so no other process can map them while this process lives.
///
/// Panics if a page is already mapped by another process: the image would then leave that
/// block open, and it should not start.
pub fn hold() -> Vec<xous::MemoryRange> {
    let mut held = Vec::with_capacity(UNUSED_UDMA.len() + UNUSED_MASTERS.len());
    for (periph, page) in UNUSED_UDMA {
        held.push(hold_page(&periph, page));
    }
    for (name, page) in UNUSED_MASTERS {
        held.push(hold_page(&name, page));
    }
    log::info!("mmio-hold: {} register pages of unused DMA masters held", held.len());
    held
}

fn hold_page(block: &dyn core::fmt::Debug, page: usize) -> xous::MemoryRange {
    xous::syscall::map_memory(
        xous::MemoryAddress::new(page),
        None,
        PAGE,
        xous::MemoryFlags::R | xous::MemoryFlags::W,
    )
    .unwrap_or_else(|e| {
        panic!("mmio-hold: the {:?} page 0x{:x} is mapped by another process ({:?})", block, page, e)
    })
}
