mod api;
#[cfg(feature = "mmio-hold")]
mod mmio_hold;
mod servers;

use std::pin::Pin;

use bao1x_api::*;
use bao1x_hal::{iox::Iox, udma::GlobalConfig};
use bitfield::*;
use num_traits::*;
use udma::UdmaGlobalConfig;
use utralib::utra;
use utralib::*;
#[cfg(feature = "swap")]
use xous::SWAPPER_PID;
use xous::{ScalarMessage, sender::Sender};

#[repr(u32)]
#[allow(dead_code)]
enum IntMode {
    RisingEdge = 0,
    FallingEdge = 1,
    HighLevel = 2,
    LowLevel = 3,
}
bitfield! {
    #[derive(Copy, Clone, PartialEq, Eq, Default)]
    pub struct IntCr(u32);
    impl Debug;
    pub u32, select, set_select: 6, 0;
    pub u32, mode, set_mode: 8, 7;
    pub enable, set_enable: 9;
    pub wakeup, set_wakeup: 10;
}
fn irq_select_from_port(port: IoxPort, pin: u8) -> u32 { (port as u32) * 16 + pin as u32 }
/// The PID the kernel stamped on the message; 0 when there is none (a kernel-originated message).
fn sender_pid(sender: &Sender) -> u8 { sender.pid().map(|pid| pid.get()).unwrap_or(0) }
fn find_first_none<T>(arr: &[Option<T>]) -> Option<usize> { arr.iter().position(|item| item.is_none()) }
/// Pins per IOX port; the IOX register helpers assert a pin below this.
const IOX_PINS: u8 = 16;
/// Interrupt slots of the IOX; each one is a `SFR_INTCR_CRINTn` register.
const IOX_IRQ_SLOTS: usize = 8;

/// Writes the interrupt control register of IOX interrupt slot `index`.
fn write_int_cr(iox: &Iox, index: usize, value: u32) {
    assert!(index < IOX_IRQ_SLOTS, "IOX interrupt slot out of range");
    // safety: the index is bounded by the number of slots, and the offset comes from the
    // generated header file.
    unsafe {
        iox.csr.base().add(utralib::utra::iox::SFR_INTCR_CRINT0.offset()).add(index).write_volatile(value);
    }
}

/// The IFRAM bank, first page and page count of the `size` bytes at `phys_addr`, when they
/// start on a page boundary and lie inside one bank; `None` otherwise.
fn ifram_pages(phys_addr: usize, size: usize) -> Option<(usize, usize, usize)> {
    let banks = [
        (utralib::generated::HW_IFRAM0_MEM, utralib::generated::HW_IFRAM0_MEM_LEN),
        (utralib::generated::HW_IFRAM1_MEM, utralib::generated::HW_IFRAM1_MEM_LEN),
    ];
    for (bank, (base, len)) in banks.into_iter().enumerate() {
        if phys_addr < base || phys_addr >= base + len {
            continue;
        }
        let offset = phys_addr - base;
        let count = size.div_ceil(4096);
        if offset % 4096 != 0 || count == 0 || offset / 4096 + count > len / 4096 {
            return None;
        }
        return Some((bank, offset / 4096, count));
    }
    None
}

#[derive(Debug)]
#[allow(dead_code)]
struct IrqLocalRegistration {
    /// The PID that registered the interrupt; 0 when the kernel stamped none.
    pub pid: u8,
    pub cid: xous::CID,
    pub opcode: usize,
    pub port: IoxPort,
    pub pin: u8,
    pub active: IoxValue,
    pub server: String,
}

struct IrqHandler {
    pub irq_csr: CSR<u32>,
    pub cid: xous::CID,
}
fn iox_irq_handler(_irq_no: usize, arg: *mut usize) {
    let handler = unsafe { &mut *(arg as *mut IrqHandler) };
    let pending = handler.irq_csr.r(utralib::utra::irqarray10::EV_PENDING);
    handler.irq_csr.wo(utralib::utra::irqarray10::EV_PENDING, pending);
    xous::try_send_message(
        handler.cid,
        xous::Message::Scalar(ScalarMessage::from_usize(
            HalOpcode::IrqLocalHandler.to_usize().unwrap(),
            pending as usize,
            0,
            0,
            0,
        )),
    )
    .ok();
}

/// Preemption timer call
fn timer_tick(_irq_no: usize, arg: *mut usize) {
    let mut timer = CSR::new(arg as *mut u32);
    // this call forces preemption every timer tick
    // rsyscalls are "raw syscalls" -- used for syscalls that don't have a friendly wrapper around them
    // since ReturnToParent is only used here, we haven't wrapped it, so we use an rsyscall
    xous::rsyscall(xous::SysCall::ReturnToParent(xous::PID::new(1).unwrap(), 0))
        .expect("couldn't return to parent");

    // acknowledge the timer
    timer.wfo(utra::timer0::EV_PENDING_ZERO, 0b1);
}

fn try_alloc(ifram_allocs: &mut Vec<Option<Sender>>, size: usize, sender: Sender) -> Option<usize> {
    let mut size_pages = size / 4096;
    if size % 4096 != 0 {
        size_pages += 1;
    }
    log::trace!("try_alloc search for {} pages in alloc vector {:?}", size_pages, ifram_allocs);
    let mut free_start = None;
    let mut found_len = 0;
    for (index, page) in ifram_allocs.iter().enumerate() {
        log::trace!("Checking index {}: {:?}", index, page);
        if page.is_some() {
            log::trace!("Page was allocated, restarting search");
            free_start = None;
            found_len = 0;
            continue;
        } else {
            if free_start.is_some() {
                log::trace!("Adding unallocated page at {} to length", index);
                if found_len >= size_pages {
                    break;
                }
                found_len += 1;
            } else {
                log::trace!("Starting allocation search at {}", index);
                free_start = Some(index);
                found_len = 1;
            }
        }
    }
    if let Some(start) = free_start {
        if found_len >= size_pages {
            // starting point found, and enough pages
            assert!(
                found_len == size_pages,
                "Found pages should be exactly equal to size_pages at this point"
            );
            for i in ifram_allocs[start..start + found_len].iter_mut() {
                *i = Some(sender);
            }
            // offset relative to start of IFRAM bank
            Some(start * 4096)
        } else {
            // starting point found, but not enough pages
            None
        }
    } else {
        // no starting point found
        None
    }
}
fn main() {
    #[cfg(feature = "with-duart")]
    bao1x_hal::claim_duart();
    log_server::init_wait().unwrap();
    log::set_max_level(log::LevelFilter::Info);
    log::info!("my PID is {}", xous::process::id());

    // Before the name is registered: whatever waits for this service runs after the pages
    // are held.
    #[cfg(feature = "mmio-hold")]
    let _held = mmio_hold::hold();

    let xns = xous_names::XousNames::new().unwrap();
    let sid = xns.register_name(bao1x_api::SERVER_NAME_BAO1X_HAL, None).expect("can't register server");
    let self_cid = xous::connect(sid).expect("couldn't create self-connection");

    servers::susres::start_susres_service();

    // these two servers need to start so that update_perclk calls can succeed
    servers::i2c::start_i2c_service();
    servers::others::start_peri_service();

    std::thread::spawn(move || {
        let xns = xous_names::XousNames::new().unwrap();
        let mut ifram_allocs = [Vec::new(), Vec::new()];
        // code is written assuming the IFRAM blocks have the same size. Since this is fixed in
        // hardware, it's a good assumption; but the assert is put here in case we port this to
        // a new system where for some reason they have different sizes.
        assert!(utralib::generated::HW_IFRAM0_MEM_LEN == utralib::generated::HW_IFRAM1_MEM_LEN);
        let pages = utralib::generated::HW_IFRAM0_MEM_LEN / 4096;
        for _ in 0..pages {
            ifram_allocs[0].push(None);
            ifram_allocs[1].push(None);
        }
        // mark loader-hardwired pages for IFRAM0
        for i in
            bao1x_hal::board::IFRAM0_RESERVED_PAGE_RANGE[0]..=bao1x_hal::board::IFRAM0_RESERVED_PAGE_RANGE[1]
        {
            ifram_allocs[0][i] = Some(Sender::from_usize(usize::MAX));
        }
        // mark loader-hardwired pages for IFRAM1
        for i in
            bao1x_hal::board::IFRAM1_RESERVED_PAGE_RANGE[0]..=bao1x_hal::board::IFRAM1_RESERVED_PAGE_RANGE[1]
        {
            ifram_allocs[1][i] = Some(Sender::from_usize(usize::MAX));
        }
        // Second page from top of IFRAM0 is occupied by the swap handler. This was allocated
        // by the loader, before the kernel even started. Mark the PID properly.
        #[cfg(feature = "swap")]
        {
            ifram_allocs[0][30] = Some(Sender::from_usize(SWAPPER_PID as usize));
        }

        let iox_page = xous::syscall::map_memory(
            xous::MemoryAddress::new(utralib::generated::HW_IOX_BASE),
            None,
            4096,
            xous::MemoryFlags::R | xous::MemoryFlags::W,
        )
        .expect("couldn't claim the IOX hardware page");
        let iox = Iox::new(iox_page.as_ptr() as *mut u32);
        // No pin is routed to the BIO in this build (see `ConfigureBio` below).
        #[cfg(feature = "bio-locked")]
        iox.csr.wo(utra::iox::SFR_PIOSEL, 0);

        let udma_global = GlobalConfig::new();

        // os timer initializations
        let ostimer_csr = xous::syscall::map_memory(
            xous::MemoryAddress::new(utra::timer0::HW_TIMER0_BASE),
            None,
            4096,
            xous::MemoryFlags::R | xous::MemoryFlags::W,
        )
        .expect("couldn't map Ticktimer CSR range");
        xous::claim_interrupt(
            utralib::utra::timer0::TIMER0_IRQ,
            timer_tick,
            ostimer_csr.as_mut_ptr() as *mut usize,
        )
        .expect("couldn't claim IRQ");

        // setup the OS timer
        let mut timer_run = false;
        let mut os_timer = CSR::new(ostimer_csr.as_ptr() as *mut u32);
        let ms = bao1x_api::SYSTEM_TICK_INTERVAL_MS;
        os_timer.wfo(utra::timer0::EN_EN, 0b0); // disable the timer
        // load its values
        os_timer.wfo(utra::timer0::LOAD_LOAD, (bao1x_hal::board::DEFAULT_FCLK_FREQUENCY / 1_000) * ms);
        os_timer.wfo(utra::timer0::RELOAD_RELOAD, (bao1x_hal::board::DEFAULT_FCLK_FREQUENCY / 1_000) * ms);
        // enable the timer
        os_timer.wfo(utra::timer0::EN_EN, 0b1);

        // Set EV_ENABLE according to timer_run
        // timer is OFF at boot to help insure that processes 2 & 3 get larger time slices
        // to operate & claim critical resources
        os_timer.wfo(utra::timer0::EV_ENABLE_ZERO, if timer_run { 1 } else { 0 });

        // ---- "own" the Iox IRQ bank. This might need revision once bao1x aliasing is available. ---
        let irq_page = xous::syscall::map_memory(
            xous::MemoryAddress::new(utralib::utra::irqarray10::HW_IRQARRAY10_BASE),
            None,
            4096,
            xous::MemoryFlags::R | xous::MemoryFlags::W,
        )
        .expect("couldn't claim IRQ control page");
        let irq_csr = CSR::new(irq_page.as_mut_ptr() as *mut u32);
        let mut irq = Box::pin(IrqHandler { irq_csr, cid: self_cid });
        // ensure that iox interrupts are gated off by default
        for index in 0..IOX_IRQ_SLOTS {
            write_int_cr(&iox, index, 0);
        }
        iox.csr.wo(utra::iox::SFR_INTFR, 0);
        xous::claim_interrupt(
            utralib::utra::irqarray10::IRQARRAY10_IRQ,
            iox_irq_handler,
            Pin::as_mut(&mut irq).get_mut() as *mut IrqHandler as *mut usize,
        )
        .expect("couldn't claim Iox interrupt");
        irq.irq_csr.wo(utralib::utra::irqarray10::EV_PENDING, 0xFFFF_FFFF);
        irq.irq_csr.wfo(utralib::utra::irqarray10::EV_ENABLE_IOXIRQ, 1);
        // This is an [Option; 8] instead of a Vec because in hardware we have only
        // 8 slots where we can populate interrupt mappings in the hardware. Each slot's
        // index maps 1:1 to a hardware register offset. Thus as interrupts are de-allocated,
        // we don't want the indices to change - in this case, Vec is not the right object,
        // we want a static array defined like this.
        let mut irq_table: [Option<IrqLocalRegistration>; IOX_IRQ_SLOTS] = [const { None }; IOX_IRQ_SLOTS];

        // defer these connections until required by the UpdatePerClk call. This prevents
        // race conditions on startup, as both i2c and peri rely on the loop below running
        // to complete their initializations.
        let mut i2c_conn: Option<xous::CID> = None;
        let mut peri_conn: Option<xous::CID> = None;

        #[cfg(feature = "iox-claims")]
        let mut claims = IoxClaims::new();
        // the IOX interrupt handler of this process is the only legitimate sender of
        // `IrqLocalHandler`
        #[cfg(feature = "iox-claims")]
        let own_pid = xous::process::id() as u8;

        let mut msg_opt = None;
        log::debug!("Starting main loop");
        loop {
            xous::reply_and_receive_next(sid, &mut msg_opt).unwrap();
            // this loop uses a unique form of reply_and_receive next that allows us to store and delegate
            // scalar messages if necessary (that feature is currently unused). To do this, .take() the
            // msg_opt and store it in a Vec; then use an explicit rsyscall to do a
            // return_scalar1[2,5] to unblock all the blocking threads. The current loop doesn't
            // actually do this - the feature was deemed unneeded in the end - but the structural
            // refactor is left here because I can see it being useful in the future as this
            // server does more and more hardware air traffic control.
            let opcode = {
                let msg = msg_opt.as_mut().unwrap();
                num_traits::FromPrimitive::from_usize(msg.body.id()).unwrap_or(HalOpcode::InvalidCall)
            };
            log::debug!("{:?}", opcode);
            match opcode {
                HalOpcode::MapIfram => {
                    let sender = msg_opt.as_ref().unwrap().sender;
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                        let requested_size = scalar.arg1; // requested size
                        let requested_bank = scalar.arg2; // Specifies bank 0, 1, or don't care (any number but 0 or 1)

                        let mut allocated_address = None;
                        for (bank, table) in ifram_allocs.iter_mut().enumerate() {
                            // a zero-sized request would trip the assertion in `try_alloc`
                            if requested_size == 0 {
                                break;
                            }
                            if bank == requested_bank || requested_bank > 1 {
                                match try_alloc(table, requested_size, sender) {
                                    Some(offset) => {
                                        let base = if bank == 0 {
                                            utralib::generated::HW_IFRAM0_MEM
                                        } else {
                                            utralib::generated::HW_IFRAM1_MEM
                                        };
                                        allocated_address = Some(base + offset);
                                        break;
                                    }
                                    None => {}
                                }
                            }
                        }
                        // responds with size in arg1 (0 means could not be allocated/OOM)
                        // and address of allocation in arg2
                        if let Some(addr) = allocated_address {
                            log::debug!(
                                "Allocated IFRAM at 0x{:x} to hold at least 0x{:x} bytes",
                                addr,
                                requested_size
                            );
                            log::debug!("Alloc[0]: {:x?}", ifram_allocs[0]);
                            log::debug!("Alloc[1]: {:x?}", ifram_allocs[1]);
                            scalar.arg1 = requested_size;
                            scalar.arg2 = addr;
                        } else {
                            log::debug!(
                                "Could not allocate IFRAM request of 0x{:x} bytes in bank {}",
                                requested_size,
                                requested_bank
                            );
                            scalar.arg1 = 0;
                            scalar.arg2 = 0;
                        }
                    }
                }
                HalOpcode::UnmapIfram => {
                    #[cfg(feature = "iox-claims")]
                    let sender = msg_opt.as_ref().unwrap().sender;
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message() {
                        let mapped_size = scalar.arg1;
                        let phys_addr = scalar.arg2;
                        let Some((bank, first, count)) = ifram_pages(phys_addr, mapped_size) else {
                            log::error!(
                                "UnmapIfram of 0x{:x} bytes at 0x{:x}: not a page range inside an IFRAM bank",
                                mapped_size,
                                phys_addr
                            );
                            continue;
                        };
                        let pages = &mut ifram_allocs[bank][first..first + count];
                        // only the process that was given the pages may give them back
                        #[cfg(feature = "iox-claims")]
                        if pages.iter().any(|owner| owner.and_then(|o| o.pid()) != sender.pid()) {
                            log::warn!(
                                "PID {} may not unmap IFRAM at 0x{:x}: allocated to another process",
                                sender_pid(&sender),
                                phys_addr
                            );
                            continue;
                        }
                        for record in pages.iter_mut() {
                            *record = None;
                        }
                    }
                }
                HalOpcode::ConfigureIox => {
                    #[cfg(feature = "iox-claims")]
                    let pid = sender_pid(&msg_opt.as_ref().unwrap().sender);
                    let Some(mem) = msg_opt.as_ref().unwrap().body.memory_message() else {
                        log::warn!("ConfigureIox: not a memory message");
                        continue;
                    };
                    let buf = unsafe { xous_ipc::Buffer::from_memory_message(mem) };
                    let Ok(config) = buf.to_original::<IoxConfigMessage, _>() else {
                        log::warn!("ConfigureIox: malformed configuration");
                        continue;
                    };
                    // The message is an immutable lend, so a refusal cannot be reported back;
                    // the configuration is dropped and logged.
                    if config.pin >= IOX_PINS {
                        log::warn!("ConfigureIox: no pin {} on {:?}", config.pin, config.port);
                        continue;
                    }
                    #[cfg(feature = "iox-claims")]
                    if !claims.allows(config.port, config.pin, pid) {
                        log::warn!(
                            "PID {} may not configure {:?}{}: claimed by another process",
                            pid,
                            config.port,
                            config.pin
                        );
                        continue;
                    }
                    if let Some(f) = config.function {
                        iox.set_alternate_function(config.port, config.pin, f);
                    }
                    if let Some(d) = config.direction {
                        iox.set_gpio_dir(config.port, config.pin, d);
                    }
                    if let Some(t) = config.schmitt_trigger {
                        iox.set_gpio_schmitt_trigger(config.port, config.pin, t);
                    }
                    if let Some(p) = config.pullup {
                        iox.set_gpio_pullup(config.port, config.pin, p);
                    }
                    if let Some(s) = config.slow_slew {
                        iox.set_slow_slew_rate(config.port, config.pin, s);
                    }
                    if let Some(s) = config.strength {
                        iox.set_drive_strength(config.port, config.pin, s);
                    }
                }
                HalOpcode::ConfigureIoxIrq => {
                    let pid = sender_pid(&msg_opt.as_ref().unwrap().sender);
                    let Some(mem) = msg_opt.as_mut().unwrap().body.memory_message_mut() else {
                        log::warn!("ConfigureIoxIrq: not a memory message");
                        continue;
                    };
                    let mut buf = unsafe { xous_ipc::Buffer::from_memory_message_mut(mem) };
                    let Ok(mut registration) = buf.to_original::<IoxIrqRegistration, _>() else {
                        log::warn!("ConfigureIoxIrq: malformed registration");
                        continue;
                    };
                    log::info!("Got registration request from PID {}: {:?}", pid, registration);
                    // `index` stays `None` on every refusal, which is what the caller reads back
                    registration.index = None;
                    let free_slot = find_first_none(&irq_table);
                    #[cfg(feature = "iox-claims")]
                    let claimed = !claims.allows(registration.port, registration.pin, pid);
                    #[cfg(not(feature = "iox-claims"))]
                    let claimed = false;
                    if registration.pin >= IOX_PINS {
                        log::warn!("IRQ refused: no pin {} on {:?}", registration.pin, registration.port);
                    } else if claimed {
                        log::warn!(
                            "IRQ refused: PID {} may not watch {:?}{}, claimed by another process",
                            pid,
                            registration.port,
                            registration.pin
                        );
                    } else if free_slot.is_none() {
                        log::warn!("IRQ refused: all {} Iox interrupt slots are in use", IOX_IRQ_SLOTS);
                    } else if let (Some(index), Ok(local_conn)) =
                        (free_slot, xns.request_connection(&registration.server))
                    {
                        // create the reverse-lookup registration
                        let local_reg = IrqLocalRegistration {
                            pid,
                            cid: local_conn,
                            opcode: registration.opcode,
                            port: registration.port,
                            pin: registration.pin,
                            active: registration.active,
                            server: registration.server.to_owned(),
                        };
                        irq_table[index] = Some(local_reg);

                        // now activate the hardware register
                        let select = irq_select_from_port(registration.port, registration.pin);
                        let mut int_cr = IntCr(0);
                        int_cr.set_select(select);
                        match registration.active {
                            IoxValue::Low => int_cr.set_mode(IntMode::FallingEdge as u32),
                            IoxValue::High => int_cr.set_mode(IntMode::RisingEdge as u32),
                        }
                        int_cr.set_enable(true);
                        int_cr.set_wakeup(true);
                        log::debug!("writing {:x} to interrupt slot {}", int_cr.0, index);
                        write_int_cr(&iox, index, int_cr.0);
                        registration.index = Some(index);
                    } else {
                        log::warn!("IRQ refused: no connection to server {:?}", registration.server);
                    }
                    // only the slot index goes back to the caller
                    registration.server.clear();
                    if buf.replace(registration).is_err() {
                        log::warn!("ConfigureIoxIrq: could not answer the registration");
                    }
                }
                HalOpcode::UpdateIoxIrq => {
                    #[cfg(feature = "iox-claims")]
                    let pid = sender_pid(&msg_opt.as_ref().unwrap().sender);
                    let Some(mem) = msg_opt.as_ref().unwrap().body.memory_message() else {
                        log::warn!("UpdateIoxIrq: not a memory message");
                        continue;
                    };
                    let buf = unsafe { xous_ipc::Buffer::from_memory_message(mem) };
                    let Ok(update) = buf.to_original::<IoxIrqUpdate, _>() else {
                        log::warn!("UpdateIoxIrq: malformed update");
                        continue;
                    };
                    log::debug!("Got IRQ update request: {:?}", update);
                    if let Some(Some(reg)) = irq_table.get_mut(update.index) {
                        // only the registering process may change its registration
                        #[cfg(feature = "iox-claims")]
                        if reg.pid != pid {
                            log::warn!("PID {} may not update IRQ slot {}", pid, update.index);
                            continue;
                        }
                        if reg.server == update.server {
                            if let Some(active) = update.active {
                                reg.active = active;
                                // fetch the interrupt control register value
                                let mut int_cr = IntCr(unsafe {
                                    iox.csr
                                        .base()
                                        .add(utralib::utra::iox::SFR_INTCR_CRINT0.offset())
                                        .add(update.index)
                                        .read_volatile()
                                });
                                // update the edge option
                                match active {
                                    IoxValue::Low => int_cr.set_mode(IntMode::FallingEdge as u32),
                                    IoxValue::High => int_cr.set_mode(IntMode::RisingEdge as u32),
                                }
                                // commit
                                unsafe {
                                    iox.csr
                                        .base()
                                        .add(utralib::utra::iox::SFR_INTCR_CRINT0.offset())
                                        .add(update.index)
                                        .write_volatile(int_cr.0);
                                }
                            }
                            if let Some(opcode) = update.opcode {
                                reg.opcode = opcode;
                            }
                        }
                    }
                }
                HalOpcode::IrqLocalHandler => {
                    #[cfg(feature = "iox-claims")]
                    if sender_pid(&msg_opt.as_ref().unwrap().sender) != own_pid {
                        log::warn!("IrqLocalHandler refused: sent by another process");
                        continue;
                    }
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message() {
                        // Figure out which port(s) caused the IRQ
                        let irq_flag = iox.csr.r(utralib::utra::iox::SFR_INTFR);
                        // clear the set bit by writing it back
                        iox.csr.wo(utralib::utra::iox::SFR_INTFR, irq_flag);
                        let mut found = false;
                        for bitpos in 0..8 {
                            // the bit position is flipped versus register order in memory
                            if ((irq_flag << (bitpos as u32)) & 0x80) != 0 {
                                if let Some(local_reg) = &irq_table[bitpos] {
                                    found = true;
                                    // interrupts are "Best effort" and can gracefully fail if the receiver
                                    // has been overwhelmed by too many
                                    // interrupts
                                    xous::try_send_message(
                                        local_reg.cid,
                                        xous::Message::new_scalar(local_reg.opcode, 0, 0, 0, 0),
                                    )
                                    .ok();
                                } else {
                                    log::warn!(
                                        "Got IRQ on position {} but no registration was found, ignoring!",
                                        bitpos
                                    );
                                }
                            }
                        }
                        if !found {
                            if irq_flag != 0 {
                                log::warn!(
                                    "No handler was found for raw flag: {:x} (note bit order is reversed)",
                                    irq_flag
                                );
                            } else {
                                log::info!(
                                    "Interrupt received but maybe not for us? pending {:x}",
                                    scalar.arg1
                                );
                                log::info!("CR0 {:x}", iox.csr.r(utralib::utra::iox::SFR_INTCR_CRINT0));
                                log::info!("CR1 {:x}", iox.csr.r(utralib::utra::iox::SFR_INTCR_CRINT1));
                                log::info!("CR2 {:x}", iox.csr.r(utralib::utra::iox::SFR_INTCR_CRINT2));
                                log::info!("CR3 {:x}", iox.csr.r(utralib::utra::iox::SFR_INTCR_CRINT3));
                                log::info!("CR4 {:x}", iox.csr.r(utralib::utra::iox::SFR_INTCR_CRINT4));
                                log::info!("CR5 {:x}", iox.csr.r(utralib::utra::iox::SFR_INTCR_CRINT5));
                                log::info!("CR6 {:x}", iox.csr.r(utralib::utra::iox::SFR_INTCR_CRINT6));
                                log::info!("CR7 {:x}", iox.csr.r(utralib::utra::iox::SFR_INTCR_CRINT7));
                            }
                        }
                    }
                }
                HalOpcode::SetGpioBank => {
                    #[cfg(feature = "iox-claims")]
                    let pid = sender_pid(&msg_opt.as_ref().unwrap().sender);
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message() {
                        let Some(port) = IoxPort::from_usize(scalar.arg1) else {
                            log::warn!("SetGpioBank: no port {}", scalar.arg1);
                            continue;
                        };
                        let value = scalar.arg2 as u16;
                        // pins claimed by other processes are dropped from the write; the rest go through
                        #[cfg(feature = "iox-claims")]
                        let bitmask = claims.write_mask(port, scalar.arg3 as u16, pid);
                        #[cfg(not(feature = "iox-claims"))]
                        let bitmask = scalar.arg3 as u16;
                        if bitmask != 0 {
                            iox.set_gpio_bank(port, value, bitmask);
                        }
                    }
                }
                HalOpcode::GetGpioBank => {
                    #[cfg(feature = "iox-claims")]
                    let pid = sender_pid(&msg_opt.as_ref().unwrap().sender);
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                        let Some(port) = IoxPort::from_usize(scalar.arg1) else {
                            log::warn!("GetGpioBank: no port {}", scalar.arg1);
                            scalar.arg1 = 0;
                            continue;
                        };
                        // pins claimed by other processes read as 0
                        #[cfg(feature = "iox-claims")]
                        let value = iox.get_gpio_bank(port) & claims.read_mask(port, pid);
                        #[cfg(not(feature = "iox-claims"))]
                        let value = iox.get_gpio_bank(port);
                        scalar.arg1 = value as usize;
                    }
                }
                #[cfg(feature = "iox-claims")]
                HalOpcode::ClaimIox => {
                    let pid = sender_pid(&msg_opt.as_ref().unwrap().sender);
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                        let port: Option<IoxPort> = num_traits::FromPrimitive::from_usize(scalar.arg1);
                        let pins = scalar.arg2 as u16;
                        scalar.arg1 = match port {
                            Some(port) if pid != 0 => match claims.claim(port, pins, pid) {
                                Ok(()) => {
                                    log::info!("PID {} claimed {:?} pins {:#06x}", pid, port, pins);
                                    // edge interrupts other processes registered on these pins
                                    // before the claim stop here
                                    for (index, slot) in irq_table.iter_mut().enumerate() {
                                        if let Some(reg) = slot {
                                            if !claims.allows(reg.port, reg.pin, reg.pid) {
                                                log::warn!(
                                                    "IRQ slot {} of PID {} on {:?}{} revoked by the claim",
                                                    index,
                                                    reg.pid,
                                                    reg.port,
                                                    reg.pin
                                                );
                                                write_int_cr(&iox, index, 0);
                                                *slot = None;
                                            }
                                        }
                                    }
                                    0
                                }
                                Err(ClaimError::Owned { pins }) => {
                                    log::warn!("PID {} refused {:?} pins {:#06x}: claimed", pid, port, pins);
                                    1
                                }
                            },
                            _ => 2,
                        };
                    }
                }
                #[cfg(feature = "iox-claims")]
                HalOpcode::ClaimPeriph => {
                    let pid = sender_pid(&msg_opt.as_ref().unwrap().sender);
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                        let periph: Option<PeriphId> = num_traits::FromPrimitive::from_usize(scalar.arg1);
                        scalar.arg1 = match periph {
                            Some(periph) if pid != 0 => match claims.claim_periph(periph, pid) {
                                Ok(()) => {
                                    log::info!("PID {} claimed {:?}", pid, periph);
                                    0
                                }
                                Err(PeriphClaimError::Owned) => {
                                    log::warn!("PID {} refused {:?}: claimed", pid, periph);
                                    1
                                }
                            },
                            _ => 2,
                        };
                    }
                }
                HalOpcode::ConfigureBio => {
                    // With bio-locked the image runs no BIO code and this process holds the BIO's
                    // pages so that no other process can (servers/bio.rs), so no pin may be handed
                    // to the BIO either. The single-pin form answers "no mapping"; the bitmask form
                    // is dropped.
                    #[cfg(feature = "bio-locked")]
                    {
                        log::warn!(
                            "ConfigureBio refused for PID {}: the BIO is locked",
                            sender_pid(&msg_opt.as_ref().unwrap().sender)
                        );
                        if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                            scalar.arg2 = 0;
                        }
                    }
                    #[cfg(not(feature = "bio-locked"))]
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                        if scalar.arg3 == 0 {
                            let Some(port) = IoxPort::from_usize(scalar.arg1) else {
                                log::warn!("ConfigureBio: no port {}", scalar.arg1);
                                scalar.arg2 = 0;
                                continue;
                            };
                            let pin = scalar.arg2 as u8;
                            match iox.set_bio_bit_from_port_and_pin(port, pin) {
                                Some(bit) => {
                                    scalar.arg1 = bit as usize;
                                    scalar.arg2 = 1
                                }
                                _ => scalar.arg2 = 0,
                            }
                        } else if scalar.arg4 <= bio::IoConfigMode::ClearOnly as usize {
                            let io_mode: bio::IoConfigMode = scalar.arg4.into();
                            iox.set_ports_from_bio_bitmask(scalar.arg1 as u32, io_mode);
                            log::debug!("bio_bitmask: {:x}", iox.get_bio_mapping_bitmask());
                            // no return values need to be set
                        } else {
                            log::warn!("ConfigureBio: no I/O mode {}", scalar.arg4);
                        }
                    }
                }
                HalOpcode::ConfigureUdmaClock => {
                    #[cfg(feature = "iox-claims")]
                    let pid = sender_pid(&msg_opt.as_ref().unwrap().sender);
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message() {
                        let Some(periph) = PeriphId::from_usize(scalar.arg1) else {
                            log::warn!("ConfigureUdmaClock: no peripheral {:#x}", scalar.arg1);
                            continue;
                        };
                        #[cfg(feature = "iox-claims")]
                        if !claims.periph_allows(periph, pid) {
                            log::warn!("PID {} may not gate the clock of {:?}: claimed", pid, periph);
                            continue;
                        }
                        let enable = if scalar.arg2 != 0 { true } else { false };
                        if enable {
                            udma_global.clock_on(periph);
                        } else {
                            udma_global.clock_off(periph);
                        }
                    }
                }
                HalOpcode::ConfigureUdmaEvent => {
                    #[cfg(feature = "iox-claims")]
                    let pid = sender_pid(&msg_opt.as_ref().unwrap().sender);
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message() {
                        let periph: Option<PeriphId> = num_traits::FromPrimitive::from_usize(scalar.arg1);
                        let to_channel: Option<EventChannel> =
                            num_traits::FromPrimitive::from_usize(scalar.arg3);
                        let (Some(periph), Some(to_channel)) = (periph, to_channel) else {
                            log::warn!(
                                "ConfigureUdmaEvent: no peripheral {:#x} or channel {}",
                                scalar.arg1,
                                scalar.arg3
                            );
                            continue;
                        };
                        #[cfg(feature = "iox-claims")]
                        if !claims.periph_allows(periph, pid) {
                            log::warn!("PID {} may not map the events of {:?}: claimed", pid, periph);
                            continue;
                        }
                        let event_offset = scalar.arg2 as u32;
                        // note: no "air traffic control" is done to prevent mapping other
                        // events. Maybe this should be done? but for now, let's leave it
                        // as bare iron.
                        udma_global.map_event_with_offset(periph, event_offset, to_channel);
                    }
                }
                HalOpcode::UdmaIrqStatusBits => {
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                        match num_traits::FromPrimitive::from_usize(scalar.arg1) {
                            Some(bank) => scalar.arg1 = udma_global.irq_status_bits(bank) as usize,
                            None => {
                                log::warn!("UdmaIrqStatusBits: no bank {}", scalar.arg1);
                                scalar.arg1 = 0;
                            }
                        }
                    }
                }
                HalOpcode::PeriphReset => {
                    #[cfg(feature = "iox-claims")]
                    let pid = sender_pid(&msg_opt.as_ref().unwrap().sender);
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message() {
                        let Some(periph) = PeriphId::from_usize(scalar.arg1) else {
                            log::warn!("PeriphReset: no peripheral {:#x}", scalar.arg1);
                            continue;
                        };
                        #[cfg(feature = "iox-claims")]
                        if !claims.periph_allows(periph, pid) {
                            log::warn!("PID {} may not reset {:?}: claimed", pid, periph);
                            continue;
                        }
                        udma_global.reset(periph);
                    }
                }
                HalOpcode::SetPreemptionState => {
                    // this is done as a blocking message because we want confirmation that
                    // this bit has been set before entering a critical section.
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                        timer_run = scalar.arg1 != 0;
                        os_timer.wfo(utra::timer0::EV_ENABLE_ZERO, if timer_run { 1 } else { 0 });
                    }
                }
                HalOpcode::UpdatePerclk => {
                    if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                        // lazily fill in these connections on demand. This prevents race conditions on
                        // process startup.
                        let i2c_cid = i2c_conn.get_or_insert_with(|| {
                            xns.request_connection_blocking(SERVER_NAME_BAO1X_I2C)
                                .expect("Couldn't connect to bao1x I2C server")
                        });
                        let peri_cid = peri_conn.get_or_insert_with(|| {
                            xns.request_connection_blocking(SERVER_NAME_BAO1X_OTHERS)
                                .expect("Couldn't connect to bao1x misc peri server")
                        });

                        // forward to peri block
                        xous::send_message(
                            *peri_cid,
                            xous::Message::new_blocking_scalar(
                                PeripheralOpcode::UpdatePerclk.to_usize().unwrap(),
                                scalar.arg1,
                                0,
                                0,
                                0,
                            ),
                        )
                        .ok();
                        xous::send_message(
                            *i2c_cid,
                            xous::Message::new_blocking_scalar(
                                I2cOpcode::UpdatePerclk.to_usize().unwrap(),
                                scalar.arg1,
                                0,
                                0,
                                0,
                            ),
                        )
                        .ok();
                    }
                }
                HalOpcode::InvalidCall => {
                    log::error!("Invalid opcode received: {:?}", msg_opt);
                }
            }
        }
    });

    servers::bio::start_bio_service();

    servers::keyboard::start_keyboard_service();

    // claim BIO, based on the platform
    servers::trng::start_trng_service();
    servers::rtc::start_rtc_service();

    let dummy_sid = xous::create_server().unwrap();
    // placeholder - we can put an actual service here but for now just sleep the main thread.
    loop {
        let _ = xous::receive_message(dummy_sid);
    }
}
