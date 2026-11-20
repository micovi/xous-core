use bao1x_api::*;
use bao1x_hal::udma::GlobalConfig;

pub fn start_i2c_service() {
    let _ = std::thread::spawn({
        move || {
            i2c_service();
        }
    });
}

fn i2c_service() -> ! {
    let xns = xous_names::XousNames::new().unwrap();
    let sid = xns.register_name(bao1x_api::SERVER_NAME_BAO1X_I2C, None).expect("can't register server");

    let iox = IoxHal::new();
    let udma_global = GlobalConfig::new();

    // Note: the I2C handler can be put into a separate thread if we need the main
    // HAL server to not block while a large I2C transaction is being handled. For
    // now this is all placed into a single thread. However, if we ever had a situation
    // where, for example, you had to do a compound I2C transaction and flip a GPIO pin
    // in the middle of that transaction in order for the set of I2C transactions to
    // complete, this implementation would deadlock as it would block on the I2C transaction
    // before handling the GPIO request.
    let i2c_channel = bao1x_hal::board::setup_i2c_pins(&iox);
    let periph = PeriphId::from(i2c_channel);
    udma_global.clock_on(periph);
    let i2c_pages = xous::syscall::map_memory(
        xous::MemoryAddress::new(bao1x_hal::board::I2C_IFRAM_ADDR),
        None,
        4096,
        xous::MemoryFlags::R | xous::MemoryFlags::W,
    )
    .expect("couldn't claim I2C IFRAM page");

    let i2c_ifram = unsafe {
        bao1x_hal::ifram::IframRange::from_raw_parts(
            bao1x_hal::board::I2C_IFRAM_ADDR,
            i2c_pages.as_ptr() as usize,
            i2c_pages.len(),
        )
    };
    let mut i2c = unsafe {
        bao1x_hal::udma::I2cDriver::new_with_ifram(
            i2c_channel,
            400_000,
            bao1x_api::PERCLK,
            i2c_ifram,
            &udma_global,
        )
    };
    // Once a process has claimed the channel (`HalOpcode::ClaimPeriph`), its transactions
    // and clock changes are refused to every other process: what the bus carries (a touch
    // controller's reports, say) is the owner's alone.
    #[cfg(feature = "iox-claims")]
    let own_pid = xous::process::id() as u8;
    #[cfg(feature = "iox-claims")]
    let mut refused: u32 = 0;
    let mut msg_opt = None;
    log::debug!("Starting main loop");
    loop {
        if !crate::servers::next_message(sid, &mut msg_opt, &mut 0, "I2C") {
            continue;
        }
        let opcode = {
            let msg = msg_opt.as_mut().unwrap();
            num_traits::FromPrimitive::from_usize(msg.body.id()).unwrap_or(I2cOpcode::InvalidCall)
        };

        match opcode {
            I2cOpcode::Transaction =>
            // there are no opcode types - this handles exactly one type of message, all others are ignored
            {
                #[cfg(feature = "iox-claims")]
                let pid = crate::sender_pid(&msg_opt.as_ref().unwrap().sender);
                if let Some(msg) = msg_opt.as_mut().unwrap().body.memory_message_mut() {
                    if !crate::servers::lent_length_fits(msg, "I2C") {
                        continue;
                    }
                    let mut buf = unsafe { xous_ipc::Buffer::from_memory_message_mut(msg) };
                    let Ok(mut list) = buf.to_original::<I2cTransactions, _>() else {
                        log::warn!("I2C: malformed transaction list");
                        continue;
                    };
                    // A refused list goes back as it came, each transaction marked
                    // `InternalError`: nothing is put on the bus and nothing read from it.
                    #[cfg(feature = "iox-claims")]
                    if !crate::servers::PERIPH_OWNERS.allows(periph, pid) {
                        refused = refused.saturating_add(1);
                        if refused.is_power_of_two() {
                            log::warn!(
                                "I2C: transaction of PID {} refused: {:?} is claimed by PID {} ({} so far)",
                                pid,
                                periph,
                                crate::servers::PERIPH_OWNERS.owner(periph),
                                refused
                            );
                        }
                        for transaction in list.transactions.iter_mut() {
                            transaction.result = I2cResult::InternalError;
                        }
                        if buf.replace(list).is_err() {
                            log::warn!("I2C: could not answer a refused transaction list");
                        }
                        continue;
                    }
                    for transaction in list.transactions.iter_mut() {
                        // the driver refuses a transfer longer than MAX_I2C_TRANSFER bytes
                        let outcome = match transaction.i2c_type {
                            I2cTransactionType::Write => {
                                i2c.i2c_write(transaction.device, transaction.address, &transaction.data)
                            }
                            I2cTransactionType::Read | I2cTransactionType::ReadRepeatedStart => i2c.i2c_read(
                                transaction.device,
                                transaction.address,
                                &mut transaction.data,
                                transaction.i2c_type == I2cTransactionType::ReadRepeatedStart,
                            ),
                        };
                        transaction.result = outcome.unwrap_or_else(|e| {
                            log::warn!(
                                "I2C: {} bytes for device {:#x} refused (at most {}): {:?}",
                                transaction.data.len(),
                                transaction.device,
                                bao1x_hal::udma::MAX_I2C_TRANSFER,
                                e
                            );
                            I2cResult::Nack
                        });
                    }
                    if buf.replace(list).is_err() {
                        log::warn!("I2C: could not answer the transaction list");
                    }
                }
            }
            I2cOpcode::UpdatePerclk => {
                #[cfg(feature = "iox-claims")]
                let sender = crate::sender_pid(&msg_opt.as_ref().unwrap().sender);
                if let Some(scalar) = msg_opt.as_mut().unwrap().body.scalar_message_mut() {
                    // the HAL's main loop forwards `HalOpcode::UpdatePerclk` with the PID of
                    // the process that sent it
                    #[cfg(feature = "iox-claims")]
                    let requester = if sender == own_pid { scalar.arg2 as u8 } else { sender };
                    #[cfg(feature = "iox-claims")]
                    if !crate::servers::PERIPH_OWNERS.allows(periph, requester) {
                        log::warn!("PID {} may not change the clock of {:?}: claimed", requester, periph);
                        continue;
                    }
                    let new_perclk = scalar.arg1 as u32;
                    i2c.update_perclk(new_perclk);
                }
            }
            I2cOpcode::InvalidCall => {
                log::error!("Unrecognized opcode in I2C: {:?}", opcode);
            }
        }
    }
}
