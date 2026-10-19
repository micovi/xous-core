#[cfg(all(feature = "board-baosec", not(feature = "oem-baosec-lite")))]
use bao1x_api::bio::IoConfigMode;
#[cfg(any(feature = "board-dabao", feature = "oem-baosec-lite"))]
use bao1x_hal_service::trng::Trng;
use bao1x_hal_service::trng::api;
use flatipc::Ipc;
use num_traits::*;
#[cfg(all(feature = "board-baosec", not(feature = "oem-baosec-lite")))]
use rand::RngCore;
use xous::CID;
use xous_ipc::Buffer;

/// TRNG error subscribers kept. Any process may subscribe, and each costs a connection of this
/// process (the kernel gives a process 32), so the list is bounded.
const MAX_ERROR_SUBSCRIBERS: usize = 4;

#[derive(Copy, Clone, Debug)]
#[allow(dead_code)]
struct ScalarCallback {
    server_to_cb_cid: CID,
    cb_to_client_cid: CID,
    cb_to_client_id: u32,
}

#[cfg(all(feature = "board-baosec", not(feature = "oem-baosec-lite")))]
pub fn start_trng_service() {
    std::thread::spawn({
        move || {
            trng_service();
        }
    });
}
#[cfg(all(feature = "board-baosec", not(feature = "oem-baosec-lite")))]
fn trng_service() -> ! {
    let xns = xous_names::XousNames::new().unwrap();
    // unlimited connections allowed, anyone including less-trusted processes can get a random number
    let trng_sid = xns.register_name(api::SERVER_NAME_TRNG, None).expect("can't register server");

    let iox = bao1x_api::IoxHal::new();
    let power = bao1x_hal::board::setup_trng_power_pin(&iox);
    let data = bao1x_hal::board::setup_trng_input_pin(&iox);
    let av_trng = bio_lib::av_trng::AvTrng::new(data, power, Some(IoConfigMode::Overwrite)).unwrap();
    let mut trng = crate::servers::baosec_hw::HwTrng::new(av_trng);

    let mut error_cb_conns = Vec::<ScalarCallback>::new();

    loop {
        let mut msg = xous::receive_message(trng_sid).unwrap();
        let op: Option<api::Opcode> = FromPrimitive::from_usize(msg.body.id());
        log::debug!("{:?}", op);
        match op {
            Some(api::Opcode::GetTrng) => xous::msg_blocking_scalar_unpack!(msg, count, _, _, _, {
                let first = trng.next_u32() as usize;
                let second = if count == 1 { 0xdead_beef } else { trng.next_u32() as usize };
                answer_words(msg.sender, first, second);
            }),
            Some(api::Opcode::ErrorSubscribe) => {
                let Some(mem) = msg.body.memory_message() else {
                    log::warn!("ErrorSubscribe: not a memory message");
                    continue;
                };
                let buffer = unsafe { Buffer::from_memory_message(mem) };
                match buffer.to_original::<api::ScalarHook, _>() {
                    Ok(hookdata) => do_hook(hookdata, &mut error_cb_conns),
                    Err(_) => log::warn!("ErrorSubscribe: malformed hook"),
                }
            }
            // HwTrng keeps no health or error statistics yet (`get_tests` and `get_errors` are
            // `todo!()`), and the notification is sent by no interrupt handler
            Some(api::Opcode::ErrorNotification)
            | Some(api::Opcode::HealthStats)
            | Some(api::Opcode::ErrorStats) => {
                log::warn!("{:?} is not available on this board", op);
            }
            Some(api::Opcode::FillTrng) => {
                let Some(mm) = msg.body.memory_message_mut() else {
                    log::warn!("FillTrng: not a memory message");
                    continue;
                };
                let Some(buffer) = api::IpcTrngBuf::from_memory_message_mut(mm) else {
                    log::warn!("FillTrng: malformed buffer");
                    continue;
                };
                let len = (buffer.len as usize).min(buffer.data.len());
                if let Err(e) = trng.fill_buf(&mut buffer.data[..len]) {
                    log::warn!("FillTrng: {:?}", e);
                }
            }
            None => {
                log::error!("couldn't convert opcode, ignoring");
            }
        }
    }
}

#[cfg(any(feature = "board-dabao", feature = "oem-baosec-lite"))]
pub fn start_trng_service() {
    std::thread::spawn({
        move || {
            trng_service();
        }
    });
}
#[cfg(any(feature = "board-dabao", feature = "oem-baosec-lite"))]
fn trng_service() -> ! {
    let xns = xous_names::XousNames::new().unwrap();
    // unlimited connections allowed, anyone including less-trusted processes can get a random number
    let trng_sid = xns.register_name(api::SERVER_NAME_TRNG, None).expect("can't register server");

    let trng = Trng::new(&xns).unwrap();

    let mut error_cb_conns = Vec::<ScalarCallback>::new();

    loop {
        let mut msg = xous::receive_message(trng_sid).unwrap();
        let op: Option<api::Opcode> = FromPrimitive::from_usize(msg.body.id());
        log::debug!("{:?}", op);
        match op {
            Some(api::Opcode::GetTrng) => xous::msg_blocking_scalar_unpack!(msg, count, _, _, _, {
                // `Trng::get_u32` never fails: it draws from a ChaCha generator
                let first = trng.get_u32().unwrap_or_default() as usize;
                let second =
                    if count == 1 { 0xdead_beef } else { trng.get_u32().unwrap_or_default() as usize };
                answer_words(msg.sender, first, second);
            }),
            Some(api::Opcode::ErrorSubscribe) => {
                let Some(mem) = msg.body.memory_message() else {
                    log::warn!("ErrorSubscribe: not a memory message");
                    continue;
                };
                let buffer = unsafe { Buffer::from_memory_message(mem) };
                match buffer.to_original::<api::ScalarHook, _>() {
                    Ok(hookdata) => do_hook(hookdata, &mut error_cb_conns),
                    Err(_) => log::warn!("ErrorSubscribe: malformed hook"),
                }
            }
            // the kernel TRNG keeps no health or error statistics to report
            Some(api::Opcode::ErrorNotification)
            | Some(api::Opcode::HealthStats)
            | Some(api::Opcode::ErrorStats) => {
                log::warn!("{:?} is not available on this board", op);
            }
            Some(api::Opcode::FillTrng) => {
                let Some(mm) = msg.body.memory_message_mut() else {
                    log::warn!("FillTrng: not a memory message");
                    continue;
                };
                let Some(buffer) = api::IpcTrngBuf::from_memory_message_mut(mm) else {
                    log::warn!("FillTrng: malformed buffer");
                    continue;
                };
                let len = (buffer.len as usize).min(buffer.data.len());
                if let Err(e) = trng.fill_buf(&mut buffer.data[..len]) {
                    log::warn!("FillTrng: {:?}", e);
                }
            }
            None => {
                log::error!("couldn't convert opcode, ignoring");
            }
        }
    }
}

/// Answers a `GetTrng` request. The caller may have ended since it sent the message, so a failure
/// is logged, not fatal.
fn answer_words(sender: xous::MessageSender, first: usize, second: usize) {
    if let Err(e) = xous::return_scalar2(sender, first, second) {
        log::warn!("GetTrng: could not answer PID {:?}: {:?}", sender.pid(), e);
    }
}

fn do_hook(hookdata: api::ScalarHook, cb_conns: &mut Vec<ScalarCallback>) {
    if cb_conns.len() >= MAX_ERROR_SUBSCRIBERS {
        log::warn!("TRNG error hook refused: already {} subscribers", cb_conns.len());
        return;
    }
    let (s0, s1, s2, s3) = hookdata.sid;
    let sid = xous::SID::from_u32(s0, s1, s2, s3);
    // `connect` would wait for as long as no server has this SID
    let Ok(server_to_cb_cid) = xous::try_connect(sid) else {
        log::warn!("TRNG error hook refused: no server to connect to");
        return;
    };
    let cb_dat =
        ScalarCallback { server_to_cb_cid, cb_to_client_cid: hookdata.cid, cb_to_client_id: hookdata.id };
    cb_conns.push(cb_dat);
}
#[allow(dead_code)]
/// called if the server exits, which we don't. Keep around for reference in case for some reason we need to
/// do that...
fn unhook(cb_conns: &mut Vec<ScalarCallback>) {
    for scb in cb_conns.drain(..) {
        xous::send_message(
            scb.server_to_cb_cid,
            xous::Message::new_blocking_scalar(api::EventCallback::Drop.to_usize().unwrap(), 0, 0, 0, 0),
        )
        .unwrap();
        unsafe {
            xous::disconnect(scb.server_to_cb_cid).unwrap();
        }
    }
}
#[allow(dead_code)]
fn send_event(cb_conns: &Vec<ScalarCallback>) {
    for scb in cb_conns.iter() {
        // note that the "which" argument is only used for GPIO events, to indicate which pin had the
        // event
        xous::send_message(
            scb.server_to_cb_cid,
            xous::Message::new_scalar(
                api::EventCallback::Event.to_usize().unwrap(),
                scb.cb_to_client_cid as usize,
                scb.cb_to_client_id as usize,
                0,
                0,
            ),
        )
        .unwrap();
    }
}
