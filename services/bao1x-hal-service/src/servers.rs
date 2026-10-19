pub mod keyboard;

// TRNG server is only needed for baosec; dabao can cheat and use the kernel TRNG port
#[cfg(all(feature = "board-baosec", not(feature = "oem-baosec-lite")))]
mod baosec_hw;
pub mod bio;
pub mod i2c;
pub mod others;
pub mod rtc;
pub mod susres;
pub mod trng;

/// Whether the archive length a client gives with its lent buffer (the message offset) lies
/// within the buffer; logged when not. `xous_ipc::Buffer` slices the buffer by it unchecked and
/// panics past the end. The archive itself is not validated (rkyv is built without `bytecheck`),
/// so its relative pointers, lengths and enum tags are still trusted.
pub fn lent_length_fits(mem: &xous::MemoryMessage, what: &str) -> bool {
    let length = mem.offset.map_or(0, |offset| offset.get());
    if length > mem.buf.len() {
        log::warn!("{}: archive of {} bytes in a buffer of {}", what, length, mem.buf.len());
        return false;
    }
    true
}

/// Answers the message in `msg_opt`, if there is one, and waits for the next one. `false`, logged,
/// when the kernel refused, for example because the caller of the answered message has ended;
/// `msg_opt` is then empty, and the next call only waits.
pub fn next_message(
    sid: xous::SID,
    msg_opt: &mut Option<xous::MessageEnvelope>,
    return_type: &mut usize,
    server: &str,
) -> bool {
    match xous::reply_and_receive_next_legacy(sid, msg_opt, return_type) {
        Ok(()) => true,
        Err(e) => {
            log::warn!("{}: answer or receive failed: {:?}", server, e);
            false
        }
    }
}
