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
