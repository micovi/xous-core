use std::fmt;
use std::io::{Cursor, Result, Seek, Write};
pub type XousArgumentCode = u32;
pub type XousSize = u32;
use crc::{Hasher16, crc16};

#[macro_export]
macro_rules! make_type {
    ($fcc:expr) => {{
        let mut c: [u8; 4] = Default::default();
        c.copy_from_slice($fcc.as_bytes());
        u32::from_le_bytes(c)
    }};
}

pub trait XousArgument: fmt::Display {
    /// A fourcc code of this tag
    fn code(&self) -> XousArgumentCode;

    fn name(&self) -> String {
        let tag_name_bytes = self.code().to_le_bytes();
        String::from_utf8_lossy(&tag_name_bytes).to_string()
    }

    /// The total size of this argument, not including the code and the length.
    fn length(&self) -> XousSize;

    /// Called immediately before serializing.  Returns the amount of data
    /// to reserve.
    fn finalize(&mut self, _offset: usize) -> usize { 0 }

    /// Write the contents of this argument to the specified writer.
    /// Return the number of bytes written.
    fn serialize(&self, output: &mut dyn Write) -> Result<usize>;

    /// Any last data that needs to be written.
    fn last_data(&self) -> &[u8] { &[] }

    fn alignment_offset(&self) -> usize { 0 }

    fn load_offset(&self) -> usize { 0 }
}

pub struct XousArguments {
    pub ram_start: XousSize,
    pub ram_length: XousSize,
    ram_name: u32,
    pub arguments: Vec<Box<dyn XousArgument>>,
    pub detached_offset: Option<XousSize>,
}

impl fmt::Display for XousArguments {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "Xous Arguments with {} parameters", self.arguments.len())?;

        let tag_name_bytes = self.ram_name.to_le_bytes();
        let tag_name = String::from_utf8_lossy(&tag_name_bytes);
        writeln!(
            f,
            "    Main RAM \"{}\" ({:08x}): {:08x} - {:08x}",
            tag_name,
            self.ram_name,
            self.ram_start,
            self.ram_start + self.ram_length
        )?;

        for (index, arg) in self.arguments.iter().enumerate() {
            write!(f, "{:2}{}", index + 1, arg)?;
        }
        Ok(())
    }
}

impl XousArguments {
    pub fn new(ram_start: XousSize, ram_length: XousSize, ram_name: u32) -> XousArguments {
        XousArguments { ram_start, ram_length, ram_name, arguments: vec![], detached_offset: None }
    }

    /// `offset` is where byte 0 of this argument block lands, counted from the start of the
    /// kernel's argument block. It must be page-aligned, so that the page offsets `finalize()`
    /// computes are the ones `write()` pads to.
    pub fn set_detached_offset(&mut self, offset: XousSize) {
        assert!(
            offset as usize % crate::tags::PAGE_SIZE == 0,
            "detached offset {:x} is not page-aligned",
            offset
        );
        self.detached_offset = Some(offset);
    }

    pub fn finalize(&mut self) {
        // The payloads start right after the tags, exactly where `write()` starts padding them.
        let mut running_offset = self.len() as usize + self.detached_offset.unwrap_or(0) as usize;
        // println!("offset: {:x}, alignment_offset: {:x}", running_offset, 0);
        for arg in &mut self.arguments {
            running_offset = crate::tags::align_size_up(running_offset, arg.alignment_offset());
            // println!("offset: {:x}", running_offset);
            running_offset += arg.finalize(running_offset);
        }
    }

    pub fn add<T: 'static>(&mut self, arg: T)
    where
        T: XousArgument + Sized,
    {
        self.arguments.push(Box::new(arg));
    }

    pub fn write<T>(&mut self, mut w: T) -> Result<()>
    where
        T: Write + Seek,
    {
        let total_length = self.len();

        // Finalize the arguments.  This lets any tags update their offsets
        // based on the size of the entire array.
        self.finalize();

        // XArg tag contents
        let mut tag_data = Cursor::new(Vec::new());
        tag_data.write_all(&((total_length / 4) as u32).to_le_bytes())?;
        tag_data.write_all(&1u32.to_le_bytes())?; // Version
        tag_data.write_all(&(self.ram_start as u32).to_le_bytes())?;
        tag_data.write_all(&(self.ram_length as u32).to_le_bytes())?;
        tag_data.write_all(&(self.ram_name as u32).to_le_bytes())?;

        assert!(tag_data.get_ref().len().trailing_zeros() >= 2, "tag data was not a multiple of 4 bytes!");

        let mut digest = crc16::Digest::new(crc16::X25);

        // store the header offset
        let header_offset = w.stream_position()?;

        // XArg tag header
        w.write_all(&u32::from_le_bytes(*b"XArg").to_le_bytes())?;
        digest.write(tag_data.get_ref());
        w.write_all(&digest.sum16().to_le_bytes())?; // CRC16
        w.write_all(&((tag_data.get_ref().len() / 4) as u16).to_le_bytes())?; // Size (in words)
        w.write_all(tag_data.get_ref())?;

        // Write out each subsequent argument
        for arg in &self.arguments {
            let mut tag_data = Cursor::new(Vec::new());
            let advertised_len = arg.length() as u32;
            let actual_len = arg.serialize(&mut tag_data)? as u32;
            assert_eq!(
                advertised_len,
                actual_len,
                "argument {} advertised it would write {} bytes, but it wrote {} bytes",
                arg.name(),
                advertised_len,
                actual_len
            );
            assert_eq!(
                tag_data.get_ref().len() as u32,
                actual_len,
                "argument {} said it wrote {} bytes, but it actually wrote {} bytes",
                arg.name(),
                actual_len,
                tag_data.get_ref().len()
            );

            let mut digest = crc16::Digest::new(crc16::X25);
            // XArg tag header
            w.write_all(&arg.code().to_le_bytes())?;
            digest.write(tag_data.get_ref());
            w.write_all(&digest.sum16().to_le_bytes())?; // CRC16
            w.write_all(&((tag_data.get_ref().len() / 4) as u16).to_le_bytes())?; // Size (in words)
            w.write_all(tag_data.get_ref())?;
        }

        // Write any pending data, such as payloads
        for arg in &self.arguments {
            // align for FLASH mapping
            let pos = w.stream_position()?;
            // find the next padding that allows us to align our data such that page sizes align.
            let pad_len = crate::tags::align_size_up(pos as usize, arg.alignment_offset()) - pos as usize;
            // println!("padding from {:x}, align {:x}, with {:x} bytes", pos, arg.alignment_offset(),
            // pad_len);
            let pad = vec![0u8; pad_len];
            w.write_all(&pad)?;

            // only do this check on the IniS section (swap generation). Rationale: the
            // arg.load_offset() function is only implemented for IniS. It may be trivial to
            // copy this to the other sections, but, the original idea was to make a targeted check of
            // the swap format at this point in the image creation cycle.
            if arg.code() == u32::from_le_bytes(*b"IniS") {
                // println!("header_offset: {:x}", header_offset);
                // - 0x0 is the valid offset in the case that we are being called by the encrypted partition
                //   writer, since it computes offsets from the start of the encrypted partition.
                // - 0x1000 is the expected length of the header when writing the full plaintext version for
                //   sanity checking
                assert!(
                    header_offset == 0x1000 || header_offset == 0x0,
                    "Header offset assumption for IniS was not met: loader hard-codes this size"
                );
                /*
                println!(
                    "load offset: {:x}, pos: {:x}",
                    arg.load_offset(),
                    w.stream_position()? - header_offset
                ); */
                // Debugging tips
                // If this assert fails, what's happened is that the position of the loader stream
                // is offset from where it's expected to be. There is a "just so" arrangement that isn't
                // strictly enforced: the first IniS happens to load at 0x1000; and the XArgs block
                // will generally fit within a size constraint of 0x1000. If either of these assumptions
                // break, then, this assert will trigger.
                assert!(
                    arg.load_offset() as u64 == w.stream_position()? - header_offset,
                    "IniS alignment assumption not satisfied, did XArgs overflow? \
                    Did the IniS section trigger an alignment edge case with align_size_up()?"
                );
            }
            // println!("position: {:x}", w.stream_position()?);
            w.write_all(arg.last_data()).expect("couldn't write extra arg data");
        }

        Ok(())
    }

    pub fn len(&self) -> u32 {
        let mut total_length = 20 + self.header_len() as u32; // 'XArg' plus tag length total length
        for arg in &self.arguments {
            total_length += arg.length() + 8;
        }
        total_length
    }

    pub fn is_empty(&self) -> bool { false }

    pub fn header_len(&self) -> usize { 8 }
}

#[cfg(test)]
mod tests {
    use std::convert::TryInto;
    use std::io::Cursor;

    use super::XousArguments;
    use crate::elf::{MiniElfFlags, MiniElfSection};
    use crate::tags::inif::IniF;

    /// The page offset of the first section of the ELFs the Dabao apps are built into.
    const ALIGNMENT: usize = 0x118;
    const DETACHED_OFFSET: u32 = 0x16_0000;

    /// The first word of a program's payload; the rest is zeros.
    fn marker(program: usize) -> [u8; 4] { [0xa5, 0x5a, program as u8, (program >> 8) as u8] }

    fn inif(program: usize) -> IniF {
        let mut data = marker(program).to_vec();
        data.resize(0x2345, 0);
        let section = MiniElfSection {
            virt: 0x1_0000 + ALIGNMENT as u32,
            size: data.len() as u32,
            flags: MiniElfFlags::NONE,
            name: ".rodata".to_string(),
        };
        IniF::new(0x1_0000 + ALIGNMENT as u32, vec![section], data, ALIGNMENT)
    }

    /// Writes `count` programs and returns, for each IniF tag, its load offset less
    /// `detached_offset` (the position the tag claims in this block) and the first word found
    /// there.
    fn claimed_payloads(count: usize, detached_offset: Option<u32>) -> (usize, Vec<(usize, [u8; 4])>) {
        let mut args = XousArguments::new(0, 0, 0);
        if let Some(offset) = detached_offset {
            args.set_detached_offset(offset);
        }
        for program in 0..count {
            args.add(inif(program));
        }
        let header_len = args.len() as usize;
        let mut out = Cursor::new(Vec::new());
        args.write(&mut out).unwrap();
        let image = out.into_inner();

        let word = |at: usize| u32::from_le_bytes(image[at..at + 4].try_into().unwrap()) as usize;
        let mut claimed = Vec::new();
        let mut at = 0;
        while at < header_len {
            let words = u16::from_le_bytes(image[at + 6..at + 8].try_into().unwrap()) as usize;
            if image[at..at + 4] == *b"IniF" {
                let position = word(at + 8) - detached_offset.unwrap_or(0) as usize;
                claimed.push((position, image[position..position + 4].try_into().unwrap()));
            }
            at += 8 + words * 4;
        }
        (header_len, claimed)
    }

    fn assert_payloads_where_claimed(count: usize, detached_offset: Option<u32>) {
        let (header_len, claimed) = claimed_payloads(count, detached_offset);
        assert_eq!(claimed.len(), count);
        for (program, (position, first_word)) in claimed.into_iter().enumerate() {
            assert_eq!(position % crate::tags::PAGE_SIZE, ALIGNMENT, "program {program}");
            assert_eq!(
                first_word,
                marker(program),
                "program {program} of {count}: tags of {header_len} bytes claim it at {position:#x}"
            );
        }
    }

    #[test]
    fn tags_shorter_than_the_alignment_offset() {
        assert!(claimed_payloads(2, None).0 < ALIGNMENT);
        assert_payloads_where_claimed(1, Some(DETACHED_OFFSET));
        assert_payloads_where_claimed(2, Some(DETACHED_OFFSET));
        assert_payloads_where_claimed(2, None);
    }

    /// Twelve one-section programs make 316 bytes of tags, as three Dabao apps of seven
    /// sections each do.
    #[test]
    fn tags_longer_than_the_alignment_offset() {
        assert_eq!(claimed_payloads(12, None).0, 316);
        assert_payloads_where_claimed(12, Some(DETACHED_OFFSET));
        assert_payloads_where_claimed(12, None);
    }

    #[test]
    fn tags_longer_than_a_page() {
        assert!(claimed_payloads(200, None).0 > crate::tags::PAGE_SIZE + ALIGNMENT);
        assert_payloads_where_claimed(200, Some(DETACHED_OFFSET));
        assert_payloads_where_claimed(200, None);
    }

    #[test]
    #[should_panic(expected = "not page-aligned")]
    fn an_unaligned_detached_offset_is_refused() {
        XousArguments::new(0, 0, 0).set_detached_offset(DETACHED_OFFSET - 0x100);
    }
}
