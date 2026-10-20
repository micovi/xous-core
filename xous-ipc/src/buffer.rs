use core::mem::MaybeUninit;
use std::marker::PhantomData;

use rkyv::{
    Archive, Deserialize, Place, Portable, Serialize,
    api::low::LowSerializer,
    rancor::{Failure, Strategy},
    ser::{Positional, allocator::SubAllocator, writer::Buffer as RkyvBuffer},
    with::{ArchiveWith, Identity, SerializeWith},
};
#[cfg(feature = "checked-access")]
use rkyv::{api::high::HighValidator, bytecheck::CheckBytes};
use xous::{
    CID, Error, MemoryAddress, MemoryFlags, MemoryMessage, MemoryRange, MemorySize, Message, Result,
    map_memory, send_message, try_send_message, unmap_memory,
};

#[derive(Debug)]
pub struct Buffer<'buf> {
    pages: MemoryRange,
    used: usize,
    slice: &'buf mut [u8],
    should_drop: bool,
    memory_message: Option<&'buf mut MemoryMessage>,
}
const PAGE_SIZE: usize = 0x1000;

type Serializer<'a, 'b> = LowSerializer<RkyvBuffer<'b>, SubAllocator<'a>, Failure>;

impl<'buf> Buffer<'buf> {
    #[allow(dead_code)]
    pub fn new(len: usize) -> Self {
        let flags = MemoryFlags::R | MemoryFlags::W;
        let len_to_page = (len + (PAGE_SIZE - 1)) & !(PAGE_SIZE - 1);

        // Allocate enough memory to hold the requested data
        let new_mem = map_memory(None, None, len_to_page, flags).expect("xous-ipc: OOM in buffer allocation");

        Buffer {
            pages: new_mem,
            slice: unsafe { core::slice::from_raw_parts_mut(new_mem.as_mut_ptr(), len_to_page) },
            used: 0,
            should_drop: true,
            memory_message: None,
        }
    }

    /// use a volatile write to ensure a clear operation is not optimized out
    /// for ensuring that a buffer is cleared, e.g. at the exit of a function
    pub fn volatile_clear(&mut self) {
        let b = self.slice.as_mut_ptr();
        for i in 0..self.slice.len() {
            unsafe {
                b.add(i).write_volatile(core::mem::zeroed());
            }
        }
        // Ensure the compiler doesn't re-order the clear.
        // We use `SeqCst`, because `Acquire` only prevents later accesses from being reordered before
        // *reads*, but this method only *writes* to the locations.
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }

    /// use to serialize a buffer between process-local threads. mainly for spawning new threads with more
    /// complex argument structures.
    #[allow(dead_code)]
    pub unsafe fn to_raw_parts(&self) -> (usize, usize, usize) {
        (self.pages.as_ptr() as usize, self.pages.len(), self.used)
    }

    /// use to serialize a buffer between process-local threads. mainly for spawning new threads with more
    /// complex argument structures.
    #[allow(dead_code)]
    pub unsafe fn from_raw_parts(address: usize, len: usize, offset: usize) -> Self {
        let mem = MemoryRange::new(address, len).expect("invalid memory range args");
        Buffer {
            pages: mem,
            slice: core::slice::from_raw_parts_mut(mem.as_mut_ptr(), mem.len()),
            used: offset,
            should_drop: false,
            memory_message: None,
        }
    }

    /// Consume the buffer and return the underlying storage. Used for situations where we just want to
    /// serialize into a buffer and then do something manually with the serialized data.
    ///
    /// Fails if the buffer was converted from a memory message -- the Drop semantics
    /// of the memory message would cause problems with this conversion.
    pub fn into_inner(mut self) -> core::result::Result<(MemoryRange, usize), Error> {
        if self.memory_message.is_none() {
            self.should_drop = false;
            Ok((self.pages, self.used))
        } else {
            Err(Error::ShareViolation)
        }
    }

    /// Inverse of into_inner(). Used to re-cycle pages back into a Buffer so we don't have
    /// to re-allocate data. Only safe if the `pages` matches the criteria for mapped memory
    /// pages in Xous: page-aligned, with lengths that are a multiple of a whole page size.
    pub unsafe fn from_inner(pages: MemoryRange, used: usize) -> Self {
        Buffer {
            pages,
            slice: core::slice::from_raw_parts_mut(pages.as_mut_ptr(), pages.len()),
            used,
            should_drop: false,
            memory_message: None,
        }
    }

    #[allow(dead_code)]
    pub unsafe fn from_memory_message(mem: &'buf MemoryMessage) -> Self {
        Buffer {
            pages: mem.buf,
            slice: core::slice::from_raw_parts_mut(mem.buf.as_mut_ptr(), mem.buf.len()),
            used: mem.offset.map_or(0, |v| v.get()),
            should_drop: false,
            memory_message: None,
        }
    }

    #[allow(dead_code)]
    pub unsafe fn from_memory_message_mut(mem: &'buf mut MemoryMessage) -> Self {
        Buffer {
            pages: mem.buf,
            slice: core::slice::from_raw_parts_mut(mem.buf.as_mut_ptr(), mem.buf.len()),
            used: mem.offset.map_or(0, |v| v.get()),
            should_drop: false,
            memory_message: Some(mem),
        }
    }

    /// Perform a mutable lend of this Buffer to the server.
    #[allow(dead_code)]
    pub fn lend_mut(&mut self, connection: CID, id: u32) -> core::result::Result<Result, Error> {
        let msg = MemoryMessage {
            id: id as usize,
            buf: self.pages,
            offset: MemoryAddress::new(self.used),
            valid: MemorySize::new(self.pages.len()),
        };

        // Update the offset pointer if the server modified it.
        let result = send_message(connection, Message::MutableBorrow(msg));
        if let Ok(Result::MemoryReturned(offset, _valid)) = result {
            self.used = offset.map_or(0, |v| v.get());
        }

        result
    }

    #[allow(dead_code)]
    pub fn lend(&self, connection: CID, id: u32) -> core::result::Result<Result, Error> {
        let msg = MemoryMessage {
            id: id as usize,
            buf: self.pages,
            offset: MemoryAddress::new(self.used),
            valid: MemorySize::new(self.pages.len()),
        };
        send_message(connection, Message::Borrow(msg))
    }

    #[allow(dead_code)]
    pub fn send(mut self, connection: CID, id: u32) -> core::result::Result<Result, Error> {
        let msg = MemoryMessage {
            id: id as usize,
            buf: self.pages,
            offset: MemoryAddress::new(self.used),
            valid: MemorySize::new(self.pages.len()),
        };
        let result = send_message(connection, Message::Move(msg))?;

        // prevents it from being Dropped.
        self.should_drop = false;
        Ok(result)
    }

    #[allow(dead_code)]
    pub fn try_send(mut self, connection: CID, id: u32) -> core::result::Result<Result, Error> {
        let msg = MemoryMessage {
            id: id as usize,
            buf: self.pages,
            offset: MemoryAddress::new(self.used),
            valid: MemorySize::new(self.pages.len()),
        };
        let result = try_send_message(connection, Message::Move(msg))?;

        // prevents it from being Dropped.
        self.should_drop = false;
        Ok(result)
    }

    fn into_buf_inner<F, T>(src: &T) -> core::result::Result<Self, ()>
    where
        F: for<'a, 'b> SerializeWith<T, Serializer<'a, 'b>>,
    {
        struct Wrap<'a, F, T>(&'a T, PhantomData<F>);

        impl<F, T> Archive for Wrap<'_, F, T>
        where
            F: ArchiveWith<T>,
        {
            type Archived = <F as ArchiveWith<T>>::Archived;
            type Resolver = <F as ArchiveWith<T>>::Resolver;

            fn resolve(&self, resolver: Self::Resolver, out: Place<Self::Archived>) {
                F::resolve_with(self.0, resolver, out)
            }
        }

        impl<'a, 'b, F, T> Serialize<Serializer<'a, 'b>> for Wrap<'_, F, T>
        where
            F: SerializeWith<T, Serializer<'a, 'b>>,
        {
            fn serialize(
                &self,
                serializer: &mut Serializer<'a, 'b>,
            ) -> core::result::Result<Self::Resolver, Failure> {
                F::serialize_with(self.0, serializer)
            }
        }
        let mut xous_buf = Self::new(core::mem::size_of::<T>());
        let mut scratch = [MaybeUninit::<u8>::uninit(); 256];

        let wrap = Wrap(src, PhantomData::<F>);
        let writer = RkyvBuffer::from(&mut xous_buf.slice[..]);
        let alloc = SubAllocator::new(&mut scratch);

        let serialized_buf =
            rkyv::api::low::to_bytes_in_with_alloc::<_, _, Failure>(&wrap, writer, alloc).map_err(|_| ())?;
        xous_buf.used = serialized_buf.pos();
        Ok(xous_buf)
    }

    #[allow(dead_code)]
    pub fn into_buf<T>(src: T) -> core::result::Result<Self, ()>
    where
        T: for<'b, 'a> rkyv::Serialize<
                rkyv::rancor::Strategy<
                    rkyv::ser::Serializer<
                        rkyv::ser::writer::Buffer<'b>,
                        rkyv::ser::allocator::SubAllocator<'a>,
                        (),
                    >,
                    rkyv::rancor::Failure,
                >,
            >,
    {
        Buffer::into_buf_inner::<Identity, T>(&src)
    }

    fn replace_inner<F, T>(&mut self, src: T) -> core::result::Result<(), &'static str>
    where
        F: for<'a, 'b> SerializeWith<T, Serializer<'a, 'b>>,
    {
        struct Wrap<'a, F, T>(&'a T, PhantomData<F>);

        impl<F, T> Archive for Wrap<'_, F, T>
        where
            F: ArchiveWith<T>,
        {
            type Archived = <F as ArchiveWith<T>>::Archived;
            type Resolver = <F as ArchiveWith<T>>::Resolver;

            fn resolve(&self, resolver: Self::Resolver, out: Place<Self::Archived>) {
                F::resolve_with(self.0, resolver, out)
            }
        }

        impl<'a, 'b, F, T> Serialize<Serializer<'a, 'b>> for Wrap<'_, F, T>
        where
            F: SerializeWith<T, Serializer<'a, 'b>>,
        {
            fn serialize(
                &self,
                serializer: &mut Serializer<'a, 'b>,
            ) -> core::result::Result<Self::Resolver, Failure> {
                F::serialize_with(self.0, serializer)
            }
        }

        let mut scratch = [MaybeUninit::<u8>::uninit(); 256];

        let wrap = Wrap(&src, PhantomData::<F>);
        let writer = RkyvBuffer::from(&mut self.slice[..]);
        let alloc = SubAllocator::new(&mut scratch);

        let serialized_buf =
            rkyv::api::low::to_bytes_in_with_alloc::<_, _, Failure>(&wrap, writer, alloc).unwrap();
        self.used = serialized_buf.pos();

        if let Some(ref mut msg) = self.memory_message.as_mut() {
            msg.offset = MemoryAddress::new(self.used);
        }
        Ok(())
    }

    #[allow(dead_code)]
    pub fn replace<T>(&mut self, src: T) -> core::result::Result<(), &'static str>
    where
        T: for<'b, 'a> rkyv::Serialize<
                rkyv::rancor::Strategy<
                    rkyv::ser::Serializer<
                        rkyv::ser::writer::Buffer<'b>,
                        rkyv::ser::allocator::SubAllocator<'a>,
                        (),
                    >,
                    rkyv::rancor::Failure,
                >,
            >,
    {
        self.replace_inner::<Identity, T>(src)
    }

    /// Zero-copy representation of the data on the receiving side, wrapped in an "Archived" trait and left in
    /// the heap. Cheap so uses "as_" prefix.
    #[cfg(not(feature = "checked-access"))]
    #[allow(dead_code)]
    pub fn as_flat<T, U>(&self) -> core::result::Result<&U, ()>
    where
        T: rkyv::Archive<Archived = U>,
        U: Portable,
    {
        let r = unsafe { rkyv::access_unchecked::<U>(&self.slice[..self.used]) };
        Ok(r)
    }

    /// A representation identical to the original, but requires copying to the stack. More expensive so uses
    /// "to_" prefix.
    #[cfg(not(feature = "checked-access"))]
    #[allow(dead_code)]
    pub fn to_original<T, U>(&self) -> core::result::Result<T, Error>
    where
        T: rkyv::Archive<Archived = U>,
        U: Portable,
        <T as Archive>::Archived: Deserialize<T, Strategy<rkyv::de::Pool, rkyv::rancor::Error>>,
    {
        let r = unsafe { rkyv::access_unchecked::<U>(&self.slice[..self.used]) };
        rkyv::deserialize::<T, rkyv::rancor::Error>(r).map_err(|_| Error::InternalError)
    }

    /// Zero-copy representation of the data on the receiving side, wrapped in an "Archived" trait and left in
    /// the heap. Cheap so uses "as_" prefix.
    ///
    /// The archive is validated first: its length, relative pointers, lengths and enum tags must lie
    /// within the buffer and be valid for `U`, so a malformed buffer from another process is refused
    /// with `Err(())` instead of being read.
    #[cfg(feature = "checked-access")]
    #[allow(dead_code)]
    pub fn as_flat<T, U>(&self) -> core::result::Result<&U, ()>
    where
        T: rkyv::Archive<Archived = U>,
        U: Portable + for<'a> CheckBytes<HighValidator<'a, Failure>>,
    {
        self.checked_archive::<U>().ok_or(())
    }

    /// A representation identical to the original, but requires copying to the stack. More expensive so uses
    /// "to_" prefix.
    ///
    /// The archive is validated first, as in `as_flat`; a malformed buffer is refused with
    /// `Error::InternalError`.
    #[cfg(feature = "checked-access")]
    #[allow(dead_code)]
    pub fn to_original<T, U>(&self) -> core::result::Result<T, Error>
    where
        T: rkyv::Archive<Archived = U>,
        U: Portable + for<'a> CheckBytes<HighValidator<'a, Failure>>,
        <T as Archive>::Archived: Deserialize<T, Strategy<rkyv::de::Pool, rkyv::rancor::Error>>,
    {
        let r = self.checked_archive::<U>().ok_or(Error::InternalError)?;
        rkyv::deserialize::<T, rkyv::rancor::Error>(r).map_err(|_| Error::InternalError)
    }

    /// The archive in the first `used` bytes, if `used` fits the buffer and the archive validates.
    #[cfg(feature = "checked-access")]
    fn checked_archive<U>(&self) -> Option<&U>
    where
        U: Portable + for<'a> CheckBytes<HighValidator<'a, Failure>>,
    {
        let archive = self.slice.get(..self.used)?;
        rkyv::access::<U, Failure>(archive).ok()
    }

    pub fn used(&self) -> usize { self.used }
}

impl<'a> core::convert::AsRef<[u8]> for Buffer<'a> {
    fn as_ref(&self) -> &[u8] { &self.slice[..self.used] }
}

impl<'a> core::convert::AsMut<[u8]> for Buffer<'a> {
    fn as_mut(&mut self) -> &mut [u8] { &mut self.slice[..self.used] }
}

impl<'a> core::ops::Deref for Buffer<'a> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target { &*(&self.slice[..self.used]) }
}

impl<'a> core::ops::DerefMut for Buffer<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target { &mut *(&mut self.slice[..self.used]) }
}

impl<'a> Drop for Buffer<'a> {
    fn drop(&mut self) {
        if self.should_drop {
            unmap_memory(self.pages).expect("Buffer: failed to drop memory");
        }
    }
}

#[cfg(all(test, feature = "checked-access"))]
mod checked_access_tests {
    use std::alloc::{Layout, alloc_zeroed, dealloc};

    use super::*;

    #[derive(Debug, PartialEq, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
    enum Kind {
        Read,
        Write,
        Both,
    }

    #[derive(Debug, PartialEq, rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)]
    struct Request {
        kind: Kind,
        data: Vec<u8>,
    }

    /// One page of page-aligned memory standing in for a lent buffer.
    struct Page(*mut u8);

    impl Page {
        fn layout() -> Layout { Layout::from_size_align(PAGE_SIZE, PAGE_SIZE).unwrap() }

        fn new() -> Self {
            let ptr = unsafe { alloc_zeroed(Self::layout()) };
            assert!(!ptr.is_null());
            Page(ptr)
        }

        /// A buffer over this page holding a valid archive of `request`.
        fn buffer_with(&self, request: Request) -> Buffer<'static> {
            let range = unsafe { MemoryRange::new(self.0 as usize, PAGE_SIZE) }.unwrap();
            let mut buffer = unsafe { Buffer::from_inner(range, 0) };
            buffer.replace(request).unwrap();
            buffer
        }
    }

    impl Drop for Page {
        fn drop(&mut self) { unsafe { dealloc(self.0, Self::layout()) } }
    }

    fn request() -> Request { Request { kind: Kind::Write, data: vec![1, 2, 3, 4] } }

    /// Offset of the archived root, which rkyv places at the end of the archive.
    fn root(buffer: &Buffer) -> usize { buffer.used() - core::mem::size_of::<ArchivedRequest>() }

    fn write_u32(buffer: &mut Buffer, offset: usize, value: u32) {
        buffer[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn refused(buffer: &Buffer) -> bool {
        buffer.as_flat::<Request, _>().is_err()
            && buffer.to_original::<Request, _>() == Err(Error::InternalError)
    }

    #[test]
    fn valid_archive_is_accepted() {
        let page = Page::new();
        let buffer = page.buffer_with(request());
        let flat = buffer.as_flat::<Request, _>().unwrap();
        assert_eq!(flat.data.as_slice(), &[1, 2, 3, 4]);
        assert_eq!(buffer.to_original::<Request, _>().unwrap(), request());
    }

    #[test]
    fn vec_length_past_the_end_is_refused() {
        let page = Page::new();
        let mut buffer = page.buffer_with(request());
        let len = root(&buffer) + core::mem::offset_of!(ArchivedRequest, data) + 4;
        write_u32(&mut buffer, len, 0x7fff_ffff);
        assert!(refused(&buffer));
    }

    #[test]
    fn relative_pointer_out_of_bounds_is_refused() {
        let page = Page::new();
        let mut buffer = page.buffer_with(request());
        let ptr = root(&buffer) + core::mem::offset_of!(ArchivedRequest, data);
        write_u32(&mut buffer, ptr, 0x0010_0000);
        assert!(refused(&buffer));
    }

    #[test]
    fn invalid_enum_tag_is_refused() {
        let page = Page::new();
        let mut buffer = page.buffer_with(request());
        let tag = root(&buffer) + core::mem::offset_of!(ArchivedRequest, kind);
        buffer[tag] = 0xff;
        assert!(refused(&buffer));
    }

    #[test]
    fn archive_length_past_the_buffer_is_refused() {
        let page = Page::new();
        let range = unsafe { MemoryRange::new(page.0 as usize, PAGE_SIZE) }.unwrap();
        let buffer = unsafe { Buffer::from_inner(range, PAGE_SIZE + 1) };
        assert!(refused(&buffer));
    }
}
