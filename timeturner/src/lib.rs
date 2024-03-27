#![feature(once_cell_try)]
use std::{
  io::{Read, Write},
  sync::{Mutex, OnceLock},
};

use interprocess::os::unix::udsocket::{tokio::UdStreamListener, UdStream};

pub const IPC_STREAM_ID: &'static str = "@timeturner";

#[repr(C)]
pub enum Registration {
  Success(u64),
  Failure,
  NoServer,
}

#[no_mangle]
pub extern "C" fn retrieve_next_id() -> Registration {
  static STREAM: OnceLock<Mutex<UdStream>> = OnceLock::new();

  let mut stream =
    match STREAM.get_or_try_init(|| -> std::io::Result<Mutex<UdStream>> {
      Ok(Mutex::new(connect_blocking()?))
    }) {
      Ok(stream) => stream,
      Err(_) => return Registration::NoServer,
    }
    .lock()
    .unwrap();

  if stream.write(&[0x00]).is_err() {
    return Registration::NoServer;
  }

  let mut buf = [0xff];
  if stream.read_exact(&mut buf).is_err() {
    return Registration::NoServer;
  }
  if buf[0] != 0x00 {
    return Registration::Failure;
  }

  let mut buf = [0x00; 8];
  if stream.read_exact(&mut buf).is_err() {
    return Registration::NoServer;
  }
  Registration::Success(u64::from_le_bytes(buf))
}

pub fn connect_blocking() -> std::io::Result<UdStream> {
  UdStream::connect(IPC_STREAM_ID)
}

pub fn initialise_server() -> std::io::Result<UdStreamListener> {
  UdStreamListener::bind(IPC_STREAM_ID)
}
