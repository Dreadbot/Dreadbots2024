#![feature(once_cell_try)]
use interprocess::os::unix::udsocket::{tokio::UdStreamListener, UdStream};
#[cfg(feature = "python")]
use pyo3::{
  pyfunction, pymodule, types::PyModule, Bound, IntoPy, Py, PyAny, PyObject,
  PyResult, Python, ToPyObject,
};
use std::{
  io::{Read, Write},
  sync::{Mutex, OnceLock},
};

pub const IPC_STREAM_ID: &'static str = "@timeturner";

#[repr(C)]
pub enum Registration {
  Success(u64),
  Failure,
  NoServer,
}

#[cfg(feature = "python")]
impl IntoPy<Py<PyAny>> for Registration {
  fn into_py(self, py: Python<'_>) -> PyObject {
    match self {
      Registration::Success(val) => val.to_object(py),
      _ => py.None(),
    }
  }
}

#[no_mangle]
#[cfg_attr(feature = "python", pyfunction)]
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

#[cfg(feature = "python")]
#[pymodule]
#[pyo3(name = "timeturner")]
pub fn timeturner(m: &Bound<'_, PyModule>) -> PyResult<()> {
  use pyo3::wrap_pyfunction;

  m.add_function(wrap_pyfunction!(retrieve_next_id, m)?)?;

  Ok(())
}
