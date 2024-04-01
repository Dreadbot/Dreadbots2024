#![feature(cfg_match)]
use std::{sync::atomic::{AtomicBool, Ordering}, time::Duration};

use interprocess::os::unix::udsocket::tokio::UdStream;
use timeturner::initialise_server;
use tokio::{
  io::{AsyncReadExt, AsyncWriteExt},
  sync::{mpsc, oneshot},
};
use tokio_gpiod::{Chip, Edge, EdgeDetect, Input, Lines, Options, Output};

const CONSUMER_NAME: &'static str = "timeturner";

static LOCK: AtomicBool = AtomicBool::new(true);

#[tokio::main]
async fn main() -> std::io::Result<()> {
  let listener = initialise_server()?;
  let chip = Chip::new(get_chip_name()).await?;
  let opts_in = Options::input([15])
    .consumer(CONSUMER_NAME)
    .edge(EdgeDetect::Both);
  let opts_out = Options::output([14]).consumer(CONSUMER_NAME);
  let input = chip.request_lines(opts_in).await?;
  let output = chip.request_lines(opts_out).await?;

  let (counter_tx, counter_rx) = mpsc::unbounded_channel();
  let (pulse_tx, pulse_rx) = mpsc::unbounded_channel();

  tokio::spawn(handle_interrupts(input));
  tokio::spawn(handle_counter(pulse_tx, counter_rx));
  tokio::spawn(handle_pulses(output, pulse_rx));

  while let Ok(stream) = listener.accept().await {
    tokio::spawn(handle_stream(stream, counter_tx.clone()));
  }

  Ok(())
}

async fn handle_pulses(
  output: Lines<Output>,
  mut rx: mpsc::UnboundedReceiver<()>,
) {
  while let Some(_) = rx.recv().await {
    output.set_values([true]).await.unwrap();
    tokio::time::sleep(Duration::from_millis(1)).await;
    output.set_values([false]).await.unwrap();
  }
}

async fn handle_counter(
  pulser: mpsc::UnboundedSender<()>,
  mut rx: mpsc::UnboundedReceiver<oneshot::Sender<Option<u64>>>,
) {
  let mut counter = 0;
  while let Some(responder) = rx.recv().await {
    let _ = responder.send(if !LOCK.load(Ordering::Acquire) {
      if pulser.send(()).is_ok() {
        counter += 1;
        Some(counter)
      } else {
        None
      }
    } else {
      None
    });
  }
}

async fn handle_interrupts(mut input: Lines<Input>) {
  loop {
    match input.read_event().await {
      Ok(event) => {
        if event.edge == Edge::Rising {
          LOCK.store(false, Ordering::Release);
        }
      }
      Err(_) => {}
    }
  }
}

async fn handle_stream(
  mut stream: UdStream,
  tx: mpsc::UnboundedSender<oneshot::Sender<Option<u64>>>,
) {
  loop {
    let mut buf = [0x00];
    if stream.read_exact(&mut buf).await.is_err() {
      return;
    }
    let (responder, receiver) = oneshot::channel();
    if tx.send(responder).is_err() {
      return;
    }
    match receiver.await {
      Ok(Some(val)) => {
        if stream
          .write_u8(0x00)
          .await
          .and(stream.write_u64_le(val).await)
          .is_err()
        {
          return;
        }
      }
      Ok(None) => {
        if stream.write_u8(0x01).await.is_err() {
          return;
        }
      }
      Err(_) => {
        return;
      }
    }
  }
}

cfg_match! {
  cfg(feature = "rpi5") => {
    const fn get_chip_name() -> &'static str {
      "/dev/gpiochip4"
    }
  }
  cfg(feature = "rpi4") => {
    const fn get_chip_name() -> &'static str {
      "/dev/gpiochip0"
    }
  }
  _ => {
    const fn get_chip_name() -> &'static str {
      "/dev/null"
    }
  }
}
