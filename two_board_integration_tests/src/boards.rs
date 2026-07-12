//! Board discovery and identity: turn "two /dev nodes" into "the parent and
//! the child", by asking each board who it is.
//!
//! The fixture stores its role in Info FRAM (survives reflash and replug), so
//! device paths never need to be remembered anywhere on the host — every run
//! re-derives the mapping from the boards themselves, and a swapped pair of
//! USB cables changes nothing.

use std::error::Error;
use std::time::{Duration, Instant};

use crate::serial;

/// A provisioned board identity.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Role {
    Parent,
    Child,
}

impl Role {
    pub fn as_str(self) -> &'static str {
        match self {
            Role::Parent => "parent",
            Role::Child => "child",
        }
    }
}

/// One connected, identified board.
pub struct Board {
    pub role: Role,
    pub path: String,
    pub port: Box<dyn serialport::SerialPort>,
}

impl Board {
    /// Send one command byte to the fixture.
    pub fn send(&mut self, cmd: u8) -> Result<(), Box<dyn Error>> {
        self.port.write_all(&[cmd])?;
        self.port.flush()?;
        Ok(())
    }

    /// Read lines until one starts with `prefix`, echoing everything else as
    /// diagnostic context. Errors out at the deadline.
    pub fn expect_prefix(
        &mut self,
        prefix: &str,
        timeout: Duration,
    ) -> Result<String, Box<dyn Error>> {
        let deadline = Instant::now() + timeout;
        loop {
            let line = serial::read_line(self.port.as_mut(), deadline).map_err(|e| {
                format!(
                    "[{}] while waiting for {prefix:?}: {e}",
                    self.role.as_str()
                )
            })?;
            if line.starts_with(prefix) {
                println!("  [{}] {line}", self.role.as_str());
                return Ok(line);
            }
            if !line.is_empty() {
                println!("  [{}] {line}", self.role.as_str());
            }
        }
    }

    /// Send a command and wait for its response line.
    pub fn cmd_expect(
        &mut self,
        cmd: u8,
        prefix: &str,
        timeout: Duration,
    ) -> Result<String, Box<dyn Error>> {
        self.send(cmd)?;
        self.expect_prefix(prefix, timeout)
    }
}

/// The identified rig.
pub struct Rig {
    pub parent: Board,
    pub child: Board,
}

/// Ask the board at `path` who it is. Re-sends `i` every 2 s (the board may
/// still be rebooting out of a DSLite flash) for up to `timeout`.
pub fn identify(path: &str, timeout: Duration) -> Result<(Role, String), Box<dyn Error>> {
    let mut port = serial::open(path)?;
    // Anything buffered (stale READY announces from before we attached) is
    // seconds old — act only on what arrives after we ask.
    port.clear(serialport::ClearBuffer::Input)?;

    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        port.write_all(b"i")?;
        port.flush()?;
        let attempt_deadline = (Instant::now() + Duration::from_secs(2)).min(deadline);
        while let Ok(line) = serial::read_line(port.as_mut(), attempt_deadline) {
            // `2B_ID role=…` answers the probe; a `2B_READY role=…` announce
            // that raced it is just as authoritative.
            if line.starts_with("2B_ID role=") || line.starts_with("2B_READY role=") {
                let fw = serial::field(&line, "fw").unwrap_or(0);
                let role = if line.contains("role=parent") {
                    Some(Role::Parent)
                } else if line.contains("role=child") {
                    Some(Role::Child)
                } else {
                    None
                };
                match role {
                    Some(role) => return Ok((role, format!("fw={fw}"))),
                    None => {
                        return Err(format!(
                            "board at {path} is UNPROVISIONED ({line:?}); run\n  \
                             cargo +nightly run -- provision parent   (with only that board attached)\n\
                             and provision the other board as child"
                        )
                        .into())
                    }
                }
            }
        }
    }
    Err(format!(
        "no fixture answer from {path} — is two_board_fixture flashed and the board powered?"
    )
    .into())
}

/// Discover, identify, and pair both boards.
pub fn discover(timeout: Duration) -> Result<Rig, Box<dyn Error>> {
    let candidates = serial::candidate_ports()?;
    if candidates.len() != 2 {
        return Err(format!(
            "need exactly two eZ-FET backchannels (/dev/cu.usbmodem*3), found {candidates:?}; \
             plug in both boards, or set {}/{}",
            serial::PARENT_PORT_ENV,
            serial::CHILD_PORT_ENV,
        )
        .into());
    }

    let mut parent: Option<Board> = None;
    let mut child: Option<Board> = None;
    for path in candidates {
        let (role, info) = identify(&path, timeout)?;
        println!("  {path}: {} ({info})", role.as_str());
        let port = serial::open(&path)?;
        // identify() ran on its own (now closed) handle; a tail of its last
        // response can still be in flight. Let it land, then drop it.
        std::thread::sleep(Duration::from_millis(100));
        port.clear(serialport::ClearBuffer::Input)?;
        let board = Board {
            role,
            path: path.clone(),
            port,
        };
        let slot = match role {
            Role::Parent => &mut parent,
            Role::Child => &mut child,
        };
        if slot.is_some() {
            return Err(format!(
                "both boards claim to be {:?} — re-provision one of them",
                role.as_str()
            )
            .into());
        }
        *slot = Some(board);
    }

    match (parent, child) {
        (Some(parent), Some(child)) => Ok(Rig { parent, child }),
        _ => Err("did not find one parent and one child board".into()),
    }
}
