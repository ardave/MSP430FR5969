// Single-producer/single-consumer byte queue for ISR -> main handoff.
//
// The intended shape is `static RX: critical_section::Mutex<RxQueue<N>> = ...`:
// the UART RX ISR pushes, thread-mode code pops, and *every* access happens
// inside `critical_section::with`. That discipline is what makes the plain
// `Cell` fields sound — `Cell` gives interior mutability through `&self` (so
// the queue works behind the `Mutex`'s shared borrow) but no synchronization
// of its own; the critical section provides all of that. There is no
// `RefCell` on purpose: no borrow-tracking state, no borrow-panic paths in
// the binary, matching the project's Mutex<Cell>-not-RefCell convention.
//
// Overflow policy: **drop-newest, and count it.** When the queue is full,
// `push` discards the incoming byte and bumps a saturating `dropped` tally
// rather than overwriting the oldest queued byte. Rationale: the consumer can
// then trust that whatever it pops is a prefix of what was sent (nothing
// silently vanishes from the middle of a message), and the tally makes the
// loss observable instead of silent. Size the queue so this never happens —
// at 9600 baud a byte lands about once a millisecond, so even a 32-byte
// queue buys ~33 ms of consumer latency.
//
// This file is dependency-free (`//` comments, core-only types) so
// `unit_tests/` can `include!` it and exercise the index arithmetic on the
// host — see the Testing section in CLAUDE.md.

use core::cell::Cell;

// Fixed-capacity SPSC byte queue. `N` must be a power of two (the index mask
// depends on it) and at most 256; both are enforced at compile time.
pub struct RxQueue<const N: usize> {
    buf: [Cell<u8>; N],
    // Free-running u16 indices; `head - tail` (wrapping) is the length, which
    // stays unambiguous because len <= N <= 256 << u16::MAX. The producer
    // (ISR) advances `head`, the consumer (main) advances `tail`.
    head: Cell<u16>,
    tail: Cell<u16>,
    // Bytes discarded by `push` against a full queue. Saturates at 255 —
    // "255 or more dropped" is already an unambiguous sizing failure.
    dropped: Cell<u8>,
}

impl<const N: usize> RxQueue<N> {
    // Compile-time capacity check; referenced in `new` so an invalid N fails
    // the build, not the first wrap.
    const VALID: () = assert!(
        N.is_power_of_two() && N <= 256,
        "RxQueue capacity must be a power of two, at most 256"
    );

    // An empty queue. `const` so it can sit directly in a
    // `static Mutex<RxQueue<N>>` initializer.
    pub const fn new() -> Self {
        let () = Self::VALID;
        RxQueue {
            buf: [const { Cell::new(0) }; N],
            head: Cell::new(0),
            tail: Cell::new(0),
            dropped: Cell::new(0),
        }
    }

    // Number of bytes currently queued.
    pub fn len(&self) -> usize {
        self.head.get().wrapping_sub(self.tail.get()) as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    // Producer side (the ISR): append one byte. Returns `false` — and counts
    // the byte as dropped — if the queue is full.
    pub fn push(&self, byte: u8) -> bool {
        if self.len() == N {
            self.dropped.set(self.dropped.get().saturating_add(1));
            return false;
        }
        let head = self.head.get();
        self.buf[(head as usize) & (N - 1)].set(byte);
        self.head.set(head.wrapping_add(1));
        true
    }

    // Consumer side (main): remove the oldest byte, or `None` if empty.
    pub fn pop(&self) -> Option<u8> {
        if self.is_empty() {
            return None;
        }
        let tail = self.tail.get();
        let byte = self.buf[(tail as usize) & (N - 1)].get();
        self.tail.set(tail.wrapping_add(1));
        Some(byte)
    }

    // How many bytes `push` has discarded against a full queue (saturating).
    pub fn dropped(&self) -> u8 {
        self.dropped.get()
    }
}
