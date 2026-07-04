//! Host-side tests for the ISR→main SPSC byte queue.
//!
//! This module `include!`s the REAL source (`hal/src/rx_queue.rs`) so the tests
//! exercise the exact index arithmetic that ships in firmware — a bad edit to
//! the wrap mask, the full/empty distinction, or the drop-newest overflow path
//! fails here without a board on the desk. (The synchronization story —
//! `Mutex` + critical sections around every access — is the firmware's
//! responsibility and can't be tested on the host; these tests cover the
//! single-threaded queue mechanics that discipline protects.)

// Pull in the actual queue (pure core-only code, `Cell` included).
include!("../../hal/src/rx_queue.rs");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn starts_empty() {
        let q: RxQueue<8> = RxQueue::new();
        assert!(q.is_empty());
        assert_eq!(q.len(), 0);
        assert_eq!(q.pop(), None);
        assert_eq!(q.dropped(), 0);
    }

    #[test]
    fn fifo_order() {
        let q: RxQueue<8> = RxQueue::new();
        for b in b"abc" {
            assert!(q.push(*b));
        }
        assert_eq!(q.len(), 3);
        assert_eq!(q.pop(), Some(b'a'));
        assert_eq!(q.pop(), Some(b'b'));
        assert_eq!(q.pop(), Some(b'c'));
        assert_eq!(q.pop(), None);
    }

    /// Fill to capacity: the Nth push fits, the N+1st is dropped (drop-newest)
    /// and counted; the queued prefix comes back intact.
    #[test]
    fn full_drops_newest_and_counts() {
        let q: RxQueue<4> = RxQueue::new();
        for b in 0..4u8 {
            assert!(q.push(b));
        }
        assert_eq!(q.len(), 4);
        assert!(!q.push(99));
        assert_eq!(q.dropped(), 1);
        assert_eq!(q.len(), 4, "a dropped push must not disturb the queue");
        for b in 0..4u8 {
            assert_eq!(q.pop(), Some(b), "queued prefix must survive overflow");
        }
        assert_eq!(q.pop(), None);
    }

    /// Drain-and-refill far past the index width's wrap point: the u16
    /// free-running indices and the power-of-two mask must keep agreeing.
    /// 100_000 cycles pushes the indices through wrapping several times
    /// (100_000 mod 65_536, and every mask wrap in between).
    #[test]
    fn survives_index_wraparound() {
        let q: RxQueue<8> = RxQueue::new();
        for i in 0..100_000u32 {
            let b = (i % 251) as u8; // non-power-of-two modulus: catches aliasing
            assert!(q.push(b));
            assert_eq!(q.pop(), Some(b), "mismatch at cycle {i}");
        }
        assert!(q.is_empty());
        assert_eq!(q.dropped(), 0);
    }

    /// Interleaved producer/consumer with the queue partially full across the
    /// wrap: contents must stay in order with no loss.
    #[test]
    fn interleaved_partial_fill() {
        let q: RxQueue<8> = RxQueue::new();
        let mut expected: u8 = 0;
        let mut next: u8 = 0;
        for _ in 0..1000 {
            // Push two, pop one — queue grows until full, then drops kick in;
            // stop pushing at 6 to stay clear of full and keep this lossless.
            for _ in 0..2 {
                if q.len() < 6 {
                    assert!(q.push(next));
                    next = next.wrapping_add(1);
                }
            }
            assert_eq!(q.pop(), Some(expected));
            expected = expected.wrapping_add(1);
        }
        while let Some(b) = q.pop() {
            assert_eq!(b, expected);
            expected = expected.wrapping_add(1);
        }
        assert_eq!(q.dropped(), 0);
    }

    /// The drop tally saturates at 255 rather than wrapping back to zero
    /// (which would masquerade as "no loss").
    #[test]
    fn dropped_saturates() {
        let q: RxQueue<2> = RxQueue::new();
        assert!(q.push(0));
        assert!(q.push(1));
        for _ in 0..300 {
            assert!(!q.push(2));
        }
        assert_eq!(q.dropped(), 255);
    }
}
