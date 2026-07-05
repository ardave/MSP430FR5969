//! SPI master driver for the eUSCI modules (`embedded_hal::spi::SpiBus`).
//!
//! Synchronous serial: unlike the UART, SPI clocks data out *and* in on the same
//! transfers, driven by a master-generated clock. This driver runs an eUSCI
//! module as a 3-pin SPI **master** (SIMO out, SOMI in, CLK out; no chip-select),
//! which is all a loopback self-test or a single always-selected slave needs.
//!
//! # Three instances
//!
//! All three serial modules on this part can be an SPI master. The eUSCI_A
//! modules are UARTs only until `UCSYNC` flips them synchronous — in SPI mode
//! they expose the same control-word layout as eUSCI_B (SLAU367P chapters 30/31
//! vs 32). Register offsets are also identical **except** the interrupt
//! registers: `IFG` sits at 0x1C on eUSCI_A but 0x2C on eUSCI_B (eUSCI_B's map
//! reserves the middle for its I2C address/mask registers). The [`Instance`]
//! trait carries that one difference plus each module's base and pin mux.
//!
//! | Module   | SIMO | SOMI | CLK  | Cost of claiming it                        |
//! |----------|------|------|------|--------------------------------------------|
//! | eUSCI_A0 | P2.0 | P2.1 | P1.5 | **the backchannel UART** (no more logging) |
//! | eUSCI_A1 | P2.5 | P2.6 | P2.4 | the second UART                            |
//! | eUSCI_B0 | P1.6 | P1.7 | P2.2 | I2C (same module), TB0.3/TB0.4 PWM         |
//!
//! All pins mux at `SEL1:SEL0 = 10`. eUSCI_A1 is the instance that finally
//! allows **SPI and I2C simultaneously** (A1-SPI + B0-I2C) — previously
//! impossible because eUSCI_B0 is one register block serving either role.
//!
//! Note that the PAC exposes *mode-view* singletons, not module singletons:
//! `UsciA0UartMode` and `UsciA0SpiMode` are two windows onto the same silicon
//! (same base address). Consuming one does not consume the other, so the type
//! system cannot stop a program configuring the same module both ways — same
//! long-standing situation as B0's SPI/I2C split. One mode per module per
//! program is the rule; last `init` wins otherwise.
//!
//! # Why raw register access (like `crate::serial`)
//!
//! The PAC models the SPI-mode control words as raw bytes with no typed fields,
//! and (for eUSCI_B) omits the interrupt-flag register entirely. So, as in
//! [`crate::serial`], we drive the peripheral through raw volatile access at
//! known offsets, with explicit, named bit constants. Offsets were taken from
//! the PAC's `usci_*_spi_mode` register blocks; the `CTLW0` bit layout from
//! SLAU367P.
//!
//! # Full-duplex is fundamental
//!
//! Every SPI byte transferred shifts one byte out on SIMO and simultaneously one
//! byte in on SOMI. There is no "write without read": writing `TXBUF` starts the
//! clock, and when the eighth clock edge lands, a received byte appears in
//! `RXBUF` and sets `UCRXIFG`. So the single primitive is
//! [`transfer_byte`](Spi::transfer_byte) (send one, get one); `write` discards
//! the received bytes and `read` sends dummy `0x00`s purely to generate clocks.
//!
//! # Loopback
//!
//! Jumper an instance's SIMO to its SOMI and the bytes you send come straight
//! back — `transfer_in_place(buf)` leaves `buf` unchanged. That is the
//! self-contained hardware test for this driver, needing no external device.
//! (It cannot vouch for the CLK *pin*: the shift clock is internal, so a
//! loopback passes whether or not CLK reaches its pad — CLK muxing is verified
//! against the datasheet pin tables instead.)

use core::marker::PhantomData;

use crate::pac;

// ---------------------------------------------------------------------------
// Register layout (offsets from the eUSCI base address)
// ---------------------------------------------------------------------------

const CTLW0: usize = 0x00; // Control word 0 (reset, mode, clock select, format)
const BRW: usize = 0x06; // Bit-rate prescaler (BRCLK / UCBRW = SPI clock)
const RXBUF: usize = 0x0C; // Receive buffer
const TXBUF: usize = 0x0E; // Transmit buffer
// IFG is per-instance: 0x1C on eUSCI_A, 0x2C on eUSCI_B (see Instance::IFG).

// CTLW0 bit fields (SLAU367P; identical for eUSCI_A and eUSCI_B in SPI mode)
const UCSWRST: u16 = 1 << 0; // Software reset (hold module in reset while = 1)
const UCSSEL_SMCLK: u16 = 0b10 << 6; // BRCLK <- SMCLK (UCSSELx = 10)
const UCSYNC: u16 = 1 << 8; // 1 = synchronous; this is what makes an eUSCI_A an SPI
const UCMST: u16 = 1 << 11; // 1 = master mode
const UCMSB: u16 = 1 << 13; // 1 = MSB first, 0 = LSB first
const UCCKPL: u16 = 1 << 14; // Clock polarity: 0 = idle low, 1 = idle high
const UCCKPH: u16 = 1 << 15; // Clock phase (note: inverted vs SPI CPHA — see SpiMode)
// UCMODEx (bits 10-9) = 00 for 3-pin SPI falls out of the zeroed bits.

// IFG bit fields
const UCRXIFG: u16 = 1 << 0; // Receive buffer full (transfer complete)
const UCTXIFG: u16 = 1 << 1; // Transmit buffer empty (ready for next byte)

// Port function-select registers (see crate::gpio for the full map).
const P1SEL0: usize = 0x020A;
const P1SEL1: usize = 0x020C;
const P2SEL0: usize = 0x020B;
const P2SEL1: usize = 0x020D;

#[inline(always)]
unsafe fn read_reg(addr: usize) -> u16 {
    (addr as *const u16).read_volatile()
}

#[inline(always)]
unsafe fn write_reg(addr: usize, val: u16) {
    (addr as *mut u16).write_volatile(val);
}

#[inline(always)]
unsafe fn set_bits_u8(addr: usize, mask: u8) {
    let p = addr as *mut u8;
    p.write_volatile(p.read_volatile() | mask);
}

#[inline(always)]
unsafe fn clear_bits_u8(addr: usize, mask: u8) {
    let p = addr as *mut u8;
    p.write_volatile(p.read_volatile() & !mask);
}

// ---------------------------------------------------------------------------
// eUSCI instance markers
// ---------------------------------------------------------------------------

mod sealed {
    pub trait Sealed {}
}

/// Describes a concrete eUSCI instance running SPI: its register base, where
/// its interrupt-flag register sits (the one eUSCI_A/eUSCI_B map difference),
/// and which `PxSEL` bits mux its SIMO/SOMI/CLK pins (all at `SEL1:SEL0 = 10`).
pub trait Instance: sealed::Sealed {
    /// Absolute base address of the eUSCI register block.
    const BASE: usize;
    /// Offset of `UCxIFG` from `BASE`: 0x1C on eUSCI_A, 0x2C on eUSCI_B.
    const IFG: usize;
    /// Mask of this instance's SPI pins within the P1 SEL registers.
    const P1_PINS: u8;
    /// Mask of this instance's SPI pins within the P2 SEL registers.
    const P2_PINS: u8;
}

/// Marker for eUSCI_A0 in SPI mode (SIMO = P2.0, SOMI = P2.1, CLK = P1.5).
///
/// **These are the backchannel UART pins** — claiming A0 for SPI forfeits the
/// eZ-FET serial console (and jumpering P2.0↔P2.1 would fight the eZ-FET's
/// own TX driver, so A0 has no safe loopback fixture on the LaunchPad).
pub struct UsciA0;
/// Marker for eUSCI_A1 in SPI mode (SIMO = P2.5, SOMI = P2.6, CLK = P2.4).
pub struct UsciA1;
/// Marker for eUSCI_B0 in SPI mode (SIMO = P1.6, SOMI = P1.7, CLK = P2.2).
pub struct UsciB0;

impl sealed::Sealed for UsciA0 {}
impl Instance for UsciA0 {
    const BASE: usize = 0x05C0;
    const IFG: usize = 0x1C;
    const P1_PINS: u8 = 1 << 5; // P1.5 CLK
    const P2_PINS: u8 = (1 << 0) | (1 << 1); // P2.0 SIMO, P2.1 SOMI
}

impl sealed::Sealed for UsciA1 {}
impl Instance for UsciA1 {
    const BASE: usize = 0x05E0;
    const IFG: usize = 0x1C;
    const P1_PINS: u8 = 0;
    const P2_PINS: u8 = (1 << 4) | (1 << 5) | (1 << 6); // P2.4 CLK, P2.5 SIMO, P2.6 SOMI
}

impl sealed::Sealed for UsciB0 {}
impl Instance for UsciB0 {
    const BASE: usize = 0x0640;
    const IFG: usize = 0x2C;
    const P1_PINS: u8 = (1 << 6) | (1 << 7); // P1.6 SIMO, P1.7 SOMI
    const P2_PINS: u8 = 1 << 2; // P2.2 CLK
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// SPI clock polarity/phase, in the standard CPOL/CPHA "mode 0..3" numbering.
///
/// Maps to the eUSCI bits, where `UCCKPL` *is* CPOL but `UCCKPH` is the
/// **inverse** of CPHA (`UCCKPH = 1` means capture-on-first-edge = CPHA 0).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SpiMode {
    /// CPOL=0, CPHA=0 (idle low, sample leading edge). The common default.
    Mode0,
    /// CPOL=0, CPHA=1.
    Mode1,
    /// CPOL=1, CPHA=0.
    Mode2,
    /// CPOL=1, CPHA=1.
    Mode3,
}

impl SpiMode {
    /// `(UCCKPH, UCCKPL)` bits for this mode.
    fn ckph_ckpl(self) -> (bool, bool) {
        match self {
            SpiMode::Mode0 => (true, false),
            SpiMode::Mode1 => (false, false),
            SpiMode::Mode2 => (true, true),
            SpiMode::Mode3 => (false, true),
        }
    }
}

/// Shift direction: most- or least-significant bit first.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BitOrder {
    /// MSB first (the usual SPI convention).
    MsbFirst,
    /// LSB first.
    LsbFirst,
}

/// SPI master configuration: clock, bit rate, and frame format.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// BRCLK frequency (the SMCLK feeding the bit-rate generator), in Hz.
    pub clock_freq: u32,
    /// Desired SPI clock, in Hz. The actual rate is `clock_freq / round(clock_freq/bit_rate)`.
    pub bit_rate: u32,
    /// Clock polarity/phase.
    pub mode: SpiMode,
    /// Bit order.
    pub bit_order: BitOrder,
}

impl Config {
    /// Start from a BRCLK frequency, defaulting to 1 MHz SPI clock, Mode 0, MSB
    /// first, 8-bit.
    pub fn new(clock_freq: u32) -> Self {
        Config {
            clock_freq,
            bit_rate: 1_000_000,
            mode: SpiMode::Mode0,
            bit_order: BitOrder::MsbFirst,
        }
    }

    /// Set the SPI clock rate (builder style).
    pub fn bit_rate(mut self, bit_rate: u32) -> Self {
        self.bit_rate = bit_rate;
        self
    }

    /// Set the clock polarity/phase mode (builder style).
    pub fn mode(mut self, mode: SpiMode) -> Self {
        self.mode = mode;
        self
    }

    /// Set the bit order (builder style).
    pub fn bit_order(mut self, bit_order: BitOrder) -> Self {
        self.bit_order = bit_order;
        self
    }

    /// The `UCBRW` prescaler value for this config, clamped to `[1, u16::MAX]`.
    fn prescaler(&self) -> u16 {
        let div = self.clock_freq / self.bit_rate.max(1);
        div.clamp(1, u16::MAX as u32) as u16
    }
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// A configured eUSCI SPI master. Implements [`embedded_hal::spi::SpiBus`].
///
/// `PhantomData<*const ()>` keeps it `!Send`/`!Sync` (it owns memory-mapped
/// peripheral state that must not cross threads/contexts) and zero-sized.
pub struct Spi<USCI> {
    _usci: PhantomData<USCI>,
    _not_send: PhantomData<*const ()>,
}

impl<USCI: Instance> Spi<USCI> {
    /// Configure the eUSCI module for 3-pin SPI master operation and return
    /// the driver.
    ///
    /// Follows the SLAU367P initialization sequence: hold in reset
    /// (`UCSWRST = 1`), program control word + bit rate, mux the pins, then
    /// release reset.
    fn init(config: Config) -> Self {
        let base = USCI::BASE;
        let (ckph, ckpl) = config.mode.ckph_ckpl();

        // Synchronous (UCSYNC) master (UCMST), BRCLK = SMCLK, 3-pin (UCMODE=00),
        // held in reset for programming. UCSWRST also sets UCTXIFG, so the first
        // transfer can start immediately after release.
        let mut ctlw0 = UCSWRST | UCSYNC | UCMST | UCSSEL_SMCLK;
        if let BitOrder::MsbFirst = config.bit_order {
            ctlw0 |= UCMSB;
        }
        if ckph {
            ctlw0 |= UCCKPH;
        }
        if ckpl {
            ctlw0 |= UCCKPL;
        }
        // 8-bit (UC7BIT = 0) is the only length exposed here.

        unsafe {
            // 1. Hold in reset while programming.
            write_reg(base + CTLW0, ctlw0);
            // 2. Bit-rate prescaler (SPI clock = BRCLK / UCBRW).
            write_reg(base + BRW, config.prescaler() as u16);
            // 3. Mux SIMO/SOMI/CLK to the eUSCI function (SEL1:SEL0 = 10).
            if USCI::P1_PINS != 0 {
                set_bits_u8(P1SEL1, USCI::P1_PINS);
                clear_bits_u8(P1SEL0, USCI::P1_PINS);
            }
            if USCI::P2_PINS != 0 {
                set_bits_u8(P2SEL1, USCI::P2_PINS);
                clear_bits_u8(P2SEL0, USCI::P2_PINS);
            }
            // 4. Release for operation.
            write_reg(base + CTLW0, ctlw0 & !UCSWRST);
        }

        Spi {
            _usci: PhantomData,
            _not_send: PhantomData,
        }
    }

    /// Full-duplex transfer of one byte: shift `byte` out on SIMO while shifting
    /// the simultaneous SOMI byte in. Blocks until the transfer completes.
    pub fn transfer_byte(&mut self, byte: u8) -> u8 {
        let base = USCI::BASE;
        unsafe {
            // Wait for TXBUF to be free, then start the transfer.
            while read_reg(base + USCI::IFG) & UCTXIFG == 0 {}
            write_reg(base + TXBUF, byte as u16);
            // Wait for the received byte — its arrival means the byte fully
            // clocked out and back in (TX and RX share the clock).
            while read_reg(base + USCI::IFG) & UCRXIFG == 0 {}
            read_reg(base + RXBUF) as u8
        }
    }
}

/// Extension trait to turn a PAC eUSCI SPI-mode peripheral into an [`Spi`].
pub trait SpiExt {
    /// The eUSCI instance marker for this peripheral.
    type Instance: Instance;

    /// Consume the PAC peripheral and configure it as a 3-pin SPI master.
    fn into_spi(self, config: Config) -> Spi<Self::Instance>;
}

impl SpiExt for pac::UsciB0SpiMode {
    type Instance = UsciB0;
    fn into_spi(self, config: Config) -> Spi<UsciB0> {
        // Consuming the PAC singleton proves exclusive ownership of the SPI
        // view of the module; the driver then drives it through raw access.
        Spi::<UsciB0>::init(config)
    }
}

impl SpiExt for pac::UsciA0SpiMode {
    type Instance = UsciA0;
    fn into_spi(self, config: Config) -> Spi<UsciA0> {
        Spi::<UsciA0>::init(config)
    }
}

impl SpiExt for pac::UsciA1SpiMode {
    type Instance = UsciA1;
    fn into_spi(self, config: Config) -> Spi<UsciA1> {
        Spi::<UsciA1>::init(config)
    }
}

// ---------------------------------------------------------------------------
// embedded-hal SpiBus
// ---------------------------------------------------------------------------

impl<USCI: Instance> embedded_hal::spi::ErrorType for Spi<USCI> {
    // A blocking master that reads each RXBUF before clocking the next byte
    // cannot overrun, so transfers never fail.
    type Error = core::convert::Infallible;
}

impl<USCI: Instance> embedded_hal::spi::SpiBus<u8> for Spi<USCI> {
    /// Clock `words.len()` bytes (sending dummy `0x00`), storing what arrives.
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for w in words.iter_mut() {
            *w = self.transfer_byte(0x00);
        }
        Ok(())
    }

    /// Send `words`, discarding the simultaneously-received bytes.
    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &w in words.iter() {
            self.transfer_byte(w);
        }
        Ok(())
    }

    /// Full-duplex: send `write`, store into `read`. If the slices differ in
    /// length, extra sends use `0x00` and extra reads are discarded.
    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let n = read.len().max(write.len());
        for i in 0..n {
            let tx = write.get(i).copied().unwrap_or(0x00);
            let rx = self.transfer_byte(tx);
            if let Some(slot) = read.get_mut(i) {
                *slot = rx;
            }
        }
        Ok(())
    }

    /// Send each byte and overwrite it with the byte received in its place.
    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for w in words.iter_mut() {
            *w = self.transfer_byte(*w);
        }
        Ok(())
    }

    /// No-op: [`transfer_byte`](Spi::transfer_byte) already blocks until each
    /// byte is fully shifted, so nothing is ever left in flight.
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
