//! UART (asynchronous serial) driver for the MSP430FR5969 eUSCI_A modules.
//!
//! This implements the embedded-hal serial traits on top of the device's two
//! eUSCI_A peripherals (`eUSCI_A0`, `eUSCI_A1`) running in UART mode:
//!
//! - [`embedded_hal_nb::serial::Read`] / [`embedded_hal_nb::serial::Write`] —
//!   the non-blocking, word-at-a-time UART abstraction.
//! - [`embedded_io::Read`] / [`embedded_io::Write`] plus
//!   [`embedded_io::ReadReady`] / [`embedded_io::WriteReady`] — blocking,
//!   byte-stream I/O.
//! - [`core::fmt::Write`] on the transmit side, so `write!`/`writeln!` work.
//!
//! # Register access
//!
//! The PAC's UART-mode register block does not expose the interrupt-flag
//! (`UCAxIFG`) or interrupt-enable (`UCAxIE`) registers, which are required to
//! poll TX/RX readiness. We therefore drive the peripheral through raw volatile
//! accesses at its documented base address — the same approach taken by
//! [`crate::gpio`]. The PAC peripheral is still consumed by value so that
//! ownership of the eUSCI_A module is tracked by the type system.
//!
//! # Pin muxing
//!
//! Each eUSCI_A has a fixed pair of port pins for its TXD/RXD signals. The
//! constructor configures the relevant `PxSEL1`/`PxSEL0` bits automatically:
//!
//! | Module    | TXD          | RXD          |
//! |-----------|--------------|--------------|
//! | eUSCI_A0  | P2.0 (SEL=10)| P2.1 (SEL=10)|
//! | eUSCI_A1  | P2.5 (SEL=10)| P2.6 (SEL=10)|
//!
//! # Baud rate
//!
//! [`Config`] takes the BRCLK source frequency and the desired baud rate; the
//! prescaler/modulator registers (`UCBRx`, `UCBRFx`, `UCBRSx`, `UCOS16`) are
//! computed per the procedure in the eUSCI_A UART chapter (SLAU367P §30.3.10),
//! including the `UCBRSx` fractional-modulation lookup table (Table 30-4).

use core::marker::PhantomData;

use embedded_hal_nb::nb;

// ---------------------------------------------------------------------------
// Register layout (offsets from the eUSCI_A base address)
// ---------------------------------------------------------------------------

const CTLW0: usize = 0x00; // Control word 0 (UCSWRST, clock select, frame format)
const BRW: usize = 0x06; // Baud-rate prescaler (UCBRx)
const MCTLW: usize = 0x08; // Modulation control (UCBRSx, UCBRFx, UCOS16)
const STATW: usize = 0x0A; // Status (error flags, UCBUSY)
const RXBUF: usize = 0x0C; // Receive buffer
const TXBUF: usize = 0x0E; // Transmit buffer
const IFG: usize = 0x1C; // Interrupt flags (UCTXIFG, UCRXIFG)

// CTLW0 bit fields
const UCSWRST: u16 = 1 << 0; // Software reset (hold module in reset while = 1)
const UCSSEL_SHIFT: u16 = 6; // Clock source select, bits 7-6
const UCSPB: u16 = 1 << 11; // 0 = one stop bit, 1 = two stop bits
const UC7BIT: u16 = 1 << 12; // 0 = 8-bit data, 1 = 7-bit data
const UCPAR: u16 = 1 << 14; // 0 = odd, 1 = even (only when UCPEN = 1)
const UCPEN: u16 = 1 << 15; // Parity enable

// MCTLW bit fields
const UCOS16: u16 = 1 << 0; // Oversampling mode enable

// STATW bit fields
const UCBUSY: u16 = 1 << 0; // Transmit/receive in progress
const UCRXERR: u16 = 1 << 2; // One or more receive errors occurred
const UCBRK: u16 = 1 << 3; // Break detected
const UCPE: u16 = 1 << 4; // Parity error
const UCOE: u16 = 1 << 5; // Overrun error
const UCFE: u16 = 1 << 6; // Framing error

// IFG bit fields
const UCRXIFG: u16 = 1 << 0; // Receive buffer full
const UCTXIFG: u16 = 1 << 1; // Transmit buffer empty

// Port 2 function-select registers (see crate::gpio for the full map).
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
// eUSCI_A instance markers
// ---------------------------------------------------------------------------

mod sealed {
    pub trait Sealed {}
}

/// Describes a concrete eUSCI_A instance: its register base address and the
/// `PxSEL` bits that mux its TXD/RXD pins to the peripheral.
pub trait Instance: sealed::Sealed {
    /// Absolute base address of the eUSCI_A register block.
    const BASE: usize;
    /// Mask of the TXD+RXD bits within the P2 SEL registers.
    const PIN_MASK: u8;
}

/// Marker for the eUSCI_A0 module (UCA0TXD = P2.0, UCA0RXD = P2.1).
pub struct UsciA0;
/// Marker for the eUSCI_A1 module (UCA1TXD = P2.5, UCA1RXD = P2.6).
pub struct UsciA1;

impl sealed::Sealed for UsciA0 {}
impl Instance for UsciA0 {
    const BASE: usize = 0x05C0;
    const PIN_MASK: u8 = (1 << 0) | (1 << 1); // P2.0, P2.1
}

impl sealed::Sealed for UsciA1 {}
impl Instance for UsciA1 {
    const BASE: usize = 0x05E0;
    const PIN_MASK: u8 = (1 << 5) | (1 << 6); // P2.5, P2.6
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// BRCLK source clock for the baud-rate generator (UCSSELx field).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClockSource {
    /// ACLK (UCSSELx = 01).
    Aclk = 1,
    /// SMCLK (UCSSELx = 10).
    Smclk = 2,
}

/// Parity configuration.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Parity {
    /// No parity bit.
    None,
    /// Even parity.
    Even,
    /// Odd parity.
    Odd,
}

/// Number of stop bits.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StopBits {
    /// One stop bit.
    One,
    /// Two stop bits.
    Two,
}

/// Character length.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DataBits {
    /// 7-bit characters.
    Seven,
    /// 8-bit characters.
    Eight,
}

/// UART configuration: clock, baud rate, and frame format.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Which clock feeds the baud-rate generator.
    pub clock_source: ClockSource,
    /// Frequency of that clock (BRCLK), in Hz.
    pub clock_freq: u32,
    /// Desired baud rate.
    pub baud: u32,
    /// Parity setting.
    pub parity: Parity,
    /// Number of stop bits.
    pub stop_bits: StopBits,
    /// Character length.
    pub data_bits: DataBits,
}

impl Config {
    /// Start from a BRCLK frequency, defaulting to 9600 baud, 8N1, SMCLK.
    pub fn new(clock_freq: u32) -> Self {
        Config {
            clock_source: ClockSource::Smclk,
            clock_freq,
            baud: 9600,
            parity: Parity::None,
            stop_bits: StopBits::One,
            data_bits: DataBits::Eight,
        }
    }

    /// Set the baud rate (builder style).
    pub fn baud(mut self, baud: u32) -> Self {
        self.baud = baud;
        self
    }

    /// Select the clock source (builder style).
    pub fn clock_source(mut self, src: ClockSource) -> Self {
        self.clock_source = src;
        self
    }

    /// Set the parity (builder style).
    pub fn parity(mut self, parity: Parity) -> Self {
        self.parity = parity;
        self
    }

    /// Set the number of stop bits (builder style).
    pub fn stop_bits(mut self, stop_bits: StopBits) -> Self {
        self.stop_bits = stop_bits;
        self
    }

    /// Set the character length (builder style).
    pub fn data_bits(mut self, data_bits: DataBits) -> Self {
        self.data_bits = data_bits;
        self
    }
}

impl Default for Config {
    /// 1 MHz BRCLK (the reset SMCLK on this device: 8 MHz DCO / 8), 9600 8N1.
    fn default() -> Self {
        Config::new(1_000_000)
    }
}

/// Computed baud-rate generator register values.
struct BaudRegs {
    ucbr: u16,
    ucbrf: u8,
    ucbrs: u8,
    oversampling: bool,
}

/// `UCBRSx` second-stage modulation lookup (SLAU367P Table 30-4).
///
/// Each entry is `(fractional_part_x10000, UCBRSx)`. The correct setting is the
/// value of the last entry whose threshold is `<=` the fractional part of N.
const UCBRS_TABLE: [(u32, u8); 36] = [
    (0, 0x00),
    (529, 0x01),
    (715, 0x02),
    (835, 0x04),
    (1001, 0x08),
    (1252, 0x10),
    (1430, 0x20),
    (1670, 0x11),
    (2147, 0x21),
    (2224, 0x22),
    (2503, 0x44),
    (3000, 0x25),
    (3335, 0x49),
    (3575, 0x4A),
    (3753, 0x52),
    (4003, 0x92),
    (4286, 0x53),
    (4378, 0x55),
    (5002, 0xAA),
    (5715, 0x6B),
    (6003, 0xAD),
    (6254, 0xB5),
    (6432, 0xB6),
    (6667, 0xD6),
    (7001, 0xB7),
    (7147, 0xBB),
    (7503, 0xDD),
    (7861, 0xED),
    (8004, 0xEE),
    (8333, 0xBF),
    (8464, 0xDF),
    (8572, 0xEF),
    (8751, 0xF7),
    (9004, 0xFB),
    (9170, 0xFD),
    (9288, 0xFE),
];

/// Look up the `UCBRSx` modulation byte for a fractional part scaled by 10000.
fn ucbrs_lookup(frac_x10000: u32) -> u8 {
    let mut result = 0u8;
    let mut i = 0;
    while i < UCBRS_TABLE.len() {
        if UCBRS_TABLE[i].0 <= frac_x10000 {
            result = UCBRS_TABLE[i].1;
        } else {
            break;
        }
        i += 1;
    }
    result
}

/// Compute the baud-rate generator settings (SLAU367P §30.3.10).
///
/// `N = f_BRCLK / baud`. If `N >= 16`, oversampling mode is used; otherwise
/// low-frequency mode. The fractional part of `N` selects `UCBRSx` from
/// [`UCBRS_TABLE`].
fn compute_baud(clock_freq: u32, baud: u32) -> BaudRegs {
    let n_int = clock_freq / baud; // INT(N)
    // Fractional part of N, scaled by 10000: (clock_freq - n_int*baud)/baud.
    let frac_x10000 = (((clock_freq - n_int * baud) as u64) * 10_000 / baud as u64) as u32;
    let ucbrs = ucbrs_lookup(frac_x10000);

    if n_int >= 16 {
        // Oversampling: UCBRx = INT(N/16), UCBRFx = INT(frac(N/16) * 16).
        let ucbr = clock_freq / (16 * baud);
        let ucbrf = (n_int - 16 * ucbr) as u8; // = INT(N) - 16*UCBRx, in 0..15
        BaudRegs {
            ucbr: ucbr as u16,
            ucbrf,
            ucbrs,
            oversampling: true,
        }
    } else {
        // Low-frequency: UCBRx = INT(N), modulation handled entirely by UCBRSx.
        BaudRegs {
            ucbr: n_int as u16,
            ucbrf: 0,
            ucbrs,
            oversampling: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors that can occur during a UART receive.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {
    /// A character was received before the previous one was read (UCOE).
    Overrun,
    /// Parity check failed (UCPE).
    Parity,
    /// Stop bit was not detected (UCFE).
    Framing,
    /// A break condition was detected (UCBRK).
    Break,
}

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        use embedded_hal_nb::serial::ErrorKind;
        match self {
            Error::Overrun => ErrorKind::Overrun,
            Error::Parity => ErrorKind::Parity,
            Error::Framing => ErrorKind::FrameFormat,
            Error::Break => ErrorKind::Other,
        }
    }
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// A configured UART. Implements the transmit and receive halves of the
/// embedded-hal serial traits; call [`Serial::split`] to separate them.
pub struct Serial<USCI> {
    _usci: PhantomData<USCI>,
}

/// Transmit half of a [`Serial`].
pub struct Tx<USCI> {
    _usci: PhantomData<USCI>,
}

/// Receive half of a [`Serial`].
pub struct Rx<USCI> {
    _usci: PhantomData<USCI>,
}

impl<USCI: Instance> Serial<USCI> {
    /// Configure the eUSCI_A module for UART operation and return the driver.
    ///
    /// Follows the initialization sequence from SLAU367P §30.3.1: hold the
    /// module in reset (`UCSWRST = 1`), program all registers, mux the pins,
    /// then release reset.
    fn init(config: Config) -> Self {
        let base = USCI::BASE;
        let regs = compute_baud(config.clock_freq, config.baud);

        // Build the CTLW0 value (frame format + clock select), keeping the
        // module in reset for now.
        let mut ctlw0 = UCSWRST | ((config.clock_source as u16) << UCSSEL_SHIFT);
        match config.parity {
            Parity::None => {}
            Parity::Even => ctlw0 |= UCPEN | UCPAR,
            Parity::Odd => ctlw0 |= UCPEN,
        }
        if let StopBits::Two = config.stop_bits {
            ctlw0 |= UCSPB;
        }
        if let DataBits::Seven = config.data_bits {
            ctlw0 |= UC7BIT;
        }
        // UCSYNC = 0 and UCMODEx = 00 (plain UART) fall out of the zeroed bits.

        let mut mctlw = ((regs.ucbrs as u16) << 8) | ((regs.ucbrf as u16) << 4);
        if regs.oversampling {
            mctlw |= UCOS16;
        }

        unsafe {
            // 1. Hold in reset (also begins programming the frame format).
            write_reg(base + CTLW0, ctlw0);
            // 2. Program baud-rate generator while UCSWRST = 1.
            write_reg(base + BRW, regs.ucbr);
            write_reg(base + MCTLW, mctlw);
            // 3. Mux the TXD/RXD pins to the eUSCI_A function (SEL1:SEL0 = 10).
            set_bits_u8(P2SEL1, USCI::PIN_MASK);
            clear_bits_u8(P2SEL0, USCI::PIN_MASK);
            // 4. Release the module for operation.
            write_reg(base + CTLW0, ctlw0 & !UCSWRST);
        }

        Serial {
            _usci: PhantomData,
        }
    }

    /// Split the driver into independent transmit and receive halves.
    pub fn split(self) -> (Tx<USCI>, Rx<USCI>) {
        (
            Tx {
                _usci: PhantomData,
            },
            Rx {
                _usci: PhantomData,
            },
        )
    }
}

/// Extension trait to turn a PAC eUSCI_A UART peripheral into a [`Serial`].
pub trait SerialExt {
    /// The eUSCI_A instance marker for this peripheral.
    type Instance: Instance;

    /// Consume the PAC peripheral and configure it as a UART.
    fn into_uart(self, config: Config) -> Serial<Self::Instance>;
}

impl SerialExt for pac::UsciA0UartMode {
    type Instance = UsciA0;
    fn into_uart(self, config: Config) -> Serial<UsciA0> {
        Serial::<UsciA0>::init(config)
    }
}

impl SerialExt for pac::UsciA1UartMode {
    type Instance = UsciA1;
    fn into_uart(self, config: Config) -> Serial<UsciA1> {
        Serial::<UsciA1>::init(config)
    }
}

// ---------------------------------------------------------------------------
// Low-level, instance-generic primitives shared by all the trait impls
// ---------------------------------------------------------------------------

/// Try to push one byte into the transmit buffer. `Err(WouldBlock)` if the TX
/// buffer is not yet empty.
fn try_write_byte<USCI: Instance>(byte: u8) -> nb::Result<(), Error> {
    let base = USCI::BASE;
    if unsafe { read_reg(base + IFG) } & UCTXIFG == 0 {
        return Err(nb::Error::WouldBlock);
    }
    unsafe { write_reg(base + TXBUF, byte as u16) };
    Ok(())
}

/// `Err(WouldBlock)` until the transmit shift register has fully drained.
fn try_flush<USCI: Instance>() -> nb::Result<(), Error> {
    let base = USCI::BASE;
    if unsafe { read_reg(base + STATW) } & UCBUSY != 0 {
        return Err(nb::Error::WouldBlock);
    }
    Ok(())
}

/// Try to read one received byte. `Err(WouldBlock)` if none is pending; a
/// receive error is reported as `Err(Error)` (and the byte is consumed to clear
/// the flags).
fn try_read_byte<USCI: Instance>() -> nb::Result<u8, Error> {
    let base = USCI::BASE;
    if unsafe { read_reg(base + IFG) } & UCRXIFG == 0 {
        return Err(nb::Error::WouldBlock);
    }
    // Read status before the buffer: reading RXBUF clears the error flags.
    let status = unsafe { read_reg(base + STATW) };
    let byte = unsafe { read_reg(base + RXBUF) } as u8;
    if status & UCRXERR != 0 {
        // Priority roughly follows severity; overrun is the most data-losing.
        let err = if status & UCOE != 0 {
            Error::Overrun
        } else if status & UCFE != 0 {
            Error::Framing
        } else if status & UCPE != 0 {
            Error::Parity
        } else if status & UCBRK != 0 {
            Error::Break
        } else {
            Error::Framing
        };
        return Err(nb::Error::Other(err));
    }
    Ok(byte)
}

#[inline]
fn tx_ready<USCI: Instance>() -> bool {
    (unsafe { read_reg(USCI::BASE + IFG) }) & UCTXIFG != 0
}

#[inline]
fn rx_ready<USCI: Instance>() -> bool {
    (unsafe { read_reg(USCI::BASE + IFG) }) & UCRXIFG != 0
}

// ---------------------------------------------------------------------------
// embedded-hal-nb: non-blocking serial traits
// ---------------------------------------------------------------------------

impl<USCI: Instance> embedded_hal_nb::serial::ErrorType for Serial<USCI> {
    type Error = Error;
}
impl<USCI: Instance> embedded_hal_nb::serial::ErrorType for Tx<USCI> {
    type Error = Error;
}
impl<USCI: Instance> embedded_hal_nb::serial::ErrorType for Rx<USCI> {
    type Error = Error;
}

impl<USCI: Instance> embedded_hal_nb::serial::Write<u8> for Serial<USCI> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        try_write_byte::<USCI>(word)
    }
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        try_flush::<USCI>()
    }
}

impl<USCI: Instance> embedded_hal_nb::serial::Write<u8> for Tx<USCI> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        try_write_byte::<USCI>(word)
    }
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        try_flush::<USCI>()
    }
}

impl<USCI: Instance> embedded_hal_nb::serial::Read<u8> for Serial<USCI> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        try_read_byte::<USCI>()
    }
}

impl<USCI: Instance> embedded_hal_nb::serial::Read<u8> for Rx<USCI> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        try_read_byte::<USCI>()
    }
}

// ---------------------------------------------------------------------------
// embedded-io: blocking byte-stream traits
// ---------------------------------------------------------------------------

impl<USCI: Instance> embedded_io::ErrorType for Serial<USCI> {
    type Error = Error;
}
impl<USCI: Instance> embedded_io::ErrorType for Tx<USCI> {
    type Error = Error;
}
impl<USCI: Instance> embedded_io::ErrorType for Rx<USCI> {
    type Error = Error;
}

/// Blocking write of an entire buffer, returning the number of bytes written.
fn io_write<USCI: Instance>(buf: &[u8]) -> Result<usize, Error> {
    for &byte in buf {
        nb::block!(try_write_byte::<USCI>(byte))?;
    }
    Ok(buf.len())
}

/// Blocking read: wait for at least one byte, then drain whatever else is
/// immediately available (without blocking) up to the buffer length.
fn io_read<USCI: Instance>(buf: &mut [u8]) -> Result<usize, Error> {
    if buf.is_empty() {
        return Ok(0);
    }
    buf[0] = nb::block!(try_read_byte::<USCI>())?;
    let mut n = 1;
    while n < buf.len() {
        match try_read_byte::<USCI>() {
            Ok(b) => {
                buf[n] = b;
                n += 1;
            }
            Err(nb::Error::WouldBlock) => break,
            Err(nb::Error::Other(e)) => return Err(e),
        }
    }
    Ok(n)
}

impl<USCI: Instance> embedded_io::Write for Serial<USCI> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        io_write::<USCI>(buf)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        nb::block!(try_flush::<USCI>())
    }
}

impl<USCI: Instance> embedded_io::Write for Tx<USCI> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        io_write::<USCI>(buf)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        nb::block!(try_flush::<USCI>())
    }
}

impl<USCI: Instance> embedded_io::Read for Serial<USCI> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        io_read::<USCI>(buf)
    }
}

impl<USCI: Instance> embedded_io::Read for Rx<USCI> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        io_read::<USCI>(buf)
    }
}

impl<USCI: Instance> embedded_io::WriteReady for Serial<USCI> {
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(tx_ready::<USCI>())
    }
}
impl<USCI: Instance> embedded_io::WriteReady for Tx<USCI> {
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(tx_ready::<USCI>())
    }
}

impl<USCI: Instance> embedded_io::ReadReady for Serial<USCI> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(rx_ready::<USCI>())
    }
}
impl<USCI: Instance> embedded_io::ReadReady for Rx<USCI> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(rx_ready::<USCI>())
    }
}

// ---------------------------------------------------------------------------
// core::fmt::Write — enables write!/writeln! on the transmit side
// ---------------------------------------------------------------------------
//
// NOTE: these impls are deliberately omitted. Pulling in `core::fmt::write`
// (the dynamic formatting engine behind `write!`) costs ~30 KB of `.rodata`,
// which does not fit alongside application code in this device's 48 KB of FRAM.
// Callers that need formatted output can use a stack buffer + `core::fmt` and
// then `embedded_io::Write::write_all`, or pull in a lightweight formatter.
