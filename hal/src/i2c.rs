//! eUSCI_B0 I2C master driver (`embedded_hal::i2c::I2c`).
//!
//! I2C is the other synchronous-serial personality of eUSCI_B0 (the same
//! peripheral [`crate::spi`] drives as SPI — they share one register block at
//! `0x0640` and the same P1.6/P1.7 pins, so only one can be live at a time). It
//! is a two-wire, open-drain, multi-drop bus: **SDA** (data) and **SCL** (clock),
//! both needing external pull-up resistors. A *master* owns the clock and frames
//! every exchange between a START and a STOP; each byte is acknowledged by the
//! receiver pulling SDA low for a ninth clock (ACK) or leaving it high (NACK).
//!
//! This driver is a single-master, blocking, polled I2C **master** for 7-bit
//! addresses. It implements [`embedded_hal::i2c::I2c`], whose one required
//! method — [`transaction`](embedded_hal::i2c::I2c::transaction) — is where all
//! the bus framing lives; `read`/`write`/`write_read` are the trait's provided
//! methods built on top of it.
//!
//! # Why raw register access (like [`crate::spi`] and [`crate::serial`])
//!
//! eUSCI_B's PAC view models the I2C control word as two fieldless raw bytes
//! (`UCB0CTL1`/`UCB0CTL0`) with no typed bits, and parks the interrupt-flag
//! register at offset `0x2C`. So, as in the SPI and UART drivers, we drive the
//! peripheral through raw volatile access at named offsets from the eUSCI_B0
//! base, with explicit bit constants taken from SLAU367P.
//!
//! # The framing the hardware does for you
//!
//! Setting `UCTXSTT` makes the module emit START + the 7-bit slave address +
//! the R/W bit, then it self-clears once that address byte's ACK/NACK has come
//! back — so polling `UCTXSTT` low is exactly "address phase done", after which
//! `UCNACKIFG` tells you whether a device answered. Setting `UCTXSTP` emits a
//! STOP. Leaving the bus held (no STOP) and flipping `UCTR` + setting `UCTXSTT`
//! again emits a *repeated* START — which is how a `write_read` turns the bus
//! around without releasing it.
//!
//! # Testing
//!
//! Unlike SPI, I2C can't self-test by jumpering two pins. The minimal hardware
//! test is a **bus scan**: with SDA/SCL pulled up, probe every address 0..=0x77
//! with a zero-length write and watch which ones ACK. Any real I2C device on the
//! bus (or none — every address NACKs) exercises the address-phase and
//! NACK-handling paths.

use core::marker::PhantomData;

use crate::pac;

// ---------------------------------------------------------------------------
// Register layout (offsets from the eUSCI_B0 base address, 0x0640)
// ---------------------------------------------------------------------------

const BASE: usize = 0x0640;

const CTLW0: usize = 0x00; // Control word 0 (reset, mode, START/STOP/dir, clock)
const BRW: usize = 0x06; // Bit-rate prescaler (SCL = BRCLK / UCBRW)
const RXBUF: usize = 0x0C; // Receive buffer
const TXBUF: usize = 0x0E; // Transmit buffer
const I2CSA: usize = 0x20; // Slave address (7-bit, right-justified)
const IFG: usize = 0x2C; // Interrupt flags (0x2C on eUSCI_B, as in SPI)

// CTLW0 bit fields (SLAU367P eUSCI_B I2C register description). The low byte is
// UCB0CTL1, the high byte UCB0CTL0; together they form this 16-bit word.
const UCSWRST: u16 = 1 << 0; // Software reset (hold module in reset while = 1)
const UCTXSTT: u16 = 1 << 1; // Transmit START condition (self-clears after addr)
const UCTXSTP: u16 = 1 << 2; // Transmit STOP condition (self-clears after STOP)
const UCTR: u16 = 1 << 4; // Transmitter (1) / receiver (0)
const UCSSEL_SMCLK: u16 = 0b10 << 6; // BRCLK <- SMCLK (UCSSELx = 10)
const UCSYNC: u16 = 1 << 8; // 1 = synchronous mode; must be set for I2C
const UCMODE_I2C: u16 = 0b11 << 9; // UCMODEx = 11 selects I2C
const UCMST: u16 = 1 << 11; // 1 = master mode

// IFG bit fields (low byte of the flags word).
const UCRXIFG: u16 = 1 << 0; // Receive buffer full (a byte arrived)
const UCTXIFG: u16 = 1 << 1; // Transmit buffer empty (ready for next byte)
const UCNACKIFG: u16 = 1 << 5; // NACK received (no/!ACK from the addressed slave)

// ---------------------------------------------------------------------------
// Pin mux: SDA = P1.6, SCL = P1.7, both at SEL1:SEL0 = 10.
//
// These are the same physical pins (and the same function-select code) the SPI
// driver muxes as SIMO/SOMI: the pin mux only connects eUSCI_B0 to the pin; the
// peripheral routes SDA/SCL vs SIMO/SOMI internally based on UCMODEx. So I2C
// needs no separate clock pin — SCL rides on what SPI calls the SOMI pin.
// ---------------------------------------------------------------------------

const P1SEL0: usize = 0x020A;
const P1SEL1: usize = 0x020C;
const P1_I2C_PINS: u8 = (1 << 6) | (1 << 7); // P1.6 SDA, P1.7 SCL

#[inline(always)]
unsafe fn read_reg(addr: usize) -> u16 {
    (addr as *const u16).read_volatile()
}

#[inline(always)]
unsafe fn write_reg(addr: usize, val: u16) {
    (addr as *mut u16).write_volatile(val);
}

#[inline(always)]
unsafe fn set_bits_u16(addr: usize, mask: u16) {
    write_reg(addr, read_reg(addr) | mask);
}

#[inline(always)]
unsafe fn clear_bits_u16(addr: usize, mask: u16) {
    write_reg(addr, read_reg(addr) & !mask);
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
// Errors
// ---------------------------------------------------------------------------

/// Reasons an I2C transaction can fail.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {
    /// No device acknowledged the address byte (nothing at this address, or it
    /// is busy / held in reset).
    AddressNack,
    /// The addressed device acknowledged its address but then NACKed a data
    /// byte (e.g. it has no room, or rejected the command).
    DataNack,
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
        match self {
            Error::AddressNack => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
            Error::DataNack => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
        }
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// I2C master configuration: source clock and target SCL frequency.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// BRCLK frequency (the SMCLK feeding the bit-rate generator), in Hz.
    pub clock_freq: u32,
    /// Desired SCL frequency, in Hz. Actual = `clock_freq / round(clock_freq/scl_freq)`.
    pub scl_freq: u32,
}

impl Config {
    /// Start from a BRCLK frequency, defaulting to 100 kHz standard-mode SCL.
    pub fn new(clock_freq: u32) -> Self {
        Config {
            clock_freq,
            scl_freq: 100_000,
        }
    }

    /// Set the SCL frequency (builder style). 100 kHz standard / 400 kHz fast
    /// are the usual choices.
    pub fn scl_freq(mut self, scl_freq: u32) -> Self {
        self.scl_freq = scl_freq;
        self
    }

    /// The `UCBRW` prescaler value for this config, clamped to `[1, u16::MAX]`.
    fn prescaler(&self) -> u16 {
        let div = self.clock_freq / self.scl_freq.max(1);
        div.clamp(1, u16::MAX as u32) as u16
    }
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// A configured eUSCI_B0 I2C master. Implements [`embedded_hal::i2c::I2c`].
///
/// `PhantomData<*const ()>` keeps it `!Send`/`!Sync` (it owns memory-mapped
/// peripheral state that must not cross threads/contexts) and zero-sized.
pub struct I2c {
    _not_send: PhantomData<*const ()>,
}

impl I2c {
    /// Configure eUSCI_B0 for single-master I2C and return the driver.
    ///
    /// Follows the SLAU367P init sequence: hold in reset (`UCSWRST = 1`),
    /// program the control word + bit rate, mux the pins, then release reset.
    fn init(config: Config) -> Self {
        // Synchronous (UCSYNC) I2C (UCMODE=11) master (UCMST), BRCLK = SMCLK,
        // held in reset for programming.
        let ctlw0 = UCSWRST | UCSYNC | UCMODE_I2C | UCMST | UCSSEL_SMCLK;

        unsafe {
            // 1. Hold in reset while programming.
            write_reg(BASE + CTLW0, ctlw0);
            // 2. Bit-rate prescaler (SCL = BRCLK / UCBRW).
            write_reg(BASE + BRW, config.prescaler());
            // 3. Mux SDA/SCL to the eUSCI_B0 function (SEL1:SEL0 = 10).
            set_bits_u8(P1SEL1, P1_I2C_PINS);
            clear_bits_u8(P1SEL0, P1_I2C_PINS);
            // 4. Release for operation.
            write_reg(BASE + CTLW0, ctlw0 & !UCSWRST);
        }

        I2c {
            _not_send: PhantomData,
        }
    }

    /// Probe an address with a zero-length write: returns `true` if a device
    /// ACKs. The building block of an I2C bus scan.
    pub fn probe(&mut self, address: u8) -> bool {
        use embedded_hal::i2c::I2c as _;
        self.write(address, &[]).is_ok()
    }

    /// Emit a STOP and block until the bus is released. Used on both the success
    /// and error paths so the bus is never left held.
    fn stop(&self) {
        unsafe {
            set_bits_u16(BASE + CTLW0, UCTXSTP);
            while read_reg(BASE + CTLW0) & UCTXSTP != 0 {}
        }
    }

    /// Drive one same-direction *write* run: (repeated) START + address, then
    /// every byte of `ops` (which are all `Write`s) back-to-back. Sends a STOP
    /// only if `send_stop` — otherwise the bus stays held for the next run's
    /// repeated START.
    fn write_run(
        &mut self,
        ops: &[embedded_hal::i2c::Operation<'_>],
        send_stop: bool,
    ) -> Result<(), Error> {
        unsafe {
            // Transmitter + START. Clearing UCNACKIFG first so we read a fresh
            // address-ACK result below.
            clear_bits_u16(BASE + IFG, UCNACKIFG);
            set_bits_u16(BASE + CTLW0, UCTR | UCTXSTT);
            // UCTXSTT self-clears once the address byte (and its ACK/NACK) is done.
            while read_reg(BASE + CTLW0) & UCTXSTT != 0 {}
            if read_reg(BASE + IFG) & UCNACKIFG != 0 {
                self.stop();
                return Err(Error::AddressNack);
            }

            for op in ops {
                if let embedded_hal::i2c::Operation::Write(bytes) = op {
                    for &b in *bytes {
                        // Wait until TXBUF can take the next byte, bailing if the
                        // slave NACKed the previous one.
                        loop {
                            let ifg = read_reg(BASE + IFG);
                            if ifg & UCNACKIFG != 0 {
                                self.stop();
                                return Err(Error::DataNack);
                            }
                            if ifg & UCTXIFG != 0 {
                                break;
                            }
                        }
                        write_reg(BASE + TXBUF, b as u16);
                    }
                }
            }

            // Wait for the final byte to leave TXBUF before we (optionally) STOP,
            // so STOP follows a fully transmitted byte rather than truncating it.
            loop {
                let ifg = read_reg(BASE + IFG);
                if ifg & UCNACKIFG != 0 {
                    self.stop();
                    return Err(Error::DataNack);
                }
                if ifg & UCTXIFG != 0 {
                    break;
                }
            }

            if send_stop {
                self.stop();
            }
        }
        Ok(())
    }

    /// Drive one same-direction *read* run: (repeated) START + address, then
    /// fill every `Read` buffer in `ops` from a continuous byte stream. When
    /// `send_stop`, arranges for the master to NACK + STOP the final byte (the
    /// timing of which differs for a single byte vs many — see below).
    fn read_run(
        &mut self,
        ops: &mut [embedded_hal::i2c::Operation<'_>],
        send_stop: bool,
    ) -> Result<(), Error> {
        let total: usize = ops
            .iter()
            .map(|o| match o {
                embedded_hal::i2c::Operation::Read(buf) => buf.len(),
                _ => 0,
            })
            .sum();

        unsafe {
            // Receiver + START.
            clear_bits_u16(BASE + IFG, UCNACKIFG);
            clear_bits_u16(BASE + CTLW0, UCTR);
            set_bits_u16(BASE + CTLW0, UCTXSTT);
            while read_reg(BASE + CTLW0) & UCTXSTT != 0 {}
            if read_reg(BASE + IFG) & UCNACKIFG != 0 {
                self.stop();
                return Err(Error::AddressNack);
            }

            // A single-byte read must request STOP *now* (right after the address
            // phase, before the one and only byte finishes), so the master NACKs
            // it. For multi-byte reads, STOP is requested just before the last
            // byte instead (handled in the loop below).
            if send_stop && total == 1 {
                set_bits_u16(BASE + CTLW0, UCTXSTP);
            }

            let mut idx = 0usize;
            for op in ops {
                if let embedded_hal::i2c::Operation::Read(buf) = op {
                    for slot in buf.iter_mut() {
                        // Request STOP before clocking in the final byte so it is
                        // NACKed (telling the slave to stop driving SDA).
                        if send_stop && total > 1 && idx == total - 1 {
                            set_bits_u16(BASE + CTLW0, UCTXSTP);
                        }
                        while read_reg(BASE + IFG) & UCRXIFG == 0 {}
                        *slot = read_reg(BASE + RXBUF) as u8;
                        idx += 1;
                    }
                }
            }

            if send_stop {
                // total == 0 (zero-length read probe): no byte was clocked, so
                // STOP still needs issuing here. Otherwise this just waits for the
                // STOP requested above to complete.
                if total == 0 {
                    set_bits_u16(BASE + CTLW0, UCTXSTP);
                }
                while read_reg(BASE + CTLW0) & UCTXSTP != 0 {}
            }
        }
        Ok(())
    }
}

/// Extension trait to turn the PAC eUSCI_B0 I2C peripheral into an [`I2c`].
pub trait I2cExt {
    /// Consume the PAC peripheral and configure it as an I2C master.
    fn into_i2c(self, config: Config) -> I2c;
}

impl I2cExt for pac::UsciB0I2cMode {
    fn into_i2c(self, config: Config) -> I2c {
        // Consuming the PAC singleton proves exclusive ownership of eUSCI_B0;
        // the driver then drives it through raw access.
        I2c::init(config)
    }
}

// ---------------------------------------------------------------------------
// embedded-hal I2c
// ---------------------------------------------------------------------------

impl embedded_hal::i2c::ErrorType for I2c {
    type Error = Error;
}

impl embedded_hal::i2c::I2c<embedded_hal::i2c::SevenBitAddress> for I2c {
    /// Run a transaction as a sequence of operations against one address.
    ///
    /// Honors the `embedded-hal` framing contract: a (repeated) START before
    /// each run of same-direction operations, the byte streams of adjacent
    /// same-direction operations concatenated without an intervening
    /// START/STOP, and a single STOP after the last operation. A direction
    /// change (write→read or read→write) emits a repeated START with the same
    /// address. NACKs abort the transaction with the bus released.
    fn transaction(
        &mut self,
        address: embedded_hal::i2c::SevenBitAddress,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        if operations.is_empty() {
            return Ok(());
        }

        unsafe {
            // The 7-bit address, right-justified, is constant for the whole
            // transaction (repeated STARTs re-send this same address).
            write_reg(BASE + I2CSA, address as u16);
            // Make sure any prior STOP has finished and the bus is idle.
            while read_reg(BASE + CTLW0) & UCTXSTP != 0 {}
        }

        let n = operations.len();
        let mut i = 0;
        while i < n {
            let is_read = matches!(operations[i], embedded_hal::i2c::Operation::Read(_));
            // Extend the run over all following same-direction operations.
            let mut j = i + 1;
            while j < n
                && matches!(operations[j], embedded_hal::i2c::Operation::Read(_)) == is_read
            {
                j += 1;
            }
            let is_last_run = j == n;

            if is_read {
                self.read_run(&mut operations[i..j], is_last_run)?;
            } else {
                self.write_run(&operations[i..j], is_last_run)?;
            }
            i = j;
        }
        Ok(())
    }
}
