use core::convert::Infallible;
use core::marker::PhantomData;

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

// ---------------------------------------------------------------------------
// Pin mode types (typestate)
// ---------------------------------------------------------------------------

/// Input mode. The type parameter selects the pull resistor configuration.
pub struct Input<PULL = Floating> {
    _pull: PhantomData<PULL>,
}

/// Push-pull output mode.
pub struct Output;

/// Analog mode: the pin is handed to an on-chip analog peripheral (e.g. an
/// ADC12_B input channel) rather than the digital I/O logic. Selecting it
/// (`SEL1 = SEL0 = 1`, the "ternary module function") disconnects the digital
/// input buffer so it neither loads the analog signal nor draws crowbar current
/// at mid-rail. See [`crate::adc`] for the pin↔channel mapping.
pub struct Analog;

/// Timer_B0 output mode: the pin is driven by a Timer0_B7 capture/compare
/// output (`TB0.n`) rather than the GPIO latch. Selecting it (`SEL1 = 0,
/// SEL0 = 1`, the **secondary** module function — datasheet SLAS704 Table 6-x)
/// hands the pad to the timer so a PWM waveform appears on it. The direction is
/// also set to output, as the datasheet pin table requires for `TB0.n`. See
/// [`crate::pwm`] for the pin↔channel mapping.
pub struct TimerB;

/// No pull resistor (floating input).
pub struct Floating;

/// Pull-up resistor enabled.
pub struct PullUp;

/// Pull-down resistor enabled.
pub struct PullDown;

// ---------------------------------------------------------------------------
// Port marker types
// ---------------------------------------------------------------------------

/// Port 1 marker.
pub struct P1;
/// Port 2 marker.
pub struct P2;
/// Port 3 marker.
pub struct P3;
/// Port 4 marker.
pub struct P4;

// ---------------------------------------------------------------------------
// Register address mapping
// ---------------------------------------------------------------------------

/// Maps a port marker to the absolute addresses of its 8-bit GPIO registers.
///
/// Port 1/2 share `PORT_1_2` (base 0x0200) with odd/even byte interleaving;
/// Port 3/4 share `PORT_3_4` (base 0x0220) the same way.
pub trait PortRegs {
    const IN: usize;
    const OUT: usize;
    const DIR: usize;
    const REN: usize;
    const SEL0: usize;
    const SEL1: usize;
}

impl PortRegs for P1 {
    const IN: usize = 0x0200;
    const OUT: usize = 0x0202;
    const DIR: usize = 0x0204;
    const REN: usize = 0x0206;
    const SEL0: usize = 0x020A;
    const SEL1: usize = 0x020C;
}

impl PortRegs for P2 {
    const IN: usize = 0x0201;
    const OUT: usize = 0x0203;
    const DIR: usize = 0x0205;
    const REN: usize = 0x0207;
    const SEL0: usize = 0x020B;
    const SEL1: usize = 0x020D;
}

impl PortRegs for P3 {
    const IN: usize = 0x0220;
    const OUT: usize = 0x0222;
    const DIR: usize = 0x0224;
    const REN: usize = 0x0226;
    const SEL0: usize = 0x022A;
    const SEL1: usize = 0x022C;
}

impl PortRegs for P4 {
    const IN: usize = 0x0221;
    const OUT: usize = 0x0223;
    const DIR: usize = 0x0225;
    const REN: usize = 0x0227;
    const SEL0: usize = 0x022B;
    const SEL1: usize = 0x022D;
}

// ---------------------------------------------------------------------------
// Pin type
// ---------------------------------------------------------------------------

/// A single GPIO pin, parameterized by port, pin number (0–7), and mode.
///
/// This is a zero-sized type. Register access goes through the port's
/// memory-mapped addresses using volatile reads/writes.
pub struct Pin<PORT, const N: u8, MODE> {
    _port: PhantomData<PORT>,
    _mode: PhantomData<MODE>,
}

impl<PORT, const N: u8, MODE> Pin<PORT, N, MODE> {
    const fn new() -> Self {
        Pin {
            _port: PhantomData,
            _mode: PhantomData,
        }
    }
}

// ---------------------------------------------------------------------------
// Helpers for volatile register bit manipulation
// ---------------------------------------------------------------------------

#[inline(always)]
unsafe fn set_bit(addr: usize, bit: u8) {
    let p = addr as *mut u8;
    p.write_volatile(p.read_volatile() | (1 << bit));
}

#[inline(always)]
unsafe fn clear_bit(addr: usize, bit: u8) {
    let p = addr as *mut u8;
    p.write_volatile(p.read_volatile() & !(1 << bit));
}

#[inline(always)]
unsafe fn read_bit(addr: usize, bit: u8) -> bool {
    let p = addr as *const u8;
    p.read_volatile() & (1 << bit) != 0
}

/// Clear SEL0 and SEL1 bits to select GPIO function for this pin.
#[inline(always)]
unsafe fn select_gpio<PORT: PortRegs>(bit: u8) {
    clear_bit(PORT::SEL0, bit);
    clear_bit(PORT::SEL1, bit);
}

/// Set SEL0 and SEL1 (`SELx = 11`) to hand the pin to the analog/ternary module
/// function — for our purposes, an ADC12_B input.
#[inline(always)]
unsafe fn select_analog<PORT: PortRegs>(bit: u8) {
    set_bit(PORT::SEL0, bit);
    set_bit(PORT::SEL1, bit);
}

/// Select the **secondary** module function (`SEL0 = 1, SEL1 = 0`) — for our
/// purposes, a Timer_B0 output (`TB0.n`) on its pin.
#[inline(always)]
unsafe fn select_secondary<PORT: PortRegs>(bit: u8) {
    set_bit(PORT::SEL0, bit);
    clear_bit(PORT::SEL1, bit);
}

// ---------------------------------------------------------------------------
// Mode transitions
// ---------------------------------------------------------------------------

impl<PORT: PortRegs, const N: u8, MODE> Pin<PORT, N, MODE> {
    /// Configure as floating input (no pull resistor).
    pub fn into_floating_input(self) -> Pin<PORT, N, Input<Floating>> {
        unsafe {
            clear_bit(PORT::DIR, N);
            clear_bit(PORT::REN, N);
            select_gpio::<PORT>(N);
        }
        Pin::new()
    }

    /// Configure as input with internal pull-up resistor.
    pub fn into_pull_up_input(self) -> Pin<PORT, N, Input<PullUp>> {
        unsafe {
            clear_bit(PORT::DIR, N);
            set_bit(PORT::OUT, N); // OUT=1 selects pull-up
            set_bit(PORT::REN, N);
            select_gpio::<PORT>(N);
        }
        Pin::new()
    }

    /// Configure as input with internal pull-down resistor.
    pub fn into_pull_down_input(self) -> Pin<PORT, N, Input<PullDown>> {
        unsafe {
            clear_bit(PORT::DIR, N);
            clear_bit(PORT::OUT, N); // OUT=0 selects pull-down
            set_bit(PORT::REN, N);
            select_gpio::<PORT>(N);
        }
        Pin::new()
    }

    /// Configure as push-pull output, driving low initially.
    ///
    /// The output latch (`PxOUT`) is **not** reset to a known value by reset or
    /// by switching `PxDIR` to output — it retains whatever it held before. So
    /// we drive the pin low here *before* enabling the output, otherwise a
    /// freshly-configured output would drive an indeterminate level (and could
    /// glitch the pin high for an instant) until the caller's first
    /// `set_high`/`set_low`. Clearing `PxOUT` first makes the level defined.
    pub fn into_output(self) -> Pin<PORT, N, Output> {
        unsafe {
            clear_bit(PORT::OUT, N); // defined initial level (low) before driving
            set_bit(PORT::DIR, N);
            clear_bit(PORT::REN, N);
            select_gpio::<PORT>(N);
        }
        Pin::new()
    }

    /// Configure as an analog input for an on-chip peripheral (ADC12_B).
    ///
    /// Selects the ternary module function (`SELx = 11`), which detaches the
    /// digital input buffer. The pin direction is left as input with no pull
    /// resistor so it presents a high impedance to the analog source. The
    /// resulting [`Pin<_, _, Analog>`] is what [`crate::adc::Adc::read`] accepts;
    /// only pins that are actually wired to an ADC channel implement
    /// [`crate::adc::AdcPin`], so a non-analog pin is rejected at compile time.
    pub fn into_analog(self) -> Pin<PORT, N, Analog> {
        unsafe {
            clear_bit(PORT::DIR, N);
            clear_bit(PORT::REN, N);
            select_analog::<PORT>(N);
        }
        Pin::new()
    }

    /// Configure as a **Timer_B0 output** (`TB0.n`) for PWM.
    ///
    /// Selects the secondary module function (`SEL0 = 1, SEL1 = 0`) so the
    /// Timer0_B7 compare logic — not the GPIO latch — drives the pad, and sets
    /// the direction to output as the datasheet pin table requires for the
    /// `TB0.n` function. The resulting [`Pin<_, _, TimerB>`] is what
    /// [`crate::pwm::Pwm::channel`] accepts; only pins actually wired to a
    /// Timer_B0 channel implement [`crate::pwm::PwmPin`], so routing a pin with
    /// no `TB0.n` function is a compile error. Which channel a pin feeds is fixed
    /// by silicon — see the table in [`crate::pwm`].
    pub fn into_timer_b_output(self) -> Pin<PORT, N, TimerB> {
        unsafe {
            set_bit(PORT::DIR, N); // output: the timer drives the pad
            clear_bit(PORT::REN, N);
            select_secondary::<PORT>(N);
        }
        Pin::new()
    }
}

// ---------------------------------------------------------------------------
// embedded-hal trait implementations
// ---------------------------------------------------------------------------

impl<PORT, const N: u8, MODE> ErrorType for Pin<PORT, N, MODE> {
    type Error = Infallible;
}

impl<PORT: PortRegs, const N: u8, PULL> InputPin for Pin<PORT, N, Input<PULL>> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(unsafe { read_bit(PORT::IN, N) })
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!unsafe { read_bit(PORT::IN, N) })
    }
}

impl<PORT: PortRegs, const N: u8> OutputPin for Pin<PORT, N, Output> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        unsafe { set_bit(PORT::OUT, N) };
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        unsafe { clear_bit(PORT::OUT, N) };
        Ok(())
    }
}

impl<PORT: PortRegs, const N: u8> StatefulOutputPin for Pin<PORT, N, Output> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(unsafe { read_bit(PORT::OUT, N) })
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!unsafe { read_bit(PORT::OUT, N) })
    }
}

// ---------------------------------------------------------------------------
// Port splitting — consumes PAC peripherals, returns individual typed pins
// ---------------------------------------------------------------------------

macro_rules! port_parts {
    ($PortParts:ident, $Port:ident) => {
        pub struct $PortParts {
            pub pin0: Pin<$Port, 0, Input<Floating>>,
            pub pin1: Pin<$Port, 1, Input<Floating>>,
            pub pin2: Pin<$Port, 2, Input<Floating>>,
            pub pin3: Pin<$Port, 3, Input<Floating>>,
            pub pin4: Pin<$Port, 4, Input<Floating>>,
            pub pin5: Pin<$Port, 5, Input<Floating>>,
            pub pin6: Pin<$Port, 6, Input<Floating>>,
            pub pin7: Pin<$Port, 7, Input<Floating>>,
        }

        impl $PortParts {
            fn new() -> Self {
                Self {
                    pin0: Pin::new(),
                    pin1: Pin::new(),
                    pin2: Pin::new(),
                    pin3: Pin::new(),
                    pin4: Pin::new(),
                    pin5: Pin::new(),
                    pin6: Pin::new(),
                    pin7: Pin::new(),
                }
            }
        }
    };
}

port_parts!(Port1Parts, P1);
port_parts!(Port2Parts, P2);
port_parts!(Port3Parts, P3);
port_parts!(Port4Parts, P4);

/// Extension trait to split a PAC port peripheral into individual typed pins.
pub trait GpioExt {
    type Parts;

    /// Consume the PAC peripheral and return individual pin objects.
    fn split(self) -> Self::Parts;
}

impl GpioExt for pac::Port1_2 {
    type Parts = (Port1Parts, Port2Parts);

    fn split(self) -> Self::Parts {
        (Port1Parts::new(), Port2Parts::new())
    }
}

impl GpioExt for pac::Port3_4 {
    type Parts = (Port3Parts, Port4Parts);

    fn split(self) -> Self::Parts {
        (Port3Parts::new(), Port4Parts::new())
    }
}
