#![no_std]
#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]
#![warn(missing_docs, missing_debug_implementations)]
//! AS5048A SPI absolute-encoder driver.
//!
//! The driver owns the protocol details that are easy to get wrong on this
//! sensor:
//!
//! - 16-bit framed SPI transactions with chip-select toggled per transfer
//! - even-parity command generation and response verification
//! - pipelined register reads using a second `NOP` transfer
//! - `EF` response handling
//! - decoded diagnostics and angle conversion helpers
//! - software zero-offset support for bring-up without OTP programming
//!
//! The SPI peripheral must still be configured with the mode and timing
//! required by the AS5048A on the target platform.

use core::fmt;

use embedded_hal::spi::SpiDevice;
#[cfg(feature = "async")]
use embedded_hal_async::spi::SpiDevice as AsyncSpiDevice;

#[cfg(feature = "fluxkit")]
use fluxkit_math::{MechanicalAngle, Seconds, angle::shortest_angle_delta, units::RadPerSec};

const DATA_MASK: u16 = 0x3FFF;
const EF_MASK: u16 = 0x4000;
const READ_MASK: u16 = 0x4000;
const PARITY_MASK: u16 = 0x8000;
const ANGLE_COUNTS_PER_REV: u16 = 1 << 14;
const RADIANS_PER_COUNT: f32 = core::f32::consts::TAU / ANGLE_COUNTS_PER_REV as f32;
const DEGREES_PER_COUNT: f32 = 360.0 / ANGLE_COUNTS_PER_REV as f32;

/// Wrapped angle in radians, normalized into `[-pi, pi)`.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Angle(f32);

impl Angle {
    /// Creates a wrapped angle from radians.
    #[inline]
    pub fn from_radians(value: f32) -> Self {
        Self(wrap_radians(value))
    }

    /// Creates an angle directly from a raw AS5048A 14-bit count.
    #[inline]
    pub fn from_raw(raw: u16) -> Self {
        Self::from_radians(raw_to_radians(raw))
    }

    /// Returns the wrapped angle in radians.
    #[inline]
    pub const fn radians(self) -> f32 {
        self.0
    }

    /// Returns the wrapped angle in degrees.
    #[inline]
    pub fn degrees(self) -> f32 {
        self.0 * (180.0 / core::f32::consts::PI)
    }
}

/// AS5048A register addresses relevant for bring-up and runtime sensing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
enum Register {
    /// Error register. Reading it clears the current SPI/protocol error state.
    ClearErrorFlag = 0x0001,
    /// Diagnostics and AGC register.
    DiagnosticsAgc = 0x3FFD,
    /// 14-bit CORDIC magnitude register.
    Magnitude = 0x3FFE,
    /// 14-bit corrected angle register.
    Angle = 0x3FFF,
}

impl Register {
    #[inline]
    const fn address(self) -> u16 {
        self as u16
    }
}

/// SPI/protocol error bits reported by the AS5048A error register.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ErrorFlags {
    /// Command parity was invalid.
    pub parity_error: bool,
    /// Command format or register access was invalid.
    pub command_invalid: bool,
    /// SPI frame timing or mode produced a framing error.
    pub framing_error: bool,
}

impl ErrorFlags {
    /// Builds decoded error flags from the `0x0001` register payload.
    #[inline]
    pub const fn from_bits(bits: u16) -> Self {
        Self {
            parity_error: (bits & 0b100) != 0,
            command_invalid: (bits & 0b010) != 0,
            framing_error: (bits & 0b001) != 0,
        }
    }

    /// Returns `true` when any error flag is set.
    #[inline]
    pub const fn any(self) -> bool {
        self.parity_error || self.command_invalid || self.framing_error
    }
}

/// Magnetic-field interpretation of the diagnostics register.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum MagneticFieldStrength {
    /// The reported field window looks nominal.
    Nominal,
    /// The magnet field is stronger than expected.
    TooStrong,
    /// The magnet field is weaker than expected.
    TooWeak,
    /// Conflicting status bits were reported.
    Indeterminate,
}

/// Decoded `0x3FFD` diagnostics / AGC register content.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Diagnostics {
    /// Automatic gain control value.
    pub agc: u8,
    /// Compensation range indicates the field is too strong.
    pub compensation_too_high: bool,
    /// Compensation range indicates the field is too weak.
    pub compensation_too_low: bool,
    /// CORDIC overflow / tracking overflow condition.
    pub cordic_overflow: bool,
    /// Offset compensation finished and tracking is established.
    pub offset_compensation_finished: bool,
}

impl Diagnostics {
    /// Decodes a diagnostics register payload.
    #[inline]
    pub const fn from_bits(bits: u16) -> Self {
        Self {
            agc: (bits & 0x00FF) as u8,
            compensation_too_high: (bits & (1 << 11)) != 0,
            compensation_too_low: (bits & (1 << 10)) != 0,
            cordic_overflow: (bits & (1 << 9)) != 0,
            offset_compensation_finished: (bits & (1 << 8)) != 0,
        }
    }

    /// Interprets the magnetic-field strength diagnostics.
    #[inline]
    pub const fn magnetic_field_strength(self) -> MagneticFieldStrength {
        match (self.compensation_too_high, self.compensation_too_low) {
            (false, false) => MagneticFieldStrength::Nominal,
            (true, false) => MagneticFieldStrength::TooStrong,
            (false, true) => MagneticFieldStrength::TooWeak,
            (true, true) => MagneticFieldStrength::Indeterminate,
        }
    }

    /// Returns `true` when the tracking path looks usable.
    #[inline]
    pub const fn tracking_valid(self) -> bool {
        self.offset_compensation_finished && !self.cordic_overflow
    }
}

/// Protocol-level errors that can be detected from a received AS5048A frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProtocolError {
    /// Response parity was invalid.
    ResponseParity,
    /// The AS5048A asserted the response error flag.
    ErrorFlagSet,
}

impl fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ResponseParity => f.write_str("as5048a response parity error"),
            Self::ErrorFlagSet => f.write_str("as5048a error flag set"),
        }
    }
}

impl core::error::Error for ProtocolError {}

/// Driver error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<SpiError> {
    /// Underlying SPI-device failure.
    Spi(SpiError),
    /// Response parity was invalid.
    ResponseParity,
    /// The AS5048A asserted the response error flag.
    ErrorFlagSet,
}

impl<SpiError> fmt::Display for Error<SpiError>
where
    SpiError: fmt::Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Spi(error) => write!(f, "spi transaction failed: {error}"),
            Self::ResponseParity => f.write_str("as5048a response parity error"),
            Self::ErrorFlagSet => f.write_str("as5048a error flag set"),
        }
    }
}

impl<SpiError> core::error::Error for Error<SpiError> where SpiError: core::error::Error {}

/// AS5048A SPI driver with optional software zero offset.
#[derive(Debug)]
pub struct As5048a<SPI> {
    spi: SPI,
    zero_offset: u16,
}

impl<SPI> As5048a<SPI> {
    /// Creates a driver with zero software offset.
    #[inline]
    pub const fn new(spi: SPI) -> Self {
        Self {
            spi,
            zero_offset: 0,
        }
    }

    /// Creates a driver with a software zero offset in raw 14-bit counts.
    #[inline]
    pub const fn with_zero_offset(spi: SPI, zero_offset: u16) -> Self {
        Self {
            spi,
            zero_offset: zero_offset & DATA_MASK,
        }
    }

    /// Returns the configured software zero offset in raw 14-bit counts.
    #[inline]
    pub const fn zero_offset_raw(&self) -> u16 {
        self.zero_offset
    }

    /// Updates the software zero offset in raw 14-bit counts.
    #[inline]
    pub fn set_zero_offset_raw(&mut self, zero_offset: u16) {
        self.zero_offset = zero_offset & DATA_MASK;
    }

    /// Releases the underlying SPI device.
    #[inline]
    pub fn release(self) -> SPI {
        self.spi
    }
}

/// Returns the framed command used to read and clear the AS5048A error register.
#[inline]
pub fn clear_error_flags_command_frame() -> u16 {
    read_command_frame(Register::ClearErrorFlag.address())
}

/// Returns the framed command used to read the corrected angle register.
#[inline]
pub fn angle_read_command_frame() -> u16 {
    read_command_frame(Register::Angle.address())
}

/// Returns the framed `NOP` command used to fetch the pipelined response.
#[inline]
pub fn nop_command_frame() -> u16 {
    nop_frame()
}

/// Parses a clear-error response frame.
#[inline]
pub fn parse_clear_error_response(response: u16) -> Result<ErrorFlags, ProtocolError> {
    parse_protocol_response(response, false).map(ErrorFlags::from_bits)
}

/// Parses an angle response frame and applies the provided software zero offset.
#[inline]
pub fn parse_angle_response(response: u16, zero_offset: u16) -> Result<Angle, ProtocolError> {
    parse_protocol_response(response, true)
        .map(|raw| apply_zero_offset(raw, zero_offset))
        .map(raw_to_angle)
}

impl<SPI> As5048a<SPI>
where
    SPI: SpiDevice,
{
    /// Reads and clears the AS5048A error register.
    pub fn clear_error_flags(&mut self) -> Result<ErrorFlags, Error<SPI::Error>> {
        self.read_register_impl(Register::ClearErrorFlag, false)
            .map(ErrorFlags::from_bits)
    }

    /// Reads the corrected angle register in raw 14-bit counts with software
    /// zero offset applied.
    pub fn read_angle_raw(&mut self) -> Result<u16, Error<SPI::Error>> {
        self.read_angle_raw_unadjusted()
            .map(|raw| apply_zero_offset(raw, self.zero_offset))
    }

    /// Reads the corrected angle register in raw 14-bit counts without software
    /// zero offset applied.
    pub fn read_angle_raw_unadjusted(&mut self) -> Result<u16, Error<SPI::Error>> {
        self.read_register(Register::Angle)
    }

    /// Reads the corrected angle register and converts it to a wrapped
    /// [`Angle`] with software zero offset applied.
    pub fn read_angle(&mut self) -> Result<Angle, Error<SPI::Error>> {
        self.read_angle_raw().map(raw_to_angle)
    }

    /// Reads the magnitude register.
    pub fn read_magnitude(&mut self) -> Result<u16, Error<SPI::Error>> {
        self.read_register(Register::Magnitude)
    }

    /// Reads and decodes the diagnostics / AGC register.
    pub fn read_diagnostics(&mut self) -> Result<Diagnostics, Error<SPI::Error>> {
        self.read_register(Register::DiagnosticsAgc)
            .map(Diagnostics::from_bits)
    }

    /// Reads a register with the AS5048A two-transfer pipelined read sequence.
    ///
    /// This helper checks the `EF` response bit by default. When it returns
    /// [`Error::ErrorFlagSet`], use [`As5048a::clear_error_flags`] to decode and
    /// clear the latched error source.
    fn read_register(&mut self, register: Register) -> Result<u16, Error<SPI::Error>> {
        self.read_register_impl(register, true)
    }

    fn read_register_impl(
        &mut self,
        register: Register,
        check_error_flag: bool,
    ) -> Result<u16, Error<SPI::Error>> {
        let _ = self.transfer_frame(read_command_frame(register.address()))?;
        let response = self.transfer_frame(nop_frame())?;
        parse_response(response, check_error_flag)
    }

    fn transfer_frame(&mut self, frame: u16) -> Result<u16, Error<SPI::Error>> {
        let mut bytes = frame.to_be_bytes();
        self.spi.transfer_in_place(&mut bytes).map_err(Error::Spi)?;
        Ok(u16::from_be_bytes(bytes))
    }
}

#[cfg(feature = "async")]
impl<SPI> As5048a<SPI>
where
    SPI: AsyncSpiDevice,
{
    /// Reads and clears the AS5048A error register using async SPI.
    pub async fn clear_error_flags_async(&mut self) -> Result<ErrorFlags, Error<SPI::Error>> {
        self.read_register_impl_async(Register::ClearErrorFlag, false)
            .await
            .map(ErrorFlags::from_bits)
    }

    /// Reads the corrected angle register in raw 14-bit counts with software
    /// zero offset applied using async SPI.
    pub async fn read_angle_raw_async(&mut self) -> Result<u16, Error<SPI::Error>> {
        self.read_angle_raw_unadjusted_async()
            .await
            .map(|raw| apply_zero_offset(raw, self.zero_offset))
    }

    /// Reads the corrected angle register in raw 14-bit counts without software
    /// zero offset applied using async SPI.
    pub async fn read_angle_raw_unadjusted_async(&mut self) -> Result<u16, Error<SPI::Error>> {
        self.read_register_async(Register::Angle).await
    }

    /// Reads the corrected angle register and converts it to a wrapped
    /// [`Angle`] with software zero offset applied using async SPI.
    pub async fn read_angle_async(&mut self) -> Result<Angle, Error<SPI::Error>> {
        self.read_angle_raw_async().await.map(raw_to_angle)
    }

    /// Reads the magnitude register using async SPI.
    pub async fn read_magnitude_async(&mut self) -> Result<u16, Error<SPI::Error>> {
        self.read_register_async(Register::Magnitude).await
    }

    /// Reads and decodes the diagnostics / AGC register using async SPI.
    pub async fn read_diagnostics_async(&mut self) -> Result<Diagnostics, Error<SPI::Error>> {
        self.read_register_async(Register::DiagnosticsAgc)
            .await
            .map(Diagnostics::from_bits)
    }

    async fn read_register_async(&mut self, register: Register) -> Result<u16, Error<SPI::Error>> {
        self.read_register_impl_async(register, true).await
    }

    async fn read_register_impl_async(
        &mut self,
        register: Register,
        check_error_flag: bool,
    ) -> Result<u16, Error<SPI::Error>> {
        let _ = self
            .transfer_frame_async(read_command_frame(register.address()))
            .await?;
        let response = self.transfer_frame_async(nop_frame()).await?;
        parse_response(response, check_error_flag)
    }

    async fn transfer_frame_async(&mut self, frame: u16) -> Result<u16, Error<SPI::Error>> {
        let mut bytes = frame.to_be_bytes();
        self.spi
            .transfer_in_place(&mut bytes)
            .await
            .map_err(Error::Spi)?;
        Ok(u16::from_be_bytes(bytes))
    }
}

/// Converts a raw 14-bit angle count to degrees.
#[inline]
pub fn raw_to_degrees(raw: u16) -> f32 {
    (raw & DATA_MASK) as f32 * DEGREES_PER_COUNT
}

/// Converts a raw 14-bit angle count to radians in `[0, 2pi)`.
#[inline]
pub fn raw_to_radians(raw: u16) -> f32 {
    (raw & DATA_MASK) as f32 * RADIANS_PER_COUNT
}

/// Converts a raw 14-bit angle count to a wrapped [`Angle`].
#[inline]
pub fn raw_to_angle(raw: u16) -> Angle {
    Angle::from_raw(raw)
}

/// Fixed-period Fluxkit HAL adapter for an AS5048A angle path.
///
/// This adapter reads angle samples from [`As5048a`] and estimates velocity
/// from consecutive wrapped-angle samples using the configured fixed sample
/// period. The first sample reports zero velocity.
#[cfg(feature = "fluxkit")]
#[derive(Debug)]
pub struct FluxkitSensor<SPI> {
    driver: As5048a<SPI>,
    sample_period: Seconds,
    last_angle: Option<Angle>,
}

#[cfg(feature = "fluxkit")]
impl<SPI> FluxkitSensor<SPI> {
    /// Creates a Fluxkit HAL adapter around an AS5048A driver.
    #[inline]
    pub const fn new(driver: As5048a<SPI>, sample_period: Seconds) -> Self {
        Self {
            driver,
            sample_period,
            last_angle: None,
        }
    }

    /// Returns the configured fixed sample period used for velocity estimation.
    #[inline]
    pub const fn sample_period(&self) -> Seconds {
        self.sample_period
    }

    /// Returns a shared reference to the underlying AS5048A driver.
    #[inline]
    pub const fn driver(&self) -> &As5048a<SPI> {
        &self.driver
    }

    /// Returns a mutable reference to the underlying AS5048A driver.
    #[inline]
    pub fn driver_mut(&mut self) -> &mut As5048a<SPI> {
        &mut self.driver
    }

    /// Releases the underlying AS5048A driver.
    #[inline]
    pub fn into_inner(self) -> As5048a<SPI> {
        self.driver
    }
}

#[cfg(feature = "fluxkit")]
impl<SPI> FluxkitSensor<SPI>
where
    SPI: SpiDevice,
{
    fn read_fluxkit_angle_velocity(
        &mut self,
    ) -> Result<(MechanicalAngle, RadPerSec), Error<SPI::Error>> {
        let angle = self.driver.read_angle()?;
        let velocity = estimate_velocity(self.last_angle, angle, self.sample_period);
        self.last_angle = Some(angle);
        Ok((to_fluxkit_mechanical_angle(angle), velocity))
    }
}

#[cfg(feature = "fluxkit")]
impl<SPI> fluxkit_hal::RotorSensor for FluxkitSensor<SPI>
where
    SPI: SpiDevice,
    SPI::Error: core::error::Error,
{
    type Error = Error<SPI::Error>;

    fn read_rotor(&mut self) -> Result<fluxkit_hal::RotorReading, Self::Error> {
        let (mechanical_angle, mechanical_velocity) = self.read_fluxkit_angle_velocity()?;
        Ok(fluxkit_hal::RotorReading {
            mechanical_angle,
            mechanical_velocity,
        })
    }
}

#[cfg(feature = "fluxkit")]
impl<SPI> fluxkit_hal::OutputSensor for FluxkitSensor<SPI>
where
    SPI: SpiDevice,
    SPI::Error: core::error::Error,
{
    type Error = Error<SPI::Error>;

    fn read_output(&mut self) -> Result<fluxkit_hal::OutputReading, Self::Error> {
        let (mechanical_angle, mechanical_velocity) = self.read_fluxkit_angle_velocity()?;
        Ok(fluxkit_hal::OutputReading {
            mechanical_angle,
            mechanical_velocity,
        })
    }
}

#[inline]
fn wrap_radians(value: f32) -> f32 {
    let mut wrapped = value % core::f32::consts::TAU;
    if wrapped >= core::f32::consts::PI {
        wrapped -= core::f32::consts::TAU;
    }
    if wrapped < -core::f32::consts::PI {
        wrapped += core::f32::consts::TAU;
    }
    wrapped
}

#[inline]
const fn apply_zero_offset(raw: u16, zero_offset: u16) -> u16 {
    raw.wrapping_sub(zero_offset) & DATA_MASK
}

#[cfg(feature = "fluxkit")]
#[inline]
fn estimate_velocity(
    previous_angle: Option<Angle>,
    angle: Angle,
    sample_period: Seconds,
) -> RadPerSec {
    let dt = sample_period.get();
    if !dt.is_finite() || dt <= 0.0 {
        return RadPerSec::ZERO;
    }

    let Some(previous_angle) = previous_angle else {
        return RadPerSec::ZERO;
    };

    RadPerSec::new(shortest_angle_delta(previous_angle.radians(), angle.radians()) / dt)
}

#[cfg(feature = "fluxkit")]
#[inline]
fn to_fluxkit_mechanical_angle(angle: Angle) -> MechanicalAngle {
    MechanicalAngle::new(angle.radians())
}

#[inline]
const fn nop_frame() -> u16 {
    0x0000
}

#[inline]
const fn read_command_frame(address: u16) -> u16 {
    add_even_parity(READ_MASK | (address & DATA_MASK))
}

#[inline]
const fn add_even_parity(frame_without_parity: u16) -> u16 {
    if (frame_without_parity & !PARITY_MASK).count_ones() % 2 == 0 {
        frame_without_parity & !PARITY_MASK
    } else {
        frame_without_parity | PARITY_MASK
    }
}

#[inline]
fn parse_response<SpiError>(frame: u16, check_error_flag: bool) -> Result<u16, Error<SpiError>> {
    parse_protocol_response(frame, check_error_flag).map_err(|error| match error {
        ProtocolError::ResponseParity => Error::ResponseParity,
        ProtocolError::ErrorFlagSet => Error::ErrorFlagSet,
    })
}

#[inline]
fn parse_protocol_response(frame: u16, check_error_flag: bool) -> Result<u16, ProtocolError> {
    if frame.count_ones() % 2 != 0 {
        return Err(ProtocolError::ResponseParity);
    }
    if check_error_flag && (frame & EF_MASK) != 0 {
        return Err(ProtocolError::ErrorFlagSet);
    }
    Ok(frame & DATA_MASK)
}

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use embedded_hal::spi::{ErrorType, Operation, SpiDevice};

    use super::{
        Angle, As5048a, DATA_MASK, Diagnostics, Error, ErrorFlags, MagneticFieldStrength, Register,
        add_even_parity, nop_frame, raw_to_angle, raw_to_degrees, raw_to_radians,
        read_command_frame,
    };

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct MockSpiError;

    impl embedded_hal::spi::Error for MockSpiError {
        fn kind(&self) -> embedded_hal::spi::ErrorKind {
            embedded_hal::spi::ErrorKind::Other
        }
    }

    impl core::fmt::Display for MockSpiError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.write_str("mock spi error")
        }
    }

    impl core::error::Error for MockSpiError {}

    #[derive(Debug)]
    struct MockSpiDevice {
        expected_frames: VecDeque<u16>,
        responses: VecDeque<u16>,
    }

    impl MockSpiDevice {
        fn new(expected_frames: &[u16], responses: &[u16]) -> Self {
            Self {
                expected_frames: expected_frames.iter().copied().collect(),
                responses: responses.iter().copied().collect(),
            }
        }

        fn assert_consumed(&self) {
            assert!(self.expected_frames.is_empty());
            assert!(self.responses.is_empty());
        }
    }

    impl ErrorType for MockSpiDevice {
        type Error = MockSpiError;
    }

    impl SpiDevice for MockSpiDevice {
        fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
            assert_eq!(operations.len(), 1);

            match &mut operations[0] {
                Operation::TransferInPlace(buffer) => {
                    assert_eq!(buffer.len(), 2);
                    let sent = u16::from_be_bytes([buffer[0], buffer[1]]);
                    assert_eq!(Some(sent), self.expected_frames.pop_front());
                    let response = self.responses.pop_front().expect("missing response");
                    buffer.copy_from_slice(&response.to_be_bytes());
                    Ok(())
                }
                _ => panic!("unexpected SPI operation"),
            }
        }
    }

    fn response_frame(data: u16, error_flag: bool) -> u16 {
        add_even_parity((data & DATA_MASK) | if error_flag { 0x4000 } else { 0 })
    }

    #[test]
    fn read_command_frame_sets_even_parity() {
        assert_eq!(read_command_frame(Register::Angle as u16), 0xFFFF);
        assert_eq!(nop_frame(), 0x0000);
    }

    #[test]
    fn reads_angle_with_pipelined_nop_followup() {
        let mut driver = As5048a::new(MockSpiDevice::new(
            &[read_command_frame(Register::Angle as u16), nop_frame()],
            &[0x0000, response_frame(0x1234, false)],
        ));

        let raw = driver.read_angle_raw().unwrap();
        assert_eq!(raw, 0x1234);

        driver.release().assert_consumed();
    }

    #[test]
    fn applies_software_zero_offset_to_angle_reads() {
        let mut driver = As5048a::with_zero_offset(
            MockSpiDevice::new(
                &[
                    read_command_frame(Register::Angle as u16),
                    nop_frame(),
                    read_command_frame(Register::Angle as u16),
                    nop_frame(),
                ],
                &[
                    0x0000,
                    response_frame(0x2000, false),
                    0x0000,
                    response_frame(0x2000, false),
                ],
            ),
            0x1000,
        );

        let raw = driver.read_angle_raw().unwrap();
        let angle = driver.read_angle().unwrap();
        assert_eq!(raw, 0x1000);
        assert!(
            (angle.radians() - core::f32::consts::FRAC_PI_2).abs() < 1.0e-6,
            "expected pi/2, got {}",
            angle.radians()
        );

        driver.release().assert_consumed();
    }

    #[test]
    fn converts_raw_angle_to_local_types() {
        assert!((raw_to_degrees(0x2000) - 180.0).abs() < 1.0e-6);
        assert!((raw_to_radians(0x2000) - core::f32::consts::PI).abs() < 1.0e-6);
        assert!((raw_to_angle(0x1000).radians() - core::f32::consts::FRAC_PI_2).abs() < 1.0e-6);
    }

    #[test]
    fn angle_wraps_without_fluxkit_math() {
        let angle = Angle::from_radians(3.5 * core::f32::consts::PI);
        assert!((angle.radians() + core::f32::consts::FRAC_PI_2).abs() < 1.0e-6);
    }

    #[test]
    fn reports_error_flag_and_allows_followup_clear() {
        let mut driver = As5048a::new(MockSpiDevice::new(
            &[
                read_command_frame(Register::Angle as u16),
                nop_frame(),
                read_command_frame(Register::ClearErrorFlag as u16),
                nop_frame(),
            ],
            &[
                0x0000,
                response_frame(0x0000, true),
                0x0000,
                response_frame(0b101, true),
            ],
        ));

        assert_eq!(driver.read_angle_raw(), Err(Error::ErrorFlagSet));

        let flags = driver.clear_error_flags().unwrap();
        assert_eq!(
            flags,
            ErrorFlags {
                parity_error: true,
                command_invalid: false,
                framing_error: true,
            }
        );

        driver.release().assert_consumed();
    }

    #[test]
    fn rejects_bad_response_parity() {
        let mut driver = As5048a::new(MockSpiDevice::new(
            &[read_command_frame(Register::Magnitude as u16), nop_frame()],
            &[0x0000, response_frame(0x0003, false) ^ 0x8000],
        ));

        assert_eq!(driver.read_magnitude(), Err(Error::ResponseParity));
    }

    #[test]
    fn decodes_diagnostics_register() {
        let bits = (1 << 11) | (1 << 9) | (1 << 8) | 0x5A;
        let diagnostics = Diagnostics::from_bits(bits);

        assert_eq!(diagnostics.agc, 0x5A);
        assert_eq!(
            diagnostics.magnetic_field_strength(),
            MagneticFieldStrength::TooStrong
        );
        assert!(diagnostics.offset_compensation_finished);
        assert!(diagnostics.cordic_overflow);
        assert!(!diagnostics.tracking_valid());
    }

    #[cfg(feature = "fluxkit")]
    #[test]
    fn fluxkit_sensor_estimates_velocity_for_rotor_reads() {
        use crate::FluxkitSensor;
        use fluxkit_hal::RotorSensor;
        use fluxkit_math::{RadPerSec, Seconds};

        let driver = As5048a::new(MockSpiDevice::new(
            &[
                read_command_frame(Register::Angle as u16),
                nop_frame(),
                read_command_frame(Register::Angle as u16),
                nop_frame(),
            ],
            &[
                0x0000,
                response_frame(0x1000, false),
                0x0000,
                response_frame(0x1800, false),
            ],
        ));
        let mut sensor = FluxkitSensor::new(driver, Seconds::new(0.1));

        let first = sensor.read_rotor().unwrap();
        let second = sensor.read_rotor().unwrap();

        assert!((first.mechanical_angle.get() - core::f32::consts::FRAC_PI_2).abs() < 1.0e-6);
        assert_eq!(first.mechanical_velocity, RadPerSec::ZERO);
        assert!(
            (second.mechanical_velocity.get() - (core::f32::consts::FRAC_PI_4 / 0.1)).abs()
                < 1.0e-5
        );

        sensor.into_inner().release().assert_consumed();
    }

    #[cfg(feature = "fluxkit")]
    #[test]
    fn fluxkit_sensor_implements_output_sensor() {
        use crate::FluxkitSensor;
        use fluxkit_hal::OutputSensor;
        use fluxkit_math::{RadPerSec, Seconds};

        let driver = As5048a::new(MockSpiDevice::new(
            &[read_command_frame(Register::Angle as u16), nop_frame()],
            &[0x0000, response_frame(0x0400, false)],
        ));
        let mut sensor = FluxkitSensor::new(driver, Seconds::new(0.001));

        let reading = sensor.read_output().unwrap();
        assert!((reading.mechanical_angle.get() - (core::f32::consts::PI / 8.0)).abs() < 1.0e-6);
        assert_eq!(reading.mechanical_velocity, RadPerSec::ZERO);
    }
}
