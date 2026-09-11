//!
//!
//!

pub mod dng;
pub mod isa;
pub mod lan;
pub mod pcc;
pub mod pci;
pub mod usb;

use crate::bus::Bus;
use crate::error::{CanError, CanOkError};
use crate::peak_lib;
use crate::peak_can;

use std::ops::Deref;

pub const STANDARD_MASK: u32 = 0x07_FF;
pub const EXTENDED_MASK: u32 = 0x1F_FF_FF_FF;

#[derive(Debug, PartialEq)]
pub enum MessageType {
    Standard,
    Extended,
}

#[derive(Debug, PartialEq)]
pub enum FrameConstructionError {
    TooMuchData,
    CanIdMessageTypeMismatch,
}

#[derive(Debug, Copy, Clone)]
pub struct CanFrame {
    frame: peak_can::TPEAKMsg,
}

impl CanFrame {
    const MAX_DLC: usize = 8;

    pub fn new(
        can_id: u32,
        msg_type: MessageType,
        data: &[u8],
    ) -> Result<CanFrame, FrameConstructionError> {
        if data.len() > Self::MAX_DLC {
            Err(FrameConstructionError::TooMuchData)
        } else {
            let mut frame_data: [u8; 8] = [0; 8];
            for (i, v) in data.into_iter().enumerate() {
                frame_data[i] = *v;
            }

            match msg_type {
                MessageType::Standard => Ok(CanFrame {
                    frame: peak_can::TPEAKMsg {
                        ID: can_id & STANDARD_MASK,
                        MSGTYPE: peak_can::PEAK_MESSAGE_STANDARD as u8,
                        LEN: data.len() as u8,
                        DATA: frame_data,
                    },
                }),
                MessageType::Extended => Ok(CanFrame {
                    frame: peak_can::TPEAKMsg {
                        ID: can_id & EXTENDED_MASK,
                        MSGTYPE: peak_can::PEAK_MESSAGE_EXTENDED as u8,
                        LEN: data.len() as u8,
                        DATA: frame_data,
                    },
                }),
            }
        }
    }

    pub fn is_standard_frame(&self) -> bool {
        // PEAK_MESSAGE_STANDARD flag is denoted as 0, so check for extended frame flag instead
        !self.is_extended_frame()
    }

    pub fn is_extended_frame(&self) -> bool {
        self.frame.MSGTYPE & peak_can::PEAK_MESSAGE_EXTENDED as u8 != 0
    }

    pub fn is_error_frame(&self) -> bool {
        self.frame.MSGTYPE & peak_can::PEAK_MESSAGE_ERRFRAME as u8 != 0
    }

    pub fn is_echo_frame(&self) -> bool {
        self.frame.MSGTYPE & peak_can::PEAK_MESSAGE_ECHO as u8 != 0
    }

    /// Whether this is a remote frame (`PCAN_MESSAGE_RTR`).
    ///
    /// A remote frame (CAN 2.0B §3.2.2) requests the data frame with this
    /// identifier; it carries no data of its own, so [`Self::data`] is not
    /// meaningful for one. Without this predicate a remote frame is
    /// indistinguishable from a data frame, since the other accessors answer
    /// the same way for both.
    ///
    /// Remote frames reach the receive queue only once they have been enabled
    /// with [`allow_rtr_frames`](crate::df::SetAllowRTRFrames::allow_rtr_frames).
    pub fn is_rtr_frame(&self) -> bool {
        self.frame.MSGTYPE & peak_can::PEAK_MESSAGE_RTR as u8 != 0
    }

    /// Whether this is a PCAN status message (`PCAN_MESSAGE_STATUS`).
    ///
    /// A status message is not a CAN frame: the driver injects it into the
    /// receive queue to report the controller's own state. It carries no
    /// identifier and no CAN payload, so [`Self::can_id`] answers 0 and
    /// [`Self::data`] is not meaningful -- read [`Self::status_bits`] instead.
    ///
    /// Without this predicate a status message is indistinguishable from a data
    /// frame with identifier 0, which is a value a caller may well be using as a
    /// sentinel.
    ///
    /// Status messages reach the receive queue only once they have been enabled
    /// with [`allow_status_frames`](crate::df::SetAllowStatusFrames::allow_status_frames).
    pub fn is_status_frame(&self) -> bool {
        self.frame.MSGTYPE & peak_can::PEAK_MESSAGE_STATUS as u8 != 0
    }

    /// The status word carried by a status message, or `None` for any other
    /// frame.
    ///
    /// A status message stores a 32-bit `TPCANStatus` in its first four data
    /// bytes, most significant byte first. The value is comparable against the
    /// `PCAN_ERROR_*` constants and can be passed to
    /// [`CanError::try_from`](crate::error::CanError) to classify it.
    pub fn status_bits(&self) -> Option<u32> {
        if !self.is_status_frame() {
            return None;
        }

        // DATA is a fixed [u8; 8], so the first four bytes are always readable
        // regardless of what LEN reports.
        Some(u32::from_be_bytes([
            self.frame.DATA[0],
            self.frame.DATA[1],
            self.frame.DATA[2],
            self.frame.DATA[3],
        ]))
    }

    pub fn can_id(&self) -> u32 {
        if self.is_standard_frame() {
            self.frame.ID & STANDARD_MASK
        } else {
            self.frame.ID & EXTENDED_MASK
        }
    }

    pub fn dlc(&self) -> u8 {
        self.frame.LEN
    }

    pub fn data(&self) -> &[u8] {
        &self.frame.DATA[0..self.dlc() as usize]
    }

    pub fn mut_data(&mut self) -> &mut [u8] {
        let dlc = self.dlc();
        &mut self.frame.DATA[0..dlc as usize]
    }
}

impl Default for CanFrame {
    fn default() -> Self {
        CanFrame::new(0, MessageType::Standard, &[]).unwrap()
    }
}

impl PartialEq for CanFrame {
    fn eq(&self, other: &Self) -> bool {
        if self.frame.ID != other.frame.ID {
            return false;
        }

        if self.frame.LEN != other.frame.LEN {
            return false;
        }

        if self.frame.MSGTYPE != other.frame.MSGTYPE {
            return false;
        }

        if self.data() != other.data() {
            return false;
        }

        true
    }
}

#[derive(Debug, Copy, Clone)]
pub struct CanFdFrame {
    frame: peak_can::TPEAKMsgFD,
}

impl CanFdFrame {
    const MAX_DATA_LENGTH: usize = 64;

    pub fn new(
        can_id: u32,
        msg_type: MessageType,
        data: &[u8],
        fd: bool,
        brs: bool,
    ) -> Result<CanFdFrame, FrameConstructionError> {
        if data.len() > Self::MAX_DATA_LENGTH {
            Err(FrameConstructionError::TooMuchData)
        } else {
            let mut frame_data: [u8; Self::MAX_DATA_LENGTH] = [0; Self::MAX_DATA_LENGTH];
            for (i, v) in data.into_iter().enumerate() {
                frame_data[i] = *v;
            }

            match msg_type {
                MessageType::Standard => Ok(CanFdFrame {
                    frame: peak_can::TPEAKMsgFD {
                        ID: can_id & STANDARD_MASK,
                        MSGTYPE: peak_can::PEAK_MESSAGE_STANDARD as u8 | 
                            if fd { peak_can::PEAK_MESSAGE_FD as u8 } else { 0 } |
                            if brs { peak_can::PEAK_MESSAGE_BRS as u8 } else { 0 },
                        DLC: Self::calc_dlc(data.len()),
                        DATA: frame_data,
                    },
                }),
                MessageType::Extended => Ok(CanFdFrame {
                    frame: peak_can::TPEAKMsgFD {
                        ID: can_id & EXTENDED_MASK,
                        MSGTYPE: peak_can::PEAK_MESSAGE_EXTENDED as u8 |
                            if fd { peak_can::PEAK_MESSAGE_FD as u8 } else { 0 } |
                            if brs { peak_can::PEAK_MESSAGE_BRS as u8 } else { 0 },
                        DLC: Self::calc_dlc(data.len()),
                        DATA: frame_data,
                    },
                }),
            }
        }
    }

    pub fn is_standard_frame(&self) -> bool {
        self.frame.MSGTYPE & peak_can::PEAK_MESSAGE_STANDARD as u8 != 0
    }

    pub fn is_extended_frame(&self) -> bool {
        if self.frame.MSGTYPE & peak_can::PEAK_MESSAGE_EXTENDED as u8 != 0 {
            true
        } else {
            false
        }
    }
    
    pub fn is_error_frame(&self) -> bool {
        self.frame.MSGTYPE & peak_can::PEAK_MESSAGE_ERRFRAME as u8 != 0
    }

    pub fn is_echo_frame(&self) -> bool {
        self.frame.MSGTYPE & peak_can::PEAK_MESSAGE_ECHO as u8 != 0
    }

    pub fn is_fd_frame(&self) -> bool {
        self.frame.MSGTYPE & peak_can::PEAK_MESSAGE_FD as u8 != 0
    }

    pub fn can_id(&self) -> u32 {
        if self.is_standard_frame() {
            self.frame.ID & STANDARD_MASK
        } else {
            self.frame.ID & EXTENDED_MASK
        }
    }

    pub fn dlc(&self) -> u8 {
        self.frame.DLC
    }

    pub fn data(&self) -> &[u8] {
        &self.frame.DATA[0..self.len() as usize]
    }

    pub fn mut_data(&mut self) -> &mut [u8] {
        let len = self.len();
        &mut self.frame.DATA[0..len as usize]
    }

    fn calc_dlc(len: usize) -> u8 {
        match len {
            0..=8 => len as u8,
            9..=12 => 9,
            13..=16 => 10,
            17..=20 => 11,
            21..=24 => 12,
            25..=32 => 13,
            33..=48 => 14,
            49..=64 => 15,
            _ => 15, // Max DLC for CAN FD is 64 bytes
        }
    }

    pub fn len(&self) -> usize {
        match self.dlc() {
            0..=8 => self.dlc() as usize,
            9 => 12,
            10 => 16,
            11 => 20,
            12 => 24,
            13 => 32,
            14 => 48,
            15 => 64,
            _ => 64, // Max DLC for CAN FD is 64 bytes
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl Default for CanFdFrame {
    fn default() -> Self {
        CanFdFrame::new(0, MessageType::Standard, &[], false, false).unwrap()
    }
}

impl PartialEq for CanFdFrame {
    fn eq(&self, other: &Self) -> bool {
        if self.frame.ID != other.frame.ID {
            return false;
        }

        if self.frame.DLC != other.frame.DLC {
            return false;
        }

        if self.frame.MSGTYPE != other.frame.MSGTYPE {
            return false;
        }

        if self.data() != other.data() {
            return false;
        }

        true
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Timestamp {
    timestamp: peak_can::TPEAKTimestamp,
}

impl Deref for Timestamp {
    type Target = peak_can::TPEAKTimestamp;

    fn deref(&self) -> &Self::Target {
        &self.timestamp
    }
}

impl Default for Timestamp {
    fn default() -> Timestamp {
        Timestamp {
            timestamp: peak_can::TPEAKTimestamp {
                micros: 0,
                millis: 0,
                millis_overflow: 0,
            },
        }
    }
}

impl PartialEq for Timestamp {
    fn eq(&self, other: &Self) -> bool {
        if self.timestamp.micros != other.timestamp.micros {
            return false;
        }

        if self.timestamp.millis != other.timestamp.millis {
            return false;
        }

        if self.timestamp.millis_overflow != other.timestamp.millis_overflow {
            return false;
        }

        true
    }
}

#[derive(Debug, PartialEq)]
pub struct CanSocket {
    handle: u16,
}

impl CanSocket {
    pub fn open<T: Bus>(bus: T, baud: Baudrate) -> Result<CanSocket, CanError> {
        let handle = bus.channel();
        let code = unsafe { peak_lib()?.CAN_Initialize(handle, baud.into(), 0, 0, 0) };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(CanSocket { handle }),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

trait HasRecvCan {}

pub trait RecvCan {
    fn recv(&self) -> Result<(CanFrame, Timestamp), CanError>;
    fn recv_frame(&self) -> Result<CanFrame, CanError>;
}

trait HasRecvCanFd {}

pub trait RecvCanFd {
    fn recv_fd(&self) -> Result<(CanFdFrame, u64), CanError>;
    fn recv_fd_frame(&self) -> Result<CanFdFrame, CanError>;
}

trait HasSendCan {}

pub trait SendCan {
    fn send(&self, frame: CanFrame) -> Result<(), CanError>;
}

trait HasSendCanFd {}

pub trait SendCanFd {
    fn send_fd(&self, frame: CanFdFrame) -> Result<(), CanError>;
}

trait Socket {
    fn handle(&self) -> u16;
}

/* Baudrate */

#[derive(Debug, PartialEq)]
pub enum Baudrate {
    Baud1M,
    Baud800K,
    Baud500K,
    Baud250K,
    Baud125K,
    Baud100K,
    Baud95K,
    Baud83K,
    Baud50K,
    Baud47K,
    Baud33K,
    Baud20K,
    Baud10K,
    Baud5K,
}

impl From<Baudrate> for u16 {
    fn from(value: Baudrate) -> Self {
        let ret = match value {
            Baudrate::Baud1M => peak_can::PEAK_BAUD_1M,
            Baudrate::Baud800K => peak_can::PEAK_BAUD_800K,
            Baudrate::Baud500K => peak_can::PEAK_BAUD_500K,
            Baudrate::Baud250K => peak_can::PEAK_BAUD_250K,
            Baudrate::Baud125K => peak_can::PEAK_BAUD_125K,
            Baudrate::Baud100K => peak_can::PEAK_BAUD_100K,
            Baudrate::Baud95K => peak_can::PEAK_BAUD_95K,
            Baudrate::Baud83K => peak_can::PEAK_BAUD_83K,
            Baudrate::Baud50K => peak_can::PEAK_BAUD_50K,
            Baudrate::Baud47K => peak_can::PEAK_BAUD_47K,
            Baudrate::Baud33K => peak_can::PEAK_BAUD_33K,
            Baudrate::Baud20K => peak_can::PEAK_BAUD_20K,
            Baudrate::Baud10K => peak_can::PEAK_BAUD_10K,
            Baudrate::Baud5K => peak_can::PEAK_BAUD_5K,
        } as u16;
        ret
    }
}

/// Hardware-specific timing parameter boundaries for classical CAN 2.0 bit timing.
///
/// These boundaries define the valid ranges for CAN bit timing parameters and are
/// hardware-specific to PEAK-System CAN devices. The values constrain the configuration
/// of the CAN controller's bit timing to ensure it operates within hardware capabilities.
///
/// # Timing Parameters
///
/// - **prescaler**: Clock prescaler that divides the CAN controller's base clock
/// - **sjw** (Synchronization Jump Width): Maximum time by which the bit sampling point
///   can be shifted to resynchronize with the bus
/// - **tseg1** (Time Segment 1): Duration before the sample point, includes propagation
///   delay and phase segment 1
/// - **tseg2** (Time Segment 2): Duration after the sample point (phase segment 2)
///
/// # Usage
///
/// These boundaries are used internally by [`CanBitTiming::new()`] to validate timing
/// parameters. Users should reference [`CAN_TIMING_BOUNDARIES`] to ensure their custom
/// timing configurations fall within acceptable ranges.
/// ```
pub struct TimingBoundaries {
    pub prescaler_min: u16,
    pub prescaler_max: u16,
    pub sjw_min: u8,
    pub sjw_max: u8,
    pub tseg1_min: u8,
    pub tseg1_max: u8,
    pub tseg2_min: u8,
    pub tseg2_max: u8,
}

/// Hardware-specific timing parameter boundaries for CAN FD bit timing.
///
/// These boundaries define the valid ranges for CAN FD bit timing parameters and are
/// hardware-specific to PEAK-System CAN FD devices. CAN FD supports dual bit rates:
/// a nominal (arbitration) bit rate and a faster data bit rate, each with their own
/// timing parameters.
///
/// # Timing Parameters
///
/// ## Nominal (Arbitration) Phase
/// Used during arbitration and control fields:
/// - **nom_prescaler**: Nominal phase clock prescaler
/// - **nom_sjw**: Nominal phase synchronization jump width
/// - **nom_tseg1**: Nominal phase time segment 1
/// - **nom_tseg2**: Nominal phase time segment 2
///
/// ## Data Phase
/// Used during the data field for higher throughput:
/// - **data_prescaler**: Data phase clock prescaler
/// - **data_sjw**: Data phase synchronization jump width
/// - **data_tseg1**: Data phase time segment 1
/// - **data_tseg2**: Data phase time segment 2
///
/// # Usage
///
/// These boundaries are used internally by [`CanFdBitTiming::new()`] to validate timing
/// parameters. Users should reference [`CANFD_TIMING_BOUNDARIES`] to ensure their custom
/// CAN FD timing configurations fall within acceptable ranges for PEAK hardware.
///
/// # Note
///
/// The data phase bit rate must be equal to or higher than the nominal bit rate. Typical
/// configurations use 500 kbit/s for nominal and 2-8 Mbit/s for data phases.
pub struct FdTimingBoundaries {
    pub nom_prescaler_min: u16,
    pub nom_prescaler_max: u16,
    pub nom_sjw_min: u8,
    pub nom_sjw_max: u8,
    pub nom_tseg1_min: u16,
    pub nom_tseg1_max: u16,
    pub nom_tseg2_min: u8,
    pub nom_tseg2_max: u8,
    pub data_prescaler_min: u16,
    pub data_prescaler_max: u16,
    pub data_sjw_min: u8,
    pub data_sjw_max: u8,
    pub data_tseg1_min: u8,
    pub data_tseg1_max: u8,
    pub data_tseg2_min: u8,
    pub data_tseg2_max: u8,
}

/// Hardware timing parameter boundaries for PEAK-System classical CAN 2.0 devices.
///
/// This constant defines the valid ranges for bit timing parameters on PEAK CAN hardware.
/// These limits are enforced when creating [`CanBitTiming`] instances to ensure
/// configurations are compatible with the hardware.
///
/// # Values
///
/// - Prescaler: 1-64
/// - SJW: 1-4
/// - TSEG1: 1-16
/// - TSEG2: 1-8
///
/// # See Also
///
/// - [`TimingBoundaries`] - Structure definition and detailed parameter descriptions
/// - [`CanBitTiming::new()`] - Uses these boundaries for validation
pub const CAN_TIMING_BOUNDARIES: TimingBoundaries = TimingBoundaries {
    prescaler_min: 1,
    prescaler_max: 64,
    sjw_min: 1,
    sjw_max: 4,
    tseg1_min: 1,
    tseg1_max: 16,
    tseg2_min: 1,
    tseg2_max: 8,
};

/// Hardware timing parameter boundaries for PEAK-System CAN FD devices.
///
/// This constant defines the valid ranges for CAN FD bit timing parameters on PEAK
/// hardware. These limits are enforced when creating [`CanFdBitTiming`] instances to
/// ensure configurations are compatible with the hardware's dual bit rate capabilities.
///
/// # Values
///
/// ## Nominal (Arbitration) Phase
/// - Prescaler: 1-1024
/// - SJW: 1-128
/// - TSEG1: 1-256
/// - TSEG2: 1-128
///
/// ## Data Phase
/// - Prescaler: 1-1024
/// - SJW: 1-16
/// - TSEG1: 1-32
/// - TSEG2: 1-16
///
/// # See Also
///
/// - [`FdTimingBoundaries`] - Structure definition and detailed parameter descriptions
/// - [`CanFdBitTiming::new()`] - Uses these boundaries for validation
pub const CANFD_TIMING_BOUNDARIES: FdTimingBoundaries = FdTimingBoundaries {
    nom_prescaler_min: 1,
    nom_prescaler_max: 1024,
    nom_sjw_min: 1,
    nom_sjw_max: 128,
    nom_tseg1_min: 1,
    nom_tseg1_max: 256,
    nom_tseg2_min: 1,
    nom_tseg2_max: 128,
    data_prescaler_min: 1,
    data_prescaler_max: 1024,
    data_sjw_min: 1,
    data_sjw_max: 16,
    data_tseg1_min: 1,
    data_tseg1_max: 32,
    data_tseg2_min: 1,
    data_tseg2_max: 16,
};

pub struct CanBitTiming {
    pub prescaler: u16,
    pub sjw: u8,
    pub tseg1: u8,
    pub tseg2: u8,
}

impl CanBitTiming {
    pub fn new(prescaler: u16, sjw: u8, tseg1: u8, tseg2: u8) -> Result<Self, Box<dyn std::error::Error>> {
        let timing = CanBitTiming {
            prescaler,
            sjw,
            tseg1,
            tseg2,
        };

        if Self::validate(&timing) {
            Ok(timing)
        } else {
            Err("Timing parameters are out of bounds".into())
        }
    }

    fn validate(timing: &CanBitTiming) -> bool {
        if timing.prescaler < CAN_TIMING_BOUNDARIES.prescaler_min
            || timing.prescaler > CAN_TIMING_BOUNDARIES.prescaler_max
        {
            return false;
        }
        if timing.sjw < CAN_TIMING_BOUNDARIES.sjw_min
            || timing.sjw > CAN_TIMING_BOUNDARIES.sjw_max
        {
            return false;
        }
        if timing.tseg1 < CAN_TIMING_BOUNDARIES.tseg1_min
            || timing.tseg1 > CAN_TIMING_BOUNDARIES.tseg1_max
        {
            return false;
        }
        if timing.tseg2 < CAN_TIMING_BOUNDARIES.tseg2_min
            || timing.tseg2 > CAN_TIMING_BOUNDARIES.tseg2_max
        {
            return false;
        }
        true
    }
}

pub struct CanFdBitTiming {
    pub nom_prescaler: u16,
    pub nom_sjw: u8,
    pub nom_tseg1: u16,
    pub nom_tseg2: u8,
    pub data_prescaler: u16,
    pub data_sjw: u8,
    pub data_tseg1: u8,
    pub data_tseg2: u8,
}

impl CanFdBitTiming {
    pub fn new(nom_prescaler: u16, nom_sjw: u8, nom_tseg1: u16, nom_tseg2: u8, data_prescaler: u16, data_sjw: u8, data_tseg1: u8, data_tseg2: u8) -> Result<Self, Box<dyn std::error::Error>> {
        let timing = CanFdBitTiming {
            nom_prescaler,
            nom_sjw,
            nom_tseg1,
            nom_tseg2,
            data_prescaler,
            data_sjw,
            data_tseg1,
            data_tseg2,
        };

        if Self::validate(&timing) {
            Ok(timing)
        } else {
            Err("Timing parameters are out of bounds".into())
        }
    }

    fn validate(timing: &CanFdBitTiming) -> bool {
        if timing.nom_prescaler < CANFD_TIMING_BOUNDARIES.nom_prescaler_min
            || timing.nom_prescaler > CANFD_TIMING_BOUNDARIES.nom_prescaler_max
        {
            return false;
        }
        if timing.nom_sjw < CANFD_TIMING_BOUNDARIES.nom_sjw_min
            || timing.nom_sjw > CANFD_TIMING_BOUNDARIES.nom_sjw_max
        {
            return false;
        }
        if timing.nom_tseg1 < CANFD_TIMING_BOUNDARIES.nom_tseg1_min
            || timing.nom_tseg1 > CANFD_TIMING_BOUNDARIES.nom_tseg1_max
        {
            return false;
        }
        if timing.nom_tseg2 < CANFD_TIMING_BOUNDARIES.nom_tseg2_min
            || timing.nom_tseg2 > CANFD_TIMING_BOUNDARIES.nom_tseg2_max
        {
            return false;
        }
        if timing.data_prescaler < CANFD_TIMING_BOUNDARIES.data_prescaler_min
            || timing.data_prescaler > CANFD_TIMING_BOUNDARIES.data_prescaler_max
        {
            return false;
        }
        if timing.data_sjw < CANFD_TIMING_BOUNDARIES.data_sjw_min
            || timing.data_sjw > CANFD_TIMING_BOUNDARIES.data_sjw_max
        {
            return false;
        }
        if timing.data_tseg1 < CANFD_TIMING_BOUNDARIES.data_tseg1_min
            || timing.data_tseg1 > CANFD_TIMING_BOUNDARIES.data_tseg1_max
        {
            return false;
        }
        if timing.data_tseg2 < CANFD_TIMING_BOUNDARIES.data_tseg2_min
            || timing.data_tseg2 > CANFD_TIMING_BOUNDARIES.data_tseg2_max
        {
            return false;
        }
        true
    }
}

/* CanRead trait implementation */

impl<T: HasRecvCan + Socket> RecvCan for T {
    fn recv(&self) -> Result<(CanFrame, Timestamp), CanError> {
        let mut frame = CanFrame::default();
        let mut timestamp = Timestamp::default();

        let error_code = unsafe {
            peak_lib()?.CAN_Read(
                self.handle(),
                &mut frame.frame as *mut peak_can::TPEAKMsg,
                &mut timestamp.timestamp as *mut peak_can::TPEAKTimestamp,
            )
        };

        match CanOkError::try_from(error_code) {
            Ok(CanOkError::Ok) => Ok((frame, timestamp)),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }

    fn recv_frame(&self) -> Result<CanFrame, CanError> {
        let mut frame = CanFrame::default();

        let error_code = unsafe {
            peak_lib()?.CAN_Read(
                self.handle(),
                &mut frame.frame as *mut peak_can::TPEAKMsg,
                0 as *mut peak_can::TPEAKTimestamp,
            )
        };

        match CanOkError::try_from(error_code) {
            Ok(CanOkError::Ok) => Ok(frame),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* CanRecvFd trait implementation */

impl<T: HasRecvCanFd + Socket> RecvCanFd for T {
    fn recv_fd(&self) -> Result<(CanFdFrame, u64), CanError> {
        let mut frame = CanFdFrame::default();
        let mut timestamp = 0u64;

        let error_code = unsafe {
            peak_lib()?.CAN_ReadFD(
                self.handle(),
                &mut frame.frame as *mut peak_can::TPEAKMsgFD,
                &mut timestamp as *mut u64,
            )
        };

        match CanOkError::try_from(error_code) {
            Ok(CanOkError::Ok) => Ok((frame, timestamp)),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }

    fn recv_fd_frame(&self) -> Result<CanFdFrame, CanError> {
        let mut frame = CanFdFrame::default();

        let error_code = unsafe {
            peak_lib()?.CAN_ReadFD(
                self.handle(),
                &mut frame.frame as *mut peak_can::TPEAKMsgFD,
                0 as *mut u64,
            )
        };

        match CanOkError::try_from(error_code) {
            Ok(CanOkError::Ok) => Ok(frame),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* CanSend trait implementations */

impl<T: HasSendCan + Socket> SendCan for T {
    fn send(&self, frame: CanFrame) -> Result<(), CanError> {
        let mut frame = frame;
        let error_code = unsafe {
            peak_lib()?.CAN_Write(self.handle(), &mut frame.frame as *mut peak_can::TPEAKMsg)
        };

        match CanOkError::try_from(error_code) {
            Ok(CanOkError::Ok) => Ok(()),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* CanSendFd trait implementation */

impl<T: HasSendCanFd + Socket> SendCanFd for T {
    fn send_fd(&self, frame: CanFdFrame) -> Result<(), CanError> {
        let mut frame = frame;
        let error_code = unsafe {
            peak_lib()?.CAN_WriteFD(self.handle(), &mut frame.frame as *mut peak_can::TPEAKMsgFD)
        };

        match CanOkError::try_from(error_code) {
            Ok(CanOkError::Ok) => Ok(()),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn can_frame_new_001() {
        let can_frame_1 =
            CanFrame::new(0x20, MessageType::Standard, &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();

        let can_frame_2 =
            CanFrame::new(0x20, MessageType::Standard, &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();

        assert_eq!(can_frame_1, can_frame_2);
    }

    #[test]
    fn can_frame_new_002() {
        let can_frame_1 =
            CanFrame::new(0x20, MessageType::Extended, &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();

        let can_frame_2 =
            CanFrame::new(0x20, MessageType::Extended, &[0, 1, 2, 3, 4, 5, 6, 7]).unwrap();

        assert_eq!(can_frame_1, can_frame_2);
    }

    #[test]
    #[should_panic]
    fn can_frame_new_003() {
        let _can_frame_1 =
            CanFrame::new(0x20, MessageType::Standard, &[0, 1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
    }

    #[test]
    #[should_panic]
    fn can_frame_new_004() {
        let _can_frame_1 =
            CanFrame::new(0x20, MessageType::Extended, &[0, 1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
    }

    #[test]
    fn can_frame_new_005() {
        let extended_id = 0x1E_C5_7E_D0;
        // Extended id bitwise and with standard mask
        let standard_id = 0x06_D0;

        let can_frame_1 = CanFrame::new(extended_id, MessageType::Standard, &[0, 1, 2]).unwrap();
        assert_eq!(can_frame_1.can_id(), standard_id);

        let can_frame_2 = CanFrame::new(extended_id, MessageType::Extended, &[0, 1, 2]).unwrap();
        assert_eq!(can_frame_2.can_id(), extended_id);
    }

    #[test]
    fn can_frame_new_006() {
        let can_frame_1 = CanFrame::new(0x01_23, MessageType::Standard, &[0, 1, 2]).unwrap();
        assert!(can_frame_1.is_standard_frame());

        let can_frame_2 = CanFrame::new(0x1f_ff_00_ff, MessageType::Extended, &[0, 1, 2]).unwrap();
        assert!(can_frame_2.is_extended_frame());
    }

    /* CAN FD FRAME */

    #[test]
    fn can_fd_frame_new_001() {
        let can_frame_1 =
            CanFdFrame::new(0x20, MessageType::Standard, &(0..64u8).collect::<Vec<_>>(), false, false).unwrap();

        let can_frame_2 =
            CanFdFrame::new(0x20, MessageType::Standard, &(0..64u8).collect::<Vec<_>>(), false, false).unwrap();

        assert_eq!(can_frame_1, can_frame_2);
    }

    #[test]
    fn can_fd_frame_new_002() {
        let can_frame_1 =
            CanFdFrame::new(0x20, MessageType::Extended, &(0..64u8).collect::<Vec<_>>(), false, false).unwrap();

        let can_frame_2 =
            CanFdFrame::new(0x20, MessageType::Extended, &(0..64u8).collect::<Vec<_>>(), false, false).unwrap();

        assert_eq!(can_frame_1, can_frame_2);
    }

    #[test]
    #[should_panic]
    fn can_fd_frame_new_003() {
        let _can_frame_1 =
            CanFdFrame::new(0x20, MessageType::Standard, &(0..65u8).collect::<Vec<_>>(), false, false).unwrap();
    }

    #[test]
    #[should_panic]
    fn can_fd_frame_new_004() {
        let _can_frame_1 =
            CanFdFrame::new(0x20, MessageType::Extended, &(0..65u8).collect::<Vec<_>>(), false, false).unwrap();
    }

    #[test]
    fn can_fd_frame_new_005() {
        let extended_id = 0x1E_C5_7E_D0;
        // Extended id bitwise and with standard mask
        let standard_id = 0x06_D0;

        let can_frame_1 = CanFdFrame::new(
            extended_id,
            MessageType::Standard,
            &(0..64u8).collect::<Vec<_>>(),
            false,
            false,
        )
        .unwrap();
        assert_eq!(can_frame_1.can_id(), standard_id);

        let can_frame_2 = CanFdFrame::new(
            extended_id,
            MessageType::Extended,
            &(0..64u8).collect::<Vec<_>>(),
            false,
            false,
        )
        .unwrap();

        assert_eq!(can_frame_2.can_id(), extended_id);
    }

    /* RTR FRAME TESTS */

    /// A frame as `CAN_Read` delivers it: MSGTYPE is set by the driver, not by
    /// `new()`.
    fn received_frame(msgtype: u32) -> CanFrame {
        received_frame_with(0x123, msgtype, [0; 8])
    }

    /// As above, with an explicit identifier and payload.
    fn received_frame_with(id: u32, msgtype: u32, data: [u8; 8]) -> CanFrame {
        CanFrame {
            frame: peak_can::TPEAKMsg {
                ID: id,
                MSGTYPE: msgtype as u8,
                LEN: 0,
                DATA: data,
            },
        }
    }

    /// The gap this closes: without the predicate a remote frame answers every
    /// other accessor exactly like a data frame, so it cannot be told apart.
    #[test]
    fn a_remote_frame_is_distinguishable_from_a_data_frame() {
        let remote = received_frame(peak_can::PEAK_MESSAGE_RTR);
        let data = received_frame(peak_can::PEAK_MESSAGE_STANDARD);

        assert!(remote.is_rtr_frame());
        assert!(!data.is_rtr_frame());

        // Everything else about them is identical.
        assert_eq!(remote.can_id(), data.can_id());
        assert_eq!(remote.dlc(), data.dlc());
        assert_eq!(remote.data(), data.data());
    }

    /// RTR is orthogonal to the frame format (§3.1): an extended remote frame
    /// is both.
    #[test]
    fn rtr_is_independent_of_the_frame_format() {
        let extended_remote =
            received_frame(peak_can::PEAK_MESSAGE_RTR | peak_can::PEAK_MESSAGE_EXTENDED);

        assert!(extended_remote.is_rtr_frame());
        assert!(extended_remote.is_extended_frame());
        assert!(!extended_remote.is_standard_frame());
    }

    /// A frame built by `new()` is a data frame; the crate offers no way to
    /// construct a remote frame for sending.
    #[test]
    fn a_constructed_frame_is_not_a_remote_frame() {
        let frame = CanFrame::new(0x123, MessageType::Standard, &[1, 2, 3]).unwrap();
        assert!(!frame.is_rtr_frame());
    }

    /* STATUS MESSAGE TESTS */

    /// A status message reports controller state, not bus traffic. Its
    /// identifier is 0, which a caller may well be treating as a sentinel, so
    /// without the predicate it is indistinguishable from a data frame.
    #[test]
    fn a_status_message_is_distinguishable_from_an_id_zero_frame() {
        let status = received_frame_with(0, peak_can::PEAK_MESSAGE_STATUS, [0; 8]);
        let data = received_frame_with(0, peak_can::PEAK_MESSAGE_STANDARD, [0; 8]);

        assert!(status.is_status_frame());
        assert!(!data.is_status_frame());
        assert_eq!(status.can_id(), data.can_id());
    }

    /// The status word is the first four data bytes, most significant first.
    #[test]
    fn status_bits_decode_big_endian() {
        // PCAN_ERROR_BUSPASSIVE (0x40000).
        let frame = received_frame_with(
            0,
            peak_can::PEAK_MESSAGE_STATUS,
            [0x00, 0x04, 0x00, 0x00, 0, 0, 0, 0],
        );

        assert_eq!(frame.status_bits(), Some(peak_can::PEAK_ERROR_BUSPASSIVE));
    }

    /// The word classifies through `CanError`, so a status message turns into
    /// the same error type the rest of the crate returns.
    #[test]
    fn status_bits_classify_as_a_can_error() {
        let frame = received_frame_with(
            0,
            peak_can::PEAK_MESSAGE_STATUS,
            [0x00, 0x00, 0x00, 0x10, 0, 0, 0, 0],
        );

        let bits = frame.status_bits().expect("a status message has a status word");
        assert!(matches!(CanError::try_from(bits), Ok(CanError::BusOff)));
    }

    /// Only status messages carry a status word.
    #[test]
    fn a_data_frame_has_no_status_bits() {
        let frame = received_frame_with(
            0x123,
            peak_can::PEAK_MESSAGE_STANDARD,
            [0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0],
        );

        assert_eq!(frame.status_bits(), None);
    }

    /// `LEN` is not trusted: the word is read from the fixed-size DATA array, so
    /// a driver reporting LEN 0 does not hide the payload.
    #[test]
    fn status_bits_ignore_the_reported_length() {
        let frame = received_frame_with(
            0,
            peak_can::PEAK_MESSAGE_STATUS,
            [0x00, 0x00, 0x00, 0x10, 0, 0, 0, 0],
        );

        assert_eq!(frame.dlc(), 0, "the helper reports LEN 0");
        assert_eq!(frame.data(), &[] as &[u8], "so data() is empty");
        assert_eq!(frame.status_bits(), Some(peak_can::PEAK_ERROR_BUSOFF));
    }

    /* calc_dlc TESTS */

    #[test]
    fn calc_dlc_encoding() {
        // Test all critical boundary points for CAN FD DLC encoding
        let test_cases = vec![
            (0, 0), (1, 1), (8, 8),           // 0-8: DLC equals length
            (9, 9), (12, 9),                   // 9-12: DLC 9
            (13, 10), (16, 10),                // 13-16: DLC 10
            (17, 11), (20, 11),                // 17-20: DLC 11
            (21, 12), (24, 12),                // 21-24: DLC 12
            (25, 13), (32, 13),                // 25-32: DLC 13
            (33, 14), (48, 14),                // 33-48: DLC 14
            (49, 15), (64, 15),                // 49-64: DLC 15
            (65, 15), (100, 15),               // >64: DLC 15 (max)
        ];

        for (len, expected_dlc) in test_cases {
            assert_eq!(CanFdFrame::calc_dlc(len), expected_dlc, "Failed for length {}", len);
        }
    }

    /* len (DLC to data length) TESTS */

    #[test]
    fn len_decoding() {
        // Test all valid DLC values decode to correct data lengths
        let test_cases = vec![
            (0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6), (7, 7), (8, 8),
            (12, 12), (16, 16), (20, 20), (24, 24), (32, 32), (48, 48), (64, 64),
        ];

        for (data_len, expected_len) in test_cases {
            let data = vec![0u8; data_len];
            let frame = CanFdFrame::new(0x123, MessageType::Standard, &data, false, false).unwrap();
            assert_eq!(frame.len(), expected_len, "Failed for data length {}", data_len);
        }
    }

    #[test]
    fn len_calc_dlc_inverse() {
        // Verify len and calc_dlc are proper inverses, and non-standard lengths round up
        let test_cases = vec![
            (0, 0), (8, 8),                    // Standard 0-8
            (9, 12), (12, 12),                 // 9-12 rounds to 12
            (13, 16), (16, 16),                // 13-16 rounds to 16
            (20, 20), (24, 24),                // Standard boundaries
            (25, 32), (32, 32),                // 25-32 rounds to 32
            (40, 48), (48, 48),                // 33-48 rounds to 48
            (50, 64), (64, 64),                // 49-64 rounds to 64
        ];

        for (input_len, expected_frame_len) in test_cases {
            let data = vec![0u8; input_len];
            let frame = CanFdFrame::new(0x123, MessageType::Standard, &data, false, false).unwrap();
            assert_eq!(frame.len(), expected_frame_len, "Failed for input length {}", input_len);
        }
    }

    /* CanBitTiming TESTS */

    #[test]
    fn can_bit_timing_valid_parameters() {
        // Test valid parameters within bounds are accepted
        let test_cases = vec![
            (1, 1, 1, 1),      // All minimum values
            (64, 4, 16, 8),    // All maximum values
            (8, 1, 13, 2),     // Common 500 kbit/s configuration
            (32, 2, 8, 4),     // Mid-range values
        ];

        for (prescaler, sjw, tseg1, tseg2) in test_cases {
            let result = CanBitTiming::new(prescaler, sjw, tseg1, tseg2);
            assert!(result.is_ok(), "Should accept valid parameters: prescaler={}, sjw={}, tseg1={}, tseg2={}", 
                    prescaler, sjw, tseg1, tseg2);
            
            let timing = result.unwrap();
            assert_eq!(timing.prescaler, prescaler);
            assert_eq!(timing.sjw, sjw);
            assert_eq!(timing.tseg1, tseg1);
            assert_eq!(timing.tseg2, tseg2);
        }
    }

    #[test]
    fn can_bit_timing_invalid_prescaler() {
        // Test prescaler out of bounds (valid: 1-64)
        assert!(CanBitTiming::new(0, 1, 1, 1).is_err(), "Should reject prescaler=0");
        assert!(CanBitTiming::new(65, 1, 1, 1).is_err(), "Should reject prescaler=65");
        assert!(CanBitTiming::new(100, 1, 1, 1).is_err(), "Should reject prescaler=100");
    }

    #[test]
    fn can_bit_timing_invalid_sjw() {
        // Test SJW out of bounds (valid: 1-4)
        assert!(CanBitTiming::new(8, 0, 1, 1).is_err(), "Should reject sjw=0");
        assert!(CanBitTiming::new(8, 5, 1, 1).is_err(), "Should reject sjw=5");
        assert!(CanBitTiming::new(8, 10, 1, 1).is_err(), "Should reject sjw=10");
    }

    #[test]
    fn can_bit_timing_invalid_tseg1() {
        // Test TSEG1 out of bounds (valid: 1-16)
        assert!(CanBitTiming::new(8, 1, 0, 1).is_err(), "Should reject tseg1=0");
        assert!(CanBitTiming::new(8, 1, 17, 1).is_err(), "Should reject tseg1=17");
        assert!(CanBitTiming::new(8, 1, 20, 1).is_err(), "Should reject tseg1=20");
    }

    #[test]
    fn can_bit_timing_invalid_tseg2() {
        // Test TSEG2 out of bounds (valid: 1-8)
        assert!(CanBitTiming::new(8, 1, 1, 0).is_err(), "Should reject tseg2=0");
        assert!(CanBitTiming::new(8, 1, 1, 9).is_err(), "Should reject tseg2=9");
        assert!(CanBitTiming::new(8, 1, 1, 15).is_err(), "Should reject tseg2=15");
    }

    #[test]
    fn can_bit_timing_boundary_values() {
        // Test exact boundary values are accepted
        assert!(CanBitTiming::new(1, 1, 1, 1).is_ok(), "Should accept all minimum boundaries");
        assert!(CanBitTiming::new(64, 4, 16, 8).is_ok(), "Should accept all maximum boundaries");
        
        // Test just outside boundaries are rejected
        assert!(CanBitTiming::new(0, 1, 1, 1).is_err());
        assert!(CanBitTiming::new(65, 1, 1, 1).is_err());
        assert!(CanBitTiming::new(1, 0, 1, 1).is_err());
        assert!(CanBitTiming::new(1, 5, 1, 1).is_err());
        assert!(CanBitTiming::new(1, 1, 0, 1).is_err());
        assert!(CanBitTiming::new(1, 1, 17, 1).is_err());
        assert!(CanBitTiming::new(1, 1, 1, 0).is_err());
        assert!(CanBitTiming::new(1, 1, 1, 9).is_err());
    }

    /* CanFdBitTiming TESTS */

    #[test]
    fn can_fd_bit_timing_valid_parameters() {
        // Test valid parameters within bounds are accepted
        let test_cases = vec![
            // (nom_prescaler, nom_sjw, nom_tseg1, nom_tseg2, data_prescaler, data_sjw, data_tseg1, data_tseg2)
            (1, 1, 1, 1, 1, 1, 1, 1),          // All minimum values
            (1024, 128, 256, 128, 1024, 16, 32, 16), // All maximum values
            (2, 10, 127, 32, 2, 5, 25, 8),     // Common 500k/2M configuration
            (500, 50, 100, 50, 100, 8, 16, 8), // Mid-range values
        ];

        for (np, ns, nt1, nt2, dp, ds, dt1, dt2) in test_cases {
            let result = CanFdBitTiming::new(np, ns, nt1, nt2, dp, ds, dt1, dt2);
            assert!(result.is_ok(), "Should accept valid parameters: nom({},{},{},{}), data({},{},{},{})", 
                    np, ns, nt1, nt2, dp, ds, dt1, dt2);
            
            let timing = result.unwrap();
            assert_eq!(timing.nom_prescaler, np);
            assert_eq!(timing.nom_sjw, ns);
            assert_eq!(timing.nom_tseg1, nt1);
            assert_eq!(timing.nom_tseg2, nt2);
            assert_eq!(timing.data_prescaler, dp);
            assert_eq!(timing.data_sjw, ds);
            assert_eq!(timing.data_tseg1, dt1);
            assert_eq!(timing.data_tseg2, dt2);
        }
    }

    #[test]
    fn can_fd_bit_timing_invalid_nom_prescaler() {
        // Nominal prescaler out of bounds (valid: 1-1024)
        assert!(CanFdBitTiming::new(0, 1, 1, 1, 1, 1, 1, 1).is_err(), "Should reject nom_prescaler=0");
        assert!(CanFdBitTiming::new(1025, 1, 1, 1, 1, 1, 1, 1).is_err(), "Should reject nom_prescaler=1025");
        assert!(CanFdBitTiming::new(2000, 1, 1, 1, 1, 1, 1, 1).is_err(), "Should reject nom_prescaler=2000");
    }

    #[test]
    fn can_fd_bit_timing_invalid_nom_sjw() {
        // Nominal SJW out of bounds (valid: 1-128)
        assert!(CanFdBitTiming::new(1, 0, 1, 1, 1, 1, 1, 1).is_err(), "Should reject nom_sjw=0");
        assert!(CanFdBitTiming::new(1, 129, 1, 1, 1, 1, 1, 1).is_err(), "Should reject nom_sjw=129");
        assert!(CanFdBitTiming::new(1, 200, 1, 1, 1, 1, 1, 1).is_err(), "Should reject nom_sjw=200");
    }

    #[test]
    fn can_fd_bit_timing_invalid_nom_tseg1() {
        // Nominal TSEG1 out of bounds (valid: 1-256)
        assert!(CanFdBitTiming::new(1, 1, 0, 1, 1, 1, 1, 1).is_err(), "Should reject nom_tseg1=0");
        assert!(CanFdBitTiming::new(1, 1, 257, 1, 1, 1, 1, 1).is_err(), "Should reject nom_tseg1=257");
        assert!(CanFdBitTiming::new(1, 1, 300, 1, 1, 1, 1, 1).is_err(), "Should reject nom_tseg1=300");
    }

    #[test]
    fn can_fd_bit_timing_invalid_nom_tseg2() {
        // Nominal TSEG2 out of bounds (valid: 1-128)
        assert!(CanFdBitTiming::new(1, 1, 1, 0, 1, 1, 1, 1).is_err(), "Should reject nom_tseg2=0");
        assert!(CanFdBitTiming::new(1, 1, 1, 129, 1, 1, 1, 1).is_err(), "Should reject nom_tseg2=129");
        assert!(CanFdBitTiming::new(1, 1, 1, 200, 1, 1, 1, 1).is_err(), "Should reject nom_tseg2=200");
    }

    #[test]
    fn can_fd_bit_timing_invalid_data_prescaler() {
        // Data prescaler out of bounds (valid: 1-1024)
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 0, 1, 1, 1).is_err(), "Should reject data_prescaler=0");
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1025, 1, 1, 1).is_err(), "Should reject data_prescaler=1025");
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 2000, 1, 1, 1).is_err(), "Should reject data_prescaler=2000");
    }

    #[test]
    fn can_fd_bit_timing_invalid_data_sjw() {
        // Data SJW out of bounds (valid: 1-16)
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 0, 1, 1).is_err(), "Should reject data_sjw=0");
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 17, 1, 1).is_err(), "Should reject data_sjw=17");
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 50, 1, 1).is_err(), "Should reject data_sjw=50");
    }

    #[test]
    fn can_fd_bit_timing_invalid_data_tseg1() {
        // Data TSEG1 out of bounds (valid: 1-32)
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 1, 0, 1).is_err(), "Should reject data_tseg1=0");
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 1, 33, 1).is_err(), "Should reject data_tseg1=33");
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 1, 50, 1).is_err(), "Should reject data_tseg1=50");
    }

    #[test]
    fn can_fd_bit_timing_invalid_data_tseg2() {
        // Data TSEG2 out of bounds (valid: 1-16)
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 1, 1, 0).is_err(), "Should reject data_tseg2=0");
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 1, 1, 17).is_err(), "Should reject data_tseg2=17");
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 1, 1, 50).is_err(), "Should reject data_tseg2=50");
    }

    #[test]
    fn can_fd_bit_timing_boundary_values() {
        // Test exact boundary values are accepted
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 1, 1, 1).is_ok(), 
                "Should accept all minimum boundaries");
        assert!(CanFdBitTiming::new(1024, 128, 256, 128, 1024, 16, 32, 16).is_ok(), 
                "Should accept all maximum boundaries");
        
        // Test just outside nominal boundaries are rejected
        assert!(CanFdBitTiming::new(0, 1, 1, 1, 1, 1, 1, 1).is_err());
        assert!(CanFdBitTiming::new(1025, 1, 1, 1, 1, 1, 1, 1).is_err());
        assert!(CanFdBitTiming::new(1, 0, 1, 1, 1, 1, 1, 1).is_err());
        assert!(CanFdBitTiming::new(1, 129, 1, 1, 1, 1, 1, 1).is_err());
        assert!(CanFdBitTiming::new(1, 1, 0, 1, 1, 1, 1, 1).is_err());
        assert!(CanFdBitTiming::new(1, 1, 257, 1, 1, 1, 1, 1).is_err());
        assert!(CanFdBitTiming::new(1, 1, 1, 0, 1, 1, 1, 1).is_err());
        assert!(CanFdBitTiming::new(1, 1, 1, 129, 1, 1, 1, 1).is_err());
        
        // Test just outside data boundaries are rejected
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 0, 1, 1, 1).is_err());
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1025, 1, 1, 1).is_err());
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 0, 1, 1).is_err());
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 17, 1, 1).is_err());
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 1, 0, 1).is_err());
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 1, 33, 1).is_err());
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 1, 1, 0).is_err());
        assert!(CanFdBitTiming::new(1, 1, 1, 1, 1, 1, 1, 17).is_err());
    }
}
