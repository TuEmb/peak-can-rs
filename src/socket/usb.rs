//!
//!
//!

use std::ffi::CString;

use crate::bus::UsbBus;
use crate::channel::Channel;
use crate::df::{
    HasAcceptanceFilter11Bit, HasAcceptanceFilter29Bit, HasAllowEchoFrames, HasAllowErrorFrames,
    HasAllowRTRFrames, HasAllowStatusFrames, HasMessageFilter, HasReceiveStatus,
    HasSetAcceptanceFilter11Bit, HasSetAcceptanceFilter29Bit, HasSetAllowEchoFrames,
    HasSetAllowErrorFrames, HasSetAllowRTRFrames, HasSetAllowStatusFrames, HasSetMessageFilter,
    HasSetReceiveStatus,
};
use crate::error::{CanError, CanOkError};
use crate::hw::{
    HasChannelIdentifying, HasControllerNumber, HasDeviceId, HasDevicePartNumber, HasHardwareName,
    HasSetControllerNumber, HasSetDeviceId,
};
use crate::info::{
    HasBitrateInfo, HasChannelFeatures, HasChannelVersion, HasDataBusSpeed, HasFirmwareVersion,
    HasNominalBusSpeed,
};
use crate::io::{
    HasAnalogValue, HasDigitalConfiguration, HasDigitalValue, HasSetDigitalClear,
    HasSetDigitalConfiguration, HasSetDigitalSet, HasSetDigitalValue,
};
use crate::peak_lib;
use crate::socket::{Baudrate, CanBitTiming, CanFdBitTiming, HasRecvCan, HasRecvCanFd, HasSendCan, HasSendCanFd, Socket};
use crate::special::{
    HasBusOffAutoreset, HasFiveVoltsPower, HasInterframeDelay, HasListenOnly,
    HasSetBusOffAutoreset, HasSetFiveVoltsPower, HasSetInterframeDelay, HasSetListenOnly,
};
use crate::trace::{
    HasSetTraceConfigure, HasSetTraceLocation, HasSetTraceSize, HasSetTraceStatus,
    HasTraceConfigure, HasTraceLocation, HasTraceSize, HasTraceStatus,
};

/// CAN FD controller clock frequency in Hz for PEAK USB devices.
/// 
/// This represents the base clock frequency (80 MHz) used by the CAN FD controller
/// on PEAK USB hardware. This value is used when configuring custom bit timing
/// parameters for CAN FD communication.
const CANFD_CLOCK_HZ: u32 = 80_000_000;

/// Helper function to calculate BTR0BTR1 value
fn calculate_btr0btr1(timing: &CanBitTiming) -> u16 {
    ((((timing.tseg2 - 1) & 0x07) as u16) << 4)
        | (((timing.tseg1 - 1) & 0x0F) as u16)
        | ((((timing.prescaler - 1) & 0x3F) as u16) << 8)
        | ((((timing.sjw - 1) & 0x03) as u16) << 14)
}

/// Helper function to build timing string
fn build_timing_string(timing: &CanFdBitTiming) -> String {
    format!(
        "f_clock={},nom_brp={},nom_tseg1={},nom_tseg2={},nom_sjw={},data_brp={},data_tseg1={},data_tseg2={},data_sjw={}",
        CANFD_CLOCK_HZ,
        timing.nom_prescaler,
        timing.nom_tseg1,
        timing.nom_tseg2,
        timing.nom_sjw,
        timing.data_prescaler,
        timing.data_tseg1,
        timing.data_tseg2,
        timing.data_sjw,
    )
}

#[derive(Debug, PartialEq)]
pub struct UsbCanSocket {
    handle: u16,
}

impl UsbCanSocket {
    /// Opens a CAN socket with a standard baudrate.
    ///
    /// # Arguments
    ///
    /// * `bus` - USB bus channel (USB1-USB16)
    /// * `baud` - Standard baudrate (e.g., Baud125K, Baud500K)
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use peak_can::socket::usb::UsbCanSocket;
    /// # use peak_can::socket::Baudrate;
    /// # use peak_can::bus::UsbBus;
    /// let socket = UsbCanSocket::open(UsbBus::USB1, Baudrate::Baud500K)?;
    /// # Ok::<(), peak_can::error::CanError>(())
    /// ```
    pub fn open(bus: UsbBus, baud: Baudrate) -> Result<UsbCanSocket, CanError> {
        let handle = bus.into();
        let code = unsafe { peak_lib()?.CAN_Initialize(handle, baud.into(), 0, 0, 0) };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(UsbCanSocket { handle }),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }

    pub fn open_with_usb_bus(bus: UsbBus) -> UsbCanSocket {
        let handle = bus.into();
        UsbCanSocket { handle }
    }

    /// Opens a CAN socket with custom bit timing.
    ///
    /// Use [`CAN_TIMING_BOUNDARIES`](crate::socket::CAN_TIMING_BOUNDARIES) for valid ranges.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use peak_can::socket::usb::UsbCanSocket;
    /// # use peak_can::socket::CanBitTiming;
    /// # use peak_can::bus::UsbBus;
    /// let timing = CanBitTiming::new(8, 1, 13, 2)?;  // 500 kbit/s
    /// let socket = UsbCanSocket::open_with_timing(UsbBus::USB1, &timing)?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn open_with_timing(bus: UsbBus, timing: &CanBitTiming) -> Result<UsbCanSocket, CanError> {
        let handle = bus.into();
        let btr0btr1 = calculate_btr0btr1(timing);
        let code = unsafe { peak_lib()?.CAN_Initialize(handle, btr0btr1, 0, 0, 0) };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(UsbCanSocket { handle }),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }

    /// Opens a CAN FD socket with custom timing for nominal and data phases.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use peak_can::socket::usb::UsbCanSocket;
    /// # use peak_can::socket::CanFdBitTiming;
    /// # use peak_can::bus::UsbBus;
    /// let timing = CanFdBitTiming::new(
    ///     10, 4, 13, 2,  // Nominal phase
    ///     5, 2, 6, 1     // Data phase
    /// )?;
    /// let socket = UsbCanSocket::open_fd_with_timing(UsbBus::USB1, &timing)?;
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn open_fd_with_timing(bus: UsbBus, timing: &CanFdBitTiming) -> Result<UsbCanSocket, CanError> {
        let handle = bus.into();
        let timing_str = build_timing_string(timing);

        let mut timing_bytes = CString::new(timing_str)
            .map_err(|_| CanError::Unknown)?
            .into_bytes_with_nul();

        let code = unsafe { peak_lib()?.CAN_InitializeFD(handle, timing_bytes.as_mut_ptr().cast()) };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(UsbCanSocket { handle }),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* Drop trait implementation */

impl Drop for UsbCanSocket {
    fn drop(&mut self) {
        let Ok(peak_lib) = peak_lib() else {
            return;
        };
        unsafe { peak_lib.CAN_Uninitialize(self.handle) };
    }
}

/* Socket trait implementation */

impl Socket for UsbCanSocket {
    fn handle(&self) -> u16 {
        self.handle
    }
}

/* Channel trait implementation */

impl Channel for UsbCanSocket {
    fn channel(&self) -> u16 {
        self.handle
    }
}

/* CAN trait implementations */

impl HasRecvCan for UsbCanSocket {}
impl HasSendCan for UsbCanSocket {}

impl HasRecvCanFd for UsbCanSocket {}
impl HasSendCanFd for UsbCanSocket {}

/* HARDWARE IDENTIFICATION */

impl HasChannelIdentifying for UsbCanSocket {}

impl HasDeviceId for UsbCanSocket {}
impl HasSetDeviceId for UsbCanSocket {}

impl HasHardwareName for UsbCanSocket {}

impl HasControllerNumber for UsbCanSocket {}
impl HasSetControllerNumber for UsbCanSocket {}

impl HasDevicePartNumber for UsbCanSocket {}

/* INFORMATIONAL PARAMETER */

impl HasChannelVersion for UsbCanSocket {}

impl HasChannelFeatures for UsbCanSocket {}

impl HasBitrateInfo for UsbCanSocket {}

impl HasNominalBusSpeed for UsbCanSocket {}

impl HasDataBusSpeed for UsbCanSocket {}

impl HasFirmwareVersion for UsbCanSocket {}

/* SPECIAL BEHAVIOR */

impl HasFiveVoltsPower for UsbCanSocket {}
impl HasSetFiveVoltsPower for UsbCanSocket {}

impl HasBusOffAutoreset for UsbCanSocket {}
impl HasSetBusOffAutoreset for UsbCanSocket {}

impl HasListenOnly for UsbCanSocket {}
impl HasSetListenOnly for UsbCanSocket {}

impl HasInterframeDelay for UsbCanSocket {}
impl HasSetInterframeDelay for UsbCanSocket {}

/* CONTROLLING DATA FLOW */

impl HasMessageFilter for UsbCanSocket {}
impl HasSetMessageFilter for UsbCanSocket {}

impl HasReceiveStatus for UsbCanSocket {}
impl HasSetReceiveStatus for UsbCanSocket {}

impl HasAllowStatusFrames for UsbCanSocket {}
impl HasSetAllowStatusFrames for UsbCanSocket {}

impl HasAllowRTRFrames for UsbCanSocket {}
impl HasSetAllowRTRFrames for UsbCanSocket {}

impl HasAllowErrorFrames for UsbCanSocket {}
impl HasSetAllowErrorFrames for UsbCanSocket {}

impl HasAllowEchoFrames for UsbCanSocket {}
impl HasSetAllowEchoFrames for UsbCanSocket {}

impl HasAcceptanceFilter11Bit for UsbCanSocket {}
impl HasSetAcceptanceFilter11Bit for UsbCanSocket {}

impl HasAcceptanceFilter29Bit for UsbCanSocket {}
impl HasSetAcceptanceFilter29Bit for UsbCanSocket {}

/* TRACING PARAMETERS */

impl HasTraceLocation for UsbCanSocket {}
impl HasSetTraceLocation for UsbCanSocket {}

impl HasTraceStatus for UsbCanSocket {}
impl HasSetTraceStatus for UsbCanSocket {}

impl HasTraceSize for UsbCanSocket {}
impl HasSetTraceSize for UsbCanSocket {}

impl HasTraceConfigure for UsbCanSocket {}
impl HasSetTraceConfigure for UsbCanSocket {}

/* ELECTRONIC CIRCUITS PARAMETERS */

impl HasDigitalConfiguration for UsbCanSocket {}
impl HasSetDigitalConfiguration for UsbCanSocket {}

impl HasDigitalValue for UsbCanSocket {}
impl HasSetDigitalValue for UsbCanSocket {}

impl HasSetDigitalSet for UsbCanSocket {}

impl HasSetDigitalClear for UsbCanSocket {}

impl HasAnalogValue for UsbCanSocket {}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn btr0btr1_encoding() {
        // Test boundary values and common configurations
        let test_cases = vec![
            ((1, 1, 1, 1), 0x0000),      // Minimum values
            ((64, 4, 16, 8), 0xFF7F),    // Maximum values
            ((8, 1, 13, 2), 0x071C),     // 500k config
            ((32, 1, 13, 2), 0x1F1C),    // 125k config
            ((4, 1, 13, 2), 0x031C),     // 1M config
        ];

        for ((prescaler, sjw, tseg1, tseg2), expected) in test_cases {
            let timing = CanBitTiming::new(prescaler, sjw, tseg1, tseg2).unwrap();
            let btr0btr1 = calculate_btr0btr1(&timing);
            assert_eq!(btr0btr1, expected, 
                "Failed for prescaler={}, sjw={}, tseg1={}, tseg2={}", 
                prescaler, sjw, tseg1, tseg2);
        }

        // Verify bit field masking doesn't overflow
        let timing = CanBitTiming::new(64, 4, 16, 8).unwrap();
        let btr0btr1 = calculate_btr0btr1(&timing);
        assert_eq!(btr0btr1 & 0x000F, 15);  // tseg1
        assert_eq!((btr0btr1 >> 4) & 0x07, 7);  // tseg2
        assert_eq!((btr0btr1 >> 8) & 0x3F, 63);  // prescaler
        assert_eq!((btr0btr1 >> 14) & 0x03, 3);  // sjw
    }

    #[test]
    fn fd_timing_string_format() {
        // Test boundary values and common configurations
        let test_cases = vec![
            (
                (1, 1, 1, 1, 1, 1, 1, 1),
                "f_clock=80000000,nom_brp=1,nom_tseg1=1,nom_tseg2=1,nom_sjw=1,data_brp=1,data_tseg1=1,data_tseg2=1,data_sjw=1"
            ),
            (
                (1024, 128, 256, 128, 1024, 16, 32, 16),
                "f_clock=80000000,nom_brp=1024,nom_tseg1=256,nom_tseg2=128,nom_sjw=128,data_brp=1024,data_tseg1=32,data_tseg2=16,data_sjw=16"
            ),
            (
                (10, 4, 13, 2, 5, 2, 6, 1),  // 500k nominal / 1M data
                "f_clock=80000000,nom_brp=10,nom_tseg1=13,nom_tseg2=2,nom_sjw=4,data_brp=5,data_tseg1=6,data_tseg2=1,data_sjw=2"
            ),
            (
                (5, 2, 13, 2, 2, 2, 13, 2),  // 1M nominal / 2M data
                "f_clock=80000000,nom_brp=5,nom_tseg1=13,nom_tseg2=2,nom_sjw=2,data_brp=2,data_tseg1=13,data_tseg2=2,data_sjw=2"
            ),
        ];

        for ((nom_brp, nom_sjw, nom_tseg1, nom_tseg2, data_brp, data_sjw, data_tseg1, data_tseg2), expected) in test_cases {
            let timing = CanFdBitTiming::new(nom_brp, nom_sjw, nom_tseg1, nom_tseg2, data_brp, data_sjw, data_tseg1, data_tseg2).unwrap();
            let actual = build_timing_string(&timing);
            assert_eq!(actual, expected);
        }
    }

    #[test]
    fn fd_timing_string_structure() {
        let timing = CanFdBitTiming::new(10, 4, 13, 2, 5, 2, 6, 1).unwrap();
        let timing_str = build_timing_string(&timing);
        
        // Verify structure: 9 parameters, no spaces, proper delimiters
        assert_eq!(timing_str.matches(',').count(), 8);
        assert!(!timing_str.contains(' '));
        assert!(timing_str.starts_with("f_clock=80000000,"));
        assert!(timing_str.ends_with("data_sjw=2"));
        assert!(!timing_str.contains(|c: char| c.is_control()));
        
        // Verify parameter order
        let parts: Vec<&str> = timing_str.split(',').collect();
        assert_eq!(parts.len(), 9);
        assert!(parts[0].starts_with("f_clock="));
        assert!(parts[1].starts_with("nom_brp="));
        assert!(parts[4].starts_with("nom_sjw="));
        assert!(parts[5].starts_with("data_brp="));
        assert!(parts[8].starts_with("data_sjw="));
    }
}