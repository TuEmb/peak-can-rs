//!
//!
//!

use crate::bus::PciBus;
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
    HasControllerNumber, HasDeviceId, HasDevicePartNumber, HasHardwareName, HasSetControllerNumber,
    HasSetDeviceId,
};
use crate::info::{
    HasBitrateInfo, HasChannelFeatures, HasChannelVersion, HasDataBusSpeed, HasFirmwareVersion,
    HasNominalBusSpeed,
};
use crate::peak_can;
use crate::socket::{Baudrate, HasRecvCan, HasRecvCanFd, HasSendCan, HasSendCanFd, Socket};
use crate::trace::{
    HasSetTraceConfigure, HasSetTraceLocation, HasSetTraceSize, HasSetTraceStatus,
    HasTraceConfigure, HasTraceLocation, HasTraceSize, HasTraceStatus,
};

#[derive(Debug, PartialEq)]
pub struct PciCanSocket {
    handle: u16,
}

impl PciCanSocket {
    pub fn open(bus: PciBus, baud: Baudrate) -> Result<PciCanSocket, CanError> {
        let handle = bus.into();
        let code = unsafe { peak_can::CAN_Initialize(handle, baud.into(), 0, 0, 0) };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(PciCanSocket { handle }),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* Drop trait implementation */

impl Drop for PciCanSocket {
    fn drop(&mut self) {
        unsafe { peak_can::CAN_Uninitialize(self.handle) };
    }
}

/* Socket trait implementation */

impl Socket for PciCanSocket {
    fn handle(&self) -> u16 {
        self.handle
    }
}

/* Channel trait implementation */

impl Channel for PciCanSocket {
    fn channel(&self) -> u16 {
        self.handle
    }
}

/* CAN trait implementations */

impl HasRecvCan for PciCanSocket {}
impl HasSendCan for PciCanSocket {}

// impl HasRecvCanFd for PciCanSocket {}
// impl HasSendCanFd for PciCanSocket {}

/* HARDWARE IDENTIFICATION */

impl HasDeviceId for PciCanSocket {}
impl HasSetDeviceId for PciCanSocket {}

impl HasHardwareName for PciCanSocket {}

impl HasControllerNumber for PciCanSocket {}
impl HasSetControllerNumber for PciCanSocket {}

impl HasDevicePartNumber for PciCanSocket {}

/* INFORMATIONAL PARAMETER */

impl HasChannelVersion for PciCanSocket {}

impl HasChannelFeatures for PciCanSocket {}

impl HasBitrateInfo for PciCanSocket {}

impl HasNominalBusSpeed for PciCanSocket {}

impl HasDataBusSpeed for PciCanSocket {}

impl HasFirmwareVersion for PciCanSocket {}

/* SPECIAL BEHAVIOR */

/* CONTROLLING DATA FLOW */

impl HasMessageFilter for PciCanSocket {}
impl HasSetMessageFilter for PciCanSocket {}

impl HasReceiveStatus for PciCanSocket {}
impl HasSetReceiveStatus for PciCanSocket {}

impl HasAllowStatusFrames for PciCanSocket {}
impl HasSetAllowStatusFrames for PciCanSocket {}

impl HasAllowRTRFrames for PciCanSocket {}
impl HasSetAllowRTRFrames for PciCanSocket {}

impl HasAllowErrorFrames for PciCanSocket {}
impl HasSetAllowErrorFrames for PciCanSocket {}

impl HasAllowEchoFrames for PciCanSocket {}
impl HasSetAllowEchoFrames for PciCanSocket {}

impl HasAcceptanceFilter11Bit for PciCanSocket {}
impl HasSetAcceptanceFilter11Bit for PciCanSocket {}

impl HasAcceptanceFilter29Bit for PciCanSocket {}
impl HasSetAcceptanceFilter29Bit for PciCanSocket {}

/* TRACING PARAMETERS */

impl HasTraceLocation for PciCanSocket {}
impl HasSetTraceLocation for PciCanSocket {}

impl HasTraceStatus for PciCanSocket {}
impl HasSetTraceStatus for PciCanSocket {}

impl HasTraceSize for PciCanSocket {}
impl HasSetTraceSize for PciCanSocket {}

impl HasTraceConfigure for PciCanSocket {}
impl HasSetTraceConfigure for PciCanSocket {}
