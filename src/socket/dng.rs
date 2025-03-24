//!
//!
//!

use crate::bus::DngBus;
use crate::channel::Channel;
use crate::df::{
    HasAcceptanceFilter11Bit, HasAcceptanceFilter29Bit, HasAllowErrorFrames, HasAllowRTRFrames,
    HasAllowStatusFrames, HasMessageFilter, HasReceiveStatus, HasSetAcceptanceFilter11Bit,
    HasSetAcceptanceFilter29Bit, HasSetAllowErrorFrames, HasSetAllowRTRFrames,
    HasSetAllowStatusFrames, HasSetMessageFilter, HasSetReceiveStatus,
};
use crate::error::{CanError, CanOkError};
use crate::hw::{
    HasControllerNumber, HasDevicePartNumber, HasHardwareName, HasSetControllerNumber,
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
pub struct DngCanSocket {
    handle: u16,
}

impl DngCanSocket {
    pub fn open(bus: DngBus, baud: Baudrate) -> Result<DngCanSocket, CanError> {
        let handle = bus.into();
        let code = unsafe { peak_can::CAN_Initialize(handle, baud.into(), 0, 0, 0) };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(DngCanSocket { handle }),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* Drop trait implementations */

impl Drop for DngCanSocket {
    fn drop(&mut self) {
        unsafe { peak_can::CAN_Uninitialize(self.handle) };
    }
}

/* Socket trait implementation */

impl Socket for DngCanSocket {
    fn handle(&self) -> u16 {
        self.handle
    }
}

/* Channel trait implementation */

impl Channel for DngCanSocket {
    fn channel(&self) -> u16 {
        self.handle
    }
}

/* CAN trait implementations */

impl HasRecvCan for DngCanSocket {}
impl HasSendCan for DngCanSocket {}

// impl HasRecvCanFd for DngCanSocket {}
// impl HasSendCanFd for DngCanSocket {}

/* HARDWARE IDENTIFICATION */

impl HasHardwareName for DngCanSocket {}

impl HasControllerNumber for DngCanSocket {}
impl HasSetControllerNumber for DngCanSocket {}

impl HasDevicePartNumber for DngCanSocket {}

/* INFORMATIONAL PARAMETER */

impl HasChannelVersion for DngCanSocket {}

impl HasChannelFeatures for DngCanSocket {}

impl HasBitrateInfo for DngCanSocket {}

impl HasNominalBusSpeed for DngCanSocket {}

impl HasDataBusSpeed for DngCanSocket {}

impl HasFirmwareVersion for DngCanSocket {}

/* SPECIAL BEHAVIOR */

/* CONTROLLING DATA FLOW */

impl HasMessageFilter for DngCanSocket {}
impl HasSetMessageFilter for DngCanSocket {}

impl HasReceiveStatus for DngCanSocket {}
impl HasSetReceiveStatus for DngCanSocket {}

impl HasAllowStatusFrames for DngCanSocket {}
impl HasSetAllowStatusFrames for DngCanSocket {}

impl HasAllowRTRFrames for DngCanSocket {}
impl HasSetAllowRTRFrames for DngCanSocket {}

impl HasAllowErrorFrames for DngCanSocket {}
impl HasSetAllowErrorFrames for DngCanSocket {}

impl HasAcceptanceFilter11Bit for DngCanSocket {}
impl HasSetAcceptanceFilter11Bit for DngCanSocket {}

impl HasAcceptanceFilter29Bit for DngCanSocket {}
impl HasSetAcceptanceFilter29Bit for DngCanSocket {}

/* TRACING PARAMETERS */

impl HasTraceLocation for DngCanSocket {}
impl HasSetTraceLocation for DngCanSocket {}

impl HasTraceStatus for DngCanSocket {}
impl HasSetTraceStatus for DngCanSocket {}

impl HasTraceSize for DngCanSocket {}
impl HasSetTraceSize for DngCanSocket {}

impl HasTraceConfigure for DngCanSocket {}
impl HasSetTraceConfigure for DngCanSocket {}
