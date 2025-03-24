//!
//!
//!

use crate::bus::IsaBus;
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
pub struct IsaCanSocket {
    handle: u16,
}

impl IsaCanSocket {
    pub fn open(bus: IsaBus, baud: Baudrate) -> Result<IsaCanSocket, CanError> {
        let code = unsafe { peak_can::CAN_Initialize(bus.into(), baud.into(), 0, 0, 0) };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(IsaCanSocket { handle: bus.into() }),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* Drop trait implementation */

impl Drop for IsaCanSocket {
    fn drop(&mut self) {
        unsafe { peak_can::CAN_Uninitialize(self.handle) };
    }
}

/* Socket trait implementation */

impl Socket for IsaCanSocket {
    fn handle(&self) -> u16 {
        self.handle
    }
}

/* Channel trait implementation */

impl Channel for IsaCanSocket {
    fn channel(&self) -> u16 {
        self.handle
    }
}

/* CAN trait implementations */

impl HasRecvCan for IsaCanSocket {}
impl HasSendCan for IsaCanSocket {}

// impl HasRecvCanFd for IsaCanSocket {}
// impl HasSendCanFd for IsaCanSocket {}

/* HARDWARE IDENTIFICATION */

impl HasHardwareName for IsaCanSocket {}

impl HasControllerNumber for IsaCanSocket {}
impl HasSetControllerNumber for IsaCanSocket {}

impl HasDevicePartNumber for IsaCanSocket {}

/* INFORMATIONAL PARAMETER */

impl HasChannelVersion for IsaCanSocket {}

impl HasChannelFeatures for IsaCanSocket {}

impl HasBitrateInfo for IsaCanSocket {}

impl HasNominalBusSpeed for IsaCanSocket {}

impl HasDataBusSpeed for IsaCanSocket {}

impl HasFirmwareVersion for IsaCanSocket {}

/* SPECIAL BEHAVIOR */

/* CONTROLLING DATA FLOW */

impl HasMessageFilter for IsaCanSocket {}
impl HasSetMessageFilter for IsaCanSocket {}

impl HasReceiveStatus for IsaCanSocket {}
impl HasSetReceiveStatus for IsaCanSocket {}

impl HasAllowStatusFrames for IsaCanSocket {}
impl HasSetAllowStatusFrames for IsaCanSocket {}

impl HasAllowRTRFrames for IsaCanSocket {}
impl HasSetAllowRTRFrames for IsaCanSocket {}

impl HasAllowErrorFrames for IsaCanSocket {}
impl HasSetAllowErrorFrames for IsaCanSocket {}

impl HasAcceptanceFilter11Bit for IsaCanSocket {}
impl HasSetAcceptanceFilter11Bit for IsaCanSocket {}

impl HasAcceptanceFilter29Bit for IsaCanSocket {}
impl HasSetAcceptanceFilter29Bit for IsaCanSocket {}

/* TRACING PARAMETERS */

impl HasTraceLocation for IsaCanSocket {}
impl HasSetTraceLocation for IsaCanSocket {}

impl HasTraceStatus for IsaCanSocket {}
impl HasSetTraceStatus for IsaCanSocket {}

impl HasTraceSize for IsaCanSocket {}
impl HasSetTraceSize for IsaCanSocket {}

impl HasTraceConfigure for IsaCanSocket {}
impl HasSetTraceConfigure for IsaCanSocket {}
