//!
//!
//!

use crate::bus::PccBus;
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
use crate::special::{HasFiveVoltsPower, HasSetFiveVoltsPower};
use crate::trace::{
    HasSetTraceConfigure, HasSetTraceLocation, HasSetTraceSize, HasSetTraceStatus,
    HasTraceConfigure, HasTraceLocation, HasTraceSize, HasTraceStatus,
};

#[derive(Debug, PartialEq)]
pub struct PccCanSocket {
    handle: u16,
}

impl PccCanSocket {
    pub fn open(bus: PccBus, baud: Baudrate) -> Result<PccCanSocket, CanError> {
        let handle = bus.into();
        let code = unsafe { peak_can::CAN_Initialize(handle, baud.into(), 0, 0, 0) };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(PccCanSocket { handle }),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* Drop trait implementation */

impl Drop for PccCanSocket {
    fn drop(&mut self) {
        unsafe { peak_can::CAN_Uninitialize(self.handle) };
    }
}

/* Socket trait implementation */

impl Socket for PccCanSocket {
    fn handle(&self) -> u16 {
        self.handle
    }
}

/* Channel trait implementation */

impl Channel for PccCanSocket {
    fn channel(&self) -> u16 {
        self.handle
    }
}

/* CAN trait implementations */

impl HasRecvCan for PccCanSocket {}
impl HasSendCan for PccCanSocket {}

// impl HasRecvCanFd for PccCanSocket {}
// impl HasSendCanFd for PccCanSocket {}

/* HARDWARE IDENTIFICATION */

impl HasHardwareName for PccCanSocket {}

impl HasControllerNumber for PccCanSocket {}
impl HasSetControllerNumber for PccCanSocket {}

impl HasDevicePartNumber for PccCanSocket {}

/* INFORMATIONAL PARAMETER */

impl HasChannelVersion for PccCanSocket {}

impl HasChannelFeatures for PccCanSocket {}

impl HasBitrateInfo for PccCanSocket {}

impl HasNominalBusSpeed for PccCanSocket {}

impl HasDataBusSpeed for PccCanSocket {}

impl HasFirmwareVersion for PccCanSocket {}

/* SPECIAL BEHAVIOR */

impl HasFiveVoltsPower for PccCanSocket {}
impl HasSetFiveVoltsPower for PccCanSocket {}

/* CONTROLLING DATA FLOW */

impl HasMessageFilter for PccCanSocket {}
impl HasSetMessageFilter for PccCanSocket {}

impl HasReceiveStatus for PccCanSocket {}
impl HasSetReceiveStatus for PccCanSocket {}

impl HasAllowStatusFrames for PccCanSocket {}
impl HasSetAllowStatusFrames for PccCanSocket {}

impl HasAllowRTRFrames for PccCanSocket {}
impl HasSetAllowRTRFrames for PccCanSocket {}

impl HasAllowErrorFrames for PccCanSocket {}
impl HasSetAllowErrorFrames for PccCanSocket {}

impl HasAcceptanceFilter11Bit for PccCanSocket {}
impl HasSetAcceptanceFilter11Bit for PccCanSocket {}

impl HasAcceptanceFilter29Bit for PccCanSocket {}
impl HasSetAcceptanceFilter29Bit for PccCanSocket {}

/* TRACING PARAMETERS */

impl HasTraceLocation for PccCanSocket {}
impl HasSetTraceLocation for PccCanSocket {}

impl HasTraceStatus for PccCanSocket {}
impl HasSetTraceStatus for PccCanSocket {}

impl HasTraceSize for PccCanSocket {}
impl HasSetTraceSize for PccCanSocket {}

impl HasTraceConfigure for PccCanSocket {}
impl HasSetTraceConfigure for PccCanSocket {}
