use crate::bus::Bus;
use crate::channel::Channel;
use crate::df::{HasReceiveStatus, HasSetReceiveStatus};
use crate::hw::{
    HasChannelCondition, HasChannelIdentifying, HasControllerNumber, HasDeviceId,
    HasDevicePartNumber, HasHardwareName,
};
use crate::info::{HasBitrateInfo, HasBitrateInfoFd, HasChannelFeatures, HasChannelVersion};
use crate::peak_can;
use crate::special::HasFiveVoltsPower;

///
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum UsbBus {
    ///
    USB1,
    ///
    USB2,
    ///
    USB3,
    ///
    USB4,
    ///
    USB5,
    ///
    USB6,
    ///
    USB7,
    ///
    USB8,
    ///
    USB9,
    ///
    USB10,
    ///
    USB11,
    ///
    USB12,
    ///
    USB13,
    ///
    USB14,
    ///
    USB15,
    ///
    USB16,
}

impl From<UsbBus> for u16 {
    fn from(value: UsbBus) -> Self {
        let ret = match value {
            UsbBus::USB1 => peak_can::PEAK_USBBUS1,
            UsbBus::USB2 => peak_can::PEAK_USBBUS2,
            UsbBus::USB3 => peak_can::PEAK_USBBUS3,
            UsbBus::USB4 => peak_can::PEAK_USBBUS4,
            UsbBus::USB5 => peak_can::PEAK_USBBUS5,
            UsbBus::USB6 => peak_can::PEAK_USBBUS6,
            UsbBus::USB7 => peak_can::PEAK_USBBUS7,
            UsbBus::USB8 => peak_can::PEAK_USBBUS8,
            UsbBus::USB9 => peak_can::PEAK_USBBUS9,
            UsbBus::USB10 => peak_can::PEAK_USBBUS10,
            UsbBus::USB11 => peak_can::PEAK_USBBUS11,
            UsbBus::USB12 => peak_can::PEAK_USBBUS12,
            UsbBus::USB13 => peak_can::PEAK_USBBUS13,
            UsbBus::USB14 => peak_can::PEAK_USBBUS14,
            UsbBus::USB15 => peak_can::PEAK_USBBUS15,
            UsbBus::USB16 => peak_can::PEAK_USBBUS16,
        } as u16;
        ret
    }
}

impl TryFrom<u16> for UsbBus {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value as u32 {
            peak_can::PEAK_USBBUS1 => Ok(UsbBus::USB1),
            peak_can::PEAK_USBBUS2 => Ok(UsbBus::USB2),
            peak_can::PEAK_USBBUS3 => Ok(UsbBus::USB3),
            peak_can::PEAK_USBBUS4 => Ok(UsbBus::USB4),
            peak_can::PEAK_USBBUS5 => Ok(UsbBus::USB5),
            peak_can::PEAK_USBBUS6 => Ok(UsbBus::USB6),
            peak_can::PEAK_USBBUS7 => Ok(UsbBus::USB7),
            peak_can::PEAK_USBBUS8 => Ok(UsbBus::USB8),
            peak_can::PEAK_USBBUS9 => Ok(UsbBus::USB9),
            peak_can::PEAK_USBBUS10 => Ok(UsbBus::USB10),
            peak_can::PEAK_USBBUS11 => Ok(UsbBus::USB11),
            peak_can::PEAK_USBBUS12 => Ok(UsbBus::USB12),
            peak_can::PEAK_USBBUS13 => Ok(UsbBus::USB13),
            peak_can::PEAK_USBBUS14 => Ok(UsbBus::USB14),
            peak_can::PEAK_USBBUS15 => Ok(UsbBus::USB15),
            peak_can::PEAK_USBBUS16 => Ok(UsbBus::USB16),
            _ => Err(()),
        }
    }
}

/* Bus trait implementation */

impl Bus for UsbBus {
    fn channel(&self) -> u16 {
        u16::from(*self)
    }
}

/* Channel trait implementation */

impl Channel for UsbBus {
    fn channel(&self) -> u16 {
        Bus::channel(self)
    }
}

/* HARDWARE IDENTIFICATION */

impl HasChannelCondition for UsbBus {}

impl HasChannelIdentifying for UsbBus {}

impl HasDeviceId for UsbBus {}

impl HasHardwareName for UsbBus {}

impl HasControllerNumber for UsbBus {}

impl HasDevicePartNumber for UsbBus {}

/* INFORMATIONAL PARAMETERS */

impl HasChannelVersion for UsbBus {}

impl HasChannelFeatures for UsbBus {}

impl HasBitrateInfo for UsbBus {}

impl HasBitrateInfoFd for UsbBus {}

/* SPECIAL BEHAVIOR */

impl HasFiveVoltsPower for UsbBus {}

/* CONTROLLING DATA FLOW */

impl HasReceiveStatus for UsbBus {}
impl HasSetReceiveStatus for UsbBus {}
