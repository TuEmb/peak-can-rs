use crate::bus::Bus;
use crate::channel::Channel;
use crate::df::{HasReceiveStatus, HasSetReceiveStatus};
use crate::hw::{
    HasChannelCondition, HasControllerNumber, HasDeviceId, HasDevicePartNumber, HasHardwareName,
    HasIpAddress,
};
use crate::info::{HasBitrateInfo, HasBitrateInfoFd, HasChannelFeatures, HasChannelVersion};
use crate::peak_can;

///
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum LanBus {
    ///
    LAN1,
    ///
    LAN2,
    ///
    LAN3,
    ///
    LAN4,
    ///
    LAN5,
    ///
    LAN6,
    ///
    LAN7,
    ///
    LAN8,
    ///
    LAN9,
    ///
    LAN10,
    ///
    LAN11,
    ///
    LAN12,
    ///
    LAN13,
    ///
    LAN14,
    ///
    LAN15,
    ///
    LAN16,
}

impl From<LanBus> for u16 {
    fn from(value: LanBus) -> Self {
        let ret = match value {
            LanBus::LAN1 => peak_can::PEAK_LANBUS1,
            LanBus::LAN2 => peak_can::PEAK_LANBUS2,
            LanBus::LAN3 => peak_can::PEAK_LANBUS3,
            LanBus::LAN4 => peak_can::PEAK_LANBUS4,
            LanBus::LAN5 => peak_can::PEAK_LANBUS5,
            LanBus::LAN6 => peak_can::PEAK_LANBUS6,
            LanBus::LAN7 => peak_can::PEAK_LANBUS7,
            LanBus::LAN8 => peak_can::PEAK_LANBUS8,
            LanBus::LAN9 => peak_can::PEAK_LANBUS9,
            LanBus::LAN10 => peak_can::PEAK_LANBUS10,
            LanBus::LAN11 => peak_can::PEAK_LANBUS11,
            LanBus::LAN12 => peak_can::PEAK_LANBUS12,
            LanBus::LAN13 => peak_can::PEAK_LANBUS13,
            LanBus::LAN14 => peak_can::PEAK_LANBUS14,
            LanBus::LAN15 => peak_can::PEAK_LANBUS15,
            LanBus::LAN16 => peak_can::PEAK_LANBUS16,
        } as u16;
        ret
    }
}

impl TryFrom<u16> for LanBus {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value as u32 {
            peak_can::PEAK_LANBUS1 => Ok(LanBus::LAN1),
            peak_can::PEAK_LANBUS2 => Ok(LanBus::LAN2),
            peak_can::PEAK_LANBUS3 => Ok(LanBus::LAN3),
            peak_can::PEAK_LANBUS4 => Ok(LanBus::LAN4),
            peak_can::PEAK_LANBUS5 => Ok(LanBus::LAN5),
            peak_can::PEAK_LANBUS6 => Ok(LanBus::LAN6),
            peak_can::PEAK_LANBUS7 => Ok(LanBus::LAN7),
            peak_can::PEAK_LANBUS8 => Ok(LanBus::LAN8),
            peak_can::PEAK_LANBUS9 => Ok(LanBus::LAN9),
            peak_can::PEAK_LANBUS10 => Ok(LanBus::LAN10),
            peak_can::PEAK_LANBUS11 => Ok(LanBus::LAN11),
            peak_can::PEAK_LANBUS12 => Ok(LanBus::LAN12),
            peak_can::PEAK_LANBUS13 => Ok(LanBus::LAN13),
            peak_can::PEAK_LANBUS14 => Ok(LanBus::LAN14),
            peak_can::PEAK_LANBUS15 => Ok(LanBus::LAN15),
            peak_can::PEAK_LANBUS16 => Ok(LanBus::LAN16),
            _ => Err(()),
        }
    }
}

/* Bus trait implementation */

impl Bus for LanBus {
    fn channel(&self) -> u16 {
        u16::from(*self)
    }
}

/* Channel trait implementation */

impl Channel for LanBus {
    fn channel(&self) -> u16 {
        Bus::channel(self)
    }
}

/* HARDWARE IDENTIFICATION */

impl HasChannelCondition for LanBus {}

impl HasDeviceId for LanBus {}

impl HasHardwareName for LanBus {}

impl HasControllerNumber for LanBus {}

impl HasIpAddress for LanBus {}

impl HasDevicePartNumber for LanBus {}

/* INFORMATIONAL PARAMETERS */

impl HasChannelVersion for LanBus {}

impl HasChannelFeatures for LanBus {}

impl HasBitrateInfo for LanBus {}

impl HasBitrateInfoFd for LanBus {}

/* SPECIAL BEHAVIOR */

/* CONTROLLING DATA FLOW */

impl HasReceiveStatus for LanBus {}
impl HasSetReceiveStatus for LanBus {}
