use crate::bus::Bus;
use crate::channel::Channel;
use crate::df::{HasReceiveStatus, HasSetReceiveStatus};
use crate::hw::{HasChannelCondition, HasControllerNumber, HasDevicePartNumber, HasHardwareName};
use crate::info::{HasBitrateInfo, HasBitrateInfoFd, HasChannelFeatures, HasChannelVersion};
use crate::peak_can;
use crate::special::HasFiveVoltsPower;

///
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum PccBus {
    ///
    PCC1,
    ///
    PCC2,
}

impl From<PccBus> for u16 {
    fn from(value: PccBus) -> Self {
        let ret = match value {
            PccBus::PCC1 => peak_can::PEAK_PCCBUS1,
            PccBus::PCC2 => peak_can::PEAK_PCCBUS2,
        } as u16;
        ret
    }
}

impl TryFrom<u16> for PccBus {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value as u32 {
            peak_can::PEAK_PCCBUS1 => Ok(PccBus::PCC1),
            peak_can::PEAK_PCCBUS2 => Ok(PccBus::PCC2),
            _ => Err(()),
        }
    }
}

/* Bus trait implementation */

impl Bus for PccBus {
    fn channel(&self) -> u16 {
        u16::from(*self)
    }
}

/* Channel trait implementation */

impl Channel for PccBus {
    fn channel(&self) -> u16 {
        Bus::channel(self)
    }
}

/* HARDWARE IDENTIFICATION */

impl HasChannelCondition for PccBus {}

impl HasHardwareName for PccBus {}

impl HasControllerNumber for PccBus {}

impl HasDevicePartNumber for PccBus {}

/* INFORMATIONAL PARAMETERS */

impl HasChannelVersion for PccBus {}

impl HasChannelFeatures for PccBus {}

impl HasBitrateInfo for PccBus {}

impl HasBitrateInfoFd for PccBus {}

/* SPECIAL BEHAVIOR */

impl HasFiveVoltsPower for PccBus {}

/* CONTROLLING DATA FLOW */

impl HasReceiveStatus for PccBus {}
impl HasSetReceiveStatus for PccBus {}
