use crate::bus::Bus;
use crate::channel::Channel;
use crate::df::{HasReceiveStatus, HasSetReceiveStatus};
use crate::hw::{HasChannelCondition, HasControllerNumber, HasDevicePartNumber, HasHardwareName};
use crate::info::{HasBitrateInfo, HasBitrateInfoFd, HasChannelFeatures, HasChannelVersion};
use crate::peak_can;

///
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum IsaBus {
    ///
    ISA1,
    ///
    ISA2,
    ///
    ISA3,
    ///
    ISA4,
    ///
    ISA5,
    ///
    ISA6,
    ///
    ISA7,
    ///
    ISA8,
}

impl From<IsaBus> for u16 {
    fn from(value: IsaBus) -> Self {
        let ret = match value {
            IsaBus::ISA1 => peak_can::PEAK_ISABUS1,
            IsaBus::ISA2 => peak_can::PEAK_ISABUS2,
            IsaBus::ISA3 => peak_can::PEAK_ISABUS3,
            IsaBus::ISA4 => peak_can::PEAK_ISABUS4,
            IsaBus::ISA5 => peak_can::PEAK_ISABUS5,
            IsaBus::ISA6 => peak_can::PEAK_ISABUS6,
            IsaBus::ISA7 => peak_can::PEAK_ISABUS7,
            IsaBus::ISA8 => peak_can::PEAK_ISABUS8,
        } as u16;
        ret
    }
}

impl TryFrom<u16> for IsaBus {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value as u32 {
            peak_can::PEAK_ISABUS1 => Ok(IsaBus::ISA1),
            peak_can::PEAK_ISABUS2 => Ok(IsaBus::ISA2),
            peak_can::PEAK_ISABUS3 => Ok(IsaBus::ISA3),
            peak_can::PEAK_ISABUS4 => Ok(IsaBus::ISA4),
            peak_can::PEAK_ISABUS5 => Ok(IsaBus::ISA5),
            peak_can::PEAK_ISABUS6 => Ok(IsaBus::ISA6),
            peak_can::PEAK_ISABUS7 => Ok(IsaBus::ISA7),
            peak_can::PEAK_ISABUS8 => Ok(IsaBus::ISA8),
            _ => Err(()),
        }
    }
}

/* Bus trait implementation */

impl Bus for IsaBus {
    fn channel(&self) -> u16 {
        u16::from(*self)
    }
}

/* Channel trait implementation */

impl Channel for IsaBus {
    fn channel(&self) -> u16 {
        Bus::channel(self)
    }
}

/* HARDWARE IDENTIFICATION */

impl HasChannelCondition for IsaBus {}

impl HasHardwareName for IsaBus {}

impl HasControllerNumber for IsaBus {}

impl HasDevicePartNumber for IsaBus {}

/* INFORMATIONAL PARAMETERS */

impl HasChannelVersion for IsaBus {}

impl HasChannelFeatures for IsaBus {}

impl HasBitrateInfo for IsaBus {}

impl HasBitrateInfoFd for IsaBus {}

/* SPECIAL BEHAVIOR */

/* CONTROLLING DATA FLOW */

impl HasReceiveStatus for IsaBus {}
impl HasSetReceiveStatus for IsaBus {}
