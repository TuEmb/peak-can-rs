//!
//!
//!

use crate::channel::Channel;
use crate::error::{CanError, CanOkError};
use crate::peak_can;
use std::ffi::c_void;
use std::mem::size_of;
use std::net::Ipv4Addr;
use std::os::raw::c_char;

#[derive(Debug, PartialEq)]
pub enum ChannelConditionStatus {
    Unavailable,
    Available,
    Occupied,
    CanView,
}

impl From<ChannelConditionStatus> for u32 {
    fn from(value: ChannelConditionStatus) -> Self {
        match value {
            ChannelConditionStatus::Unavailable => peak_can::PEAK_CHANNEL_UNAVAILABLE,
            ChannelConditionStatus::Available => peak_can::PEAK_CHANNEL_AVAILABLE,
            ChannelConditionStatus::Occupied => peak_can::PEAK_CHANNEL_OCCUPIED,
            ChannelConditionStatus::CanView => peak_can::PEAK_CHANNEL_VIEW,
        }
    }
}

impl TryFrom<u32> for ChannelConditionStatus {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            peak_can::PEAK_CHANNEL_AVAILABLE => Ok(ChannelConditionStatus::Available),
            peak_can::PEAK_CHANNEL_UNAVAILABLE => Ok(ChannelConditionStatus::Unavailable),
            peak_can::PEAK_CHANNEL_OCCUPIED => Ok(ChannelConditionStatus::Occupied),
            peak_can::PEAK_CHANNEL_VIEW => Ok(ChannelConditionStatus::CanView),
            _ => Err(()),
        }
    }
}

/* ChannelCondition trait */

pub(crate) trait HasChannelCondition {}

pub trait ChannelCondition {
    fn channel_condition(&self) -> Result<ChannelConditionStatus, CanError>;
}

impl<T: HasChannelCondition + Channel> ChannelCondition for T {
    fn channel_condition(&self) -> Result<ChannelConditionStatus, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_CHANNEL_CONDITION as u8,
                data.as_mut_ptr() as *mut c_void,
                4,
            )
        };

        let value: u32 = u32::from_le_bytes(data);
        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => match ChannelConditionStatus::try_from(value) {
                Ok(status) => Ok(status),
                Err(_) => Err(CanError::Unknown),
            },
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* ChannelIdentifying trait */

pub(crate) trait HasChannelIdentifying {}

pub trait ChannelIdentifying {
    fn set_channel_identifying(&self, value: bool) -> Result<(), CanError>;
    fn is_channel_identifying(&self) -> Result<bool, CanError>;
}

impl<T: HasChannelIdentifying + Channel> ChannelIdentifying for T {
    fn set_channel_identifying(&self, value: bool) -> Result<(), CanError> {
        let mut data = match value {
            true => peak_can::PEAK_PARAMETER_ON.to_le_bytes(),
            false => peak_can::PEAK_PARAMETER_OFF.to_le_bytes(),
        };

        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_CHANNEL_IDENTIFYING as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(()),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }

    fn is_channel_identifying(&self) -> Result<bool, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_CHANNEL_IDENTIFYING as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => {
                let value = u32::from_le_bytes(data);
                if value & peak_can::PEAK_PARAMETER_ON == peak_can::PEAK_PARAMETER_ON {
                    Ok(true)
                } else {
                    Ok(false)
                }
            }
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* Device Id */

pub(crate) trait HasDeviceId {}

pub trait DeviceId {
    fn device_id(&self) -> Result<u32, CanError>;
}

impl<T: HasDeviceId + Channel> DeviceId for T {
    fn device_id(&self) -> Result<u32, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_DEVICE_ID as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(u32::from_le_bytes(data)),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

pub(crate) trait HasSetDeviceId {}

pub trait SetDeviceId {
    type Item;
    fn set_device_id(&self, value: Self::Item) -> Result<(), CanError>;
}

impl<T: HasSetDeviceId + Channel> SetDeviceId for T {
    type Item = u32;
    fn set_device_id(&self, value: Self::Item) -> Result<(), CanError> {
        let mut data = value.to_le_bytes();
        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_DEVICE_ID as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(()),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* Hardware Name */

pub(crate) trait HasHardwareName {}

pub trait HardwareName {
    fn hardware_name(&self) -> Result<String, CanError>;
}

impl<T: HasHardwareName + Channel> HardwareName for T {
    fn hardware_name(&self) -> Result<String, CanError> {
        let mut data = [0u8; peak_can::MAX_LENGTH_HARDWARE_NAME as usize];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_HARDWARE_NAME as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => match std::str::from_utf8(&data) {
                Ok(s) => {
                    let s = s.trim_matches(char::from(0));
                    Ok(String::from(s))
                }
                Err(_) => Err(CanError::Unknown),
            },
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* Controller Number */

pub(crate) trait HasControllerNumber {}

pub trait ControllerNumber {
    fn controller_number(&self) -> Result<u32, CanError>;
}

impl<T: HasControllerNumber + Channel> ControllerNumber for T {
    fn controller_number(&self) -> Result<u32, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_CONTROLLER_NUMBER as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(u32::from_le_bytes(data)),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

pub(crate) trait HasSetControllerNumber {}

pub trait SetControllerNumber {
    type Item;
    fn set_controller_number(&self, value: Self::Item) -> Result<(), CanError>;
}

impl<T: HasSetControllerNumber + Channel> SetControllerNumber for T {
    type Item = u32;
    fn set_controller_number(&self, value: Self::Item) -> Result<(), CanError> {
        let mut data = value.to_le_bytes();
        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_CONTROLLER_NUMBER as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(()),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* IpAddress trait */

pub(crate) trait HasIpAddress {}

pub trait IpAddress {
    fn ip_address(&self) -> Result<Ipv4Addr, CanError>;
}

impl<T: HasIpAddress + Channel> IpAddress for T {
    fn ip_address(&self) -> Result<Ipv4Addr, CanError> {
        let mut data = [0u8; 20];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_IP_ADDRESS as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => match std::str::from_utf8(&data) {
                Ok(s) => {
                    let s = s.trim_matches(char::from(0));
                    match s.parse() {
                        Ok(ip) => Ok(ip),
                        Err(_) => Err(CanError::Unknown),
                    }
                }
                Err(_) => Err(CanError::Unknown),
            },
            Ok(CanOkError::Err(err)) => Err(err),
            _ => Err(CanError::Unknown),
        }
    }
}

/* ATTACHED CHANNEL COUNT */

pub fn attached_channels_count() -> Result<u32, CanError> {
    let mut data = [0u8; 4];
    let code = unsafe {
        peak_can::CAN_GetValue(
            peak_can::PEAK_NONEBUS as u16,
            peak_can::PEAK_ATTACHED_CHANNELS_COUNT as u8,
            data.as_mut_ptr() as *mut c_void,
            data.len() as u32,
        )
    };

    match CanOkError::try_from(code) {
        Ok(CanOkError::Ok) => Ok(u32::from_le_bytes(data)),
        Ok(CanOkError::Err(err)) => Err(err),
        Err(_) => Err(CanError::Unknown),
    }
}

/* ATTACHED CHANNELS */

#[derive(Debug)]
pub struct ChannelInformation {
    pub channel_information: peak_can::tagCANTPChannelInformation,
}

impl ChannelInformation {
    pub fn new() -> Self {
        ChannelInformation {
            channel_information: peak_can::tagCANTPChannelInformation {
                channel_handle: 0,
                device_type: 0,
                controller_number: 0,
                device_features: 0,
                device_name: [c_char::default(); 33usize],
                device_id: 0,
                channel_condition: 0,
            },
        }
    }

    pub fn device_name(&self) -> String {
        let string = self
            .channel_information
            .device_name
            .as_slice()
            .iter()
            .map(|c| char::from(*c as u8))
            .collect::<Vec<_>>()
            .as_slice()
            .iter()
            .collect::<String>();

        let s = string.trim_matches(char::from(0));
        String::from(s)
    }
}

pub fn attached_channels() -> Result<Vec<ChannelInformation>, CanError> {
    let attached_channels_count = attached_channels_count()?;
    let mut channel_information_list = Vec::new();

    for _ in 0..attached_channels_count {
        channel_information_list.push(ChannelInformation::new());
    }

    let code = unsafe {
        peak_can::CAN_GetValue(
            peak_can::PEAK_NONEBUS as u16,
            peak_can::PEAK_ATTACHED_CHANNELS as u8,
            channel_information_list.as_mut_ptr() as *mut c_void,
            attached_channels_count * size_of::<peak_can::tagCANTPChannelInformation>() as u32,
        )
    };

    match CanOkError::try_from(code) {
        Ok(CanOkError::Ok) => Ok(channel_information_list),
        Ok(CanOkError::Err(err)) => Err(err),
        Err(_) => Err(CanError::Unknown),
    }
}

/* DevicePartNumber trait */

pub(crate) trait HasDevicePartNumber {}

pub trait DevicePartNumber {
    fn device_part_number(&self) -> Result<String, CanError>;
}

impl<T: HasDevicePartNumber + Channel> DevicePartNumber for T {
    fn device_part_number(&self) -> Result<String, CanError> {
        let mut data = [0u8; 100];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_DEVICE_NUMBER as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => match std::str::from_utf8(&data) {
                Ok(s) => {
                    let s = s.trim_matches(char::from(0));
                    Ok(String::from(s))
                }
                Err(_) => Err(CanError::Unknown),
            },
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}
