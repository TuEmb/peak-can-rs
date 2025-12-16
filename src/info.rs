//!
//!

use crate::channel::Channel;
use crate::error::{CanError, CanOkError};
use crate::peak_lib;
use crate::peak_can;
use std::ffi::c_void;

pub fn api_version() -> Result<String, CanError> {
    let mut data = [0u8; peak_can::MAX_LENGTH_VERSION_STRING as usize];
    let code = unsafe {
        peak_lib()?.CAN_GetValue(
            peak_can::PEAK_NONEBUS as u16,
            peak_can::PEAK_API_VERSION as u8,
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

#[derive(Debug, PartialEq, Clone)]
pub struct Version {
    pub device_driver_name_and_version: String,
    pub year_of_copyright: String,
    pub company_name_and_city: String,
}

/* ChannelVersion trait */

pub(crate) trait HasChannelVersion {}

pub trait ChannelVersion {
    fn channel_version(&self) -> Result<Version, CanError>;
}

impl<T: HasChannelVersion + Channel> ChannelVersion for T {
    fn channel_version(&self) -> Result<Version, CanError> {
        let mut data = [0u8; peak_can::MAX_LENGTH_VERSION_STRING as usize];
        let code = unsafe {
            peak_lib()?.CAN_GetValue(
                self.channel(),
                peak_can::PEAK_CHANNEL_VERSION as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => match std::str::from_utf8(&data) {
                Ok(s) => {
                    let newlines = s.lines().collect::<Vec<_>>();

                    if newlines.len() == 3 {
                        let newlines = newlines
                            .iter()
                            .map(|s| s.trim_matches(char::from(0)))
                            .collect::<Vec<_>>();

                        Ok(Version {
                            device_driver_name_and_version: String::from(newlines[0]),
                            year_of_copyright: String::from(newlines[1]),
                            company_name_and_city: String::from(newlines[2]),
                        })
                    } else {
                        Err(CanError::Unknown)
                    }
                }
                Err(_) => Err(CanError::Unknown),
            },
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* ChannelFeatures trait */

pub(crate) trait HasChannelFeatures {}

pub trait ChannelFeatures {
    fn is_fd_capable(&self) -> Result<bool, CanError>;
    fn is_delay_capable(&self) -> Result<bool, CanError>;
    fn is_io_capable(&self) -> Result<bool, CanError>;
}

impl<T: HasChannelFeatures + Channel> ChannelFeatures for T {
    fn is_fd_capable(&self) -> Result<bool, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_lib()?.CAN_GetValue(
                self.channel(),
                peak_can::PEAK_CHANNEL_FEATURES as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => {
                let value = u32::from_le_bytes(data);
                if value & peak_can::FEATURE_FD_CAPABLE == peak_can::FEATURE_FD_CAPABLE {
                    Ok(true)
                } else {
                    Ok(false)
                }
            }
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }

    fn is_delay_capable(&self) -> Result<bool, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_lib()?.CAN_GetValue(
                self.channel(),
                peak_can::PEAK_CHANNEL_FEATURES as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => {
                let value = u32::from_le_bytes(data);
                if value & peak_can::FEATURE_DELAY_CAPABLE == peak_can::FEATURE_DELAY_CAPABLE {
                    Ok(true)
                } else {
                    Ok(false)
                }
            }
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }

    fn is_io_capable(&self) -> Result<bool, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_lib()?.CAN_GetValue(
                self.channel(),
                peak_can::PEAK_CHANNEL_FEATURES as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => {
                let value = u32::from_le_bytes(data);
                if value & peak_can::FEATURE_IO_CAPABLE == peak_can::FEATURE_IO_CAPABLE {
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

/* BitrateInfo trait */

pub(crate) trait HasBitrateInfo {}

pub trait BitrateInfo {
    fn bitrate_info(&self) -> Result<(u16, u16), CanError>;
}

impl<T: HasBitrateInfo + Channel> BitrateInfo for T {
    fn bitrate_info(&self) -> Result<(u16, u16), CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_lib()?.CAN_GetValue(
                self.channel(),
                peak_can::PEAK_BITRATE_INFO as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => {
                let btr0 = u16::from_le_bytes([data[0], data[1]]);
                let btr1 = u16::from_le_bytes([data[2], data[3]]);
                Ok((btr0, btr1))
            }
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }
}

/* BitrateFdInfo trait */

pub(crate) trait HasBitrateInfoFd {}

pub trait BitrateInfoFd {
    fn bitrate_info_fd(&self) -> Result<String, CanError>;
}

impl<T: HasBitrateInfoFd + Channel> BitrateInfoFd for T {
    fn bitrate_info_fd(&self) -> Result<String, CanError> {
        let mut data = [0u8; peak_can::MAX_LENGTH_VERSION_STRING as usize];
        let code = unsafe {
            peak_lib()?.CAN_GetValue(
                self.channel(),
                peak_can::PEAK_BITRATE_INFO_FD as u8,
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

/* NominalBusSpeed trait */

pub(crate) trait HasNominalBusSpeed {}

pub trait NominalBusSpeed {
    fn nominal_bus_speed(&self) -> Result<u32, CanError>;
}

impl<T: HasNominalBusSpeed + Channel> NominalBusSpeed for T {
    fn nominal_bus_speed(&self) -> Result<u32, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_lib()?.CAN_GetValue(
                self.channel(),
                peak_can::PEAK_BUSSPEED_NOMINAL as u8,
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

/* DataBusSpeed trait */

pub(crate) trait HasDataBusSpeed {}

pub trait DataBusSpeed {
    fn data_bus_speed(&self) -> Result<u32, CanError>;
}

impl<T: HasDataBusSpeed + Channel> DataBusSpeed for T {
    fn data_bus_speed(&self) -> Result<u32, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_lib()?.CAN_GetValue(
                self.channel(),
                peak_can::PEAK_BUSSPEED_DATA as u8,
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

/* LAN SERVICE RUNNING / STOPPED */

pub fn lan_service_is_running() -> Result<bool, CanError> {
    let mut data = [0u8; 4];
    let code = unsafe {
        peak_lib()?.CAN_GetValue(
            peak_can::PEAK_NONEBUS as u16,
            peak_can::PEAK_LAN_SERVICE_STATUS as u8,
            data.as_mut_ptr() as *mut c_void,
            data.len() as u32,
        )
    };

    match CanOkError::try_from(code) {
        Ok(CanOkError::Ok) => {
            let code = u32::from_le_bytes(data);
            if code & peak_can::SERVICE_STATUS_RUNNING == peak_can::SERVICE_STATUS_RUNNING {
                Ok(true)
            } else {
                Ok(false)
            }
        }
        Ok(CanOkError::Err(err)) => Err(err),
        Err(_) => Err(CanError::Unknown),
    }
}

pub fn lan_service_is_stopped() -> Result<bool, CanError> {
    let mut data = [0u8; 4];
    let code = unsafe {
        peak_lib()?.CAN_GetValue(
            peak_can::PEAK_NONEBUS as u16,
            peak_can::PEAK_LAN_SERVICE_STATUS as u8,
            data.as_mut_ptr() as *mut c_void,
            data.len() as u32,
        )
    };

    match CanOkError::try_from(code) {
        Ok(CanOkError::Ok) => {
            let code = u32::from_le_bytes(data);
            if code & peak_can::SERVICE_STATUS_STOPPED == peak_can::SERVICE_STATUS_STOPPED {
                Ok(true)
            } else {
                Ok(false)
            }
        }
        Ok(CanOkError::Err(err)) => Err(err),
        Err(_) => Err(CanError::Unknown),
    }
}

/* FirmwareVersion trait */

pub(crate) trait HasFirmwareVersion {}

pub trait FirmwareVersion {
    fn firmware_version(&self) -> Result<String, CanError>;
}

impl<T: HasFirmwareVersion + Channel> FirmwareVersion for T {
    fn firmware_version(&self) -> Result<String, CanError> {
        let mut data = [0u8; 18usize];
        let code = unsafe {
            peak_lib()?.CAN_GetValue(
                self.channel(),
                peak_can::PEAK_FIRMWARE_VERSION as u8,
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
