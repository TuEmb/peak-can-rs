//!
//!
//!

use crate::channel::Channel;
use crate::error::{CanError, CanOkError};
use crate::peak_can;
use std::ffi::c_void;

/* Five Volts Power */

pub(crate) trait HasFiveVoltsPower {}

pub trait FiveVoltsPower {
    fn five_volts(&self) -> Result<bool, CanError>;
}

impl<T: HasFiveVoltsPower + Channel> FiveVoltsPower for T {
    fn five_volts(&self) -> Result<bool, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_5VOLTS_POWER as u8,
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

pub(crate) trait HasSetFiveVoltsPower {}

pub trait SetFiveVoltsPower {
    fn set_five_volts(&self, value: bool) -> Result<(), CanError>;
}

impl<T: HasSetFiveVoltsPower + Channel> SetFiveVoltsPower for T {
    fn set_five_volts(&self, value: bool) -> Result<(), CanError> {
        let mut data = match value {
            true => peak_can::PEAK_PARAMETER_ON.to_le_bytes(),
            false => peak_can::PEAK_PARAMETER_OFF.to_le_bytes(),
        };
        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_5VOLTS_POWER as u8,
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

/* Bus Off Autoreset */

pub(crate) trait HasBusOffAutoreset {}

pub trait BusOffAutoreset {
    fn bus_off_autoreset(&self) -> Result<bool, CanError>;
}

impl<T: HasBusOffAutoreset + Channel> BusOffAutoreset for T {
    fn bus_off_autoreset(&self) -> Result<bool, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_BUSOFF_AUTORESET as u8,
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

pub(crate) trait HasSetBusOffAutoreset {}

pub trait SetBusOffAutoreset {
    fn set_bus_off_autoreset(&self, value: bool) -> Result<(), CanError>;
}

impl<T: HasSetBusOffAutoreset + Channel> SetBusOffAutoreset for T {
    fn set_bus_off_autoreset(&self, value: bool) -> Result<(), CanError> {
        let mut data = match value {
            true => peak_can::PEAK_PARAMETER_ON.to_le_bytes(),
            false => peak_can::PEAK_PARAMETER_OFF.to_le_bytes(),
        };
        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_BUSOFF_AUTORESET as u8,
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

/* Listen Only */

pub(crate) trait HasListenOnly {}

pub trait ListenOnly {
    fn listen_only(&self) -> Result<bool, CanError>;
}

impl<T: HasListenOnly + Channel> ListenOnly for T {
    fn listen_only(&self) -> Result<bool, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_LISTEN_ONLY as u8,
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

pub(crate) trait HasSetListenOnly {}

pub trait SetListenOnly {
    fn set_listen_only(&self, value: bool) -> Result<(), CanError>;
}

impl<T: HasSetListenOnly + Channel> SetListenOnly for T {
    fn set_listen_only(&self, value: bool) -> Result<(), CanError> {
        let mut data = match value {
            true => peak_can::PEAK_PARAMETER_ON.to_le_bytes(),
            false => peak_can::PEAK_PARAMETER_OFF.to_le_bytes(),
        };
        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_LISTEN_ONLY as u8,
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

/* Bitrate Adapting */

pub(crate) trait HasBitrateAdapting {}

pub trait BitrateAdapting {
    fn bitrate_adapting(&self) -> Result<bool, CanError>;
}

impl<T: HasBitrateAdapting + Channel> BitrateAdapting for T {
    fn bitrate_adapting(&self) -> Result<bool, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_BITRATE_ADAPTING as u8,
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

pub(crate) trait HasSetBitrateAdapting {}

pub trait SetBitrateAdapting {
    fn set_bitrate_adapting(&self, value: bool) -> Result<(), CanError>;
}

impl<T: HasSetBitrateAdapting + Channel> SetBitrateAdapting for T {
    fn set_bitrate_adapting(&self, value: bool) -> Result<(), CanError> {
        let mut data = match value {
            true => peak_can::PEAK_PARAMETER_ON.to_le_bytes(),
            false => peak_can::PEAK_PARAMETER_OFF.to_le_bytes(),
        };
        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_BITRATE_ADAPTING as u8,
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

/* Interframe Delay */

pub(crate) trait HasInterframeDelay {}

pub trait InterframeDelay {
    fn interframe_delay(&self) -> Result<u32, CanError>;
}

impl<T: HasInterframeDelay + Channel> InterframeDelay for T {
    fn interframe_delay(&self) -> Result<u32, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_INTERFRAME_DELAY as u8,
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

pub(crate) trait HasSetInterframeDelay {}

pub trait SetInterframeDelay {
    fn set_interframe_delay(&self, value: u32) -> Result<(), CanError>;
}

impl<T: HasSetInterframeDelay + Channel> SetInterframeDelay for T {
    fn set_interframe_delay(&self, value: u32) -> Result<(), CanError> {
        let mut data = value.to_le_bytes();
        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_INTERFRAME_DELAY as u8,
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
