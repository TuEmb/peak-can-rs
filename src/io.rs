//!
//!
//!
//!

/* IO DIGITAL CONFIGURATION trait */

use crate::channel::Channel;
use crate::error::{CanError, CanOkError};
use crate::peak_can;
use std::ffi::c_void;

#[derive(PartialEq, Debug)]
pub enum IOConfig {
    In,
    InOut,
}

impl TryFrom<u32> for IOConfig {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(IOConfig::In),
            1 => Ok(IOConfig::InOut),
            _ => Err(()),
        }
    }
}

impl From<IOConfig> for u32 {
    fn from(value: IOConfig) -> Self {
        match value {
            IOConfig::In => 0,
            IOConfig::InOut => 1,
        }
    }
}

pub(crate) trait HasDigitalConfiguration {}

pub trait DigitalConfiguration {
    fn digital_mode(&self, pin: u8) -> Result<IOConfig, CanError>;
    fn digital_mode_word(&self) -> Result<u32, CanError>;
}

impl<T: HasDigitalConfiguration + Channel> DigitalConfiguration for T {
    fn digital_mode(&self, pin: u8) -> Result<IOConfig, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_IO_DIGITAL_CONFIGURATION as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => {
                let mode_word = u32::from_le_bytes(data);
                let pin_enabled = mode_word & (1 << pin);

                if pin_enabled == 0 {
                    Ok(IOConfig::In)
                } else {
                    Ok(IOConfig::InOut)
                }
            }
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }

    fn digital_mode_word(&self) -> Result<u32, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_IO_DIGITAL_CONFIGURATION as u8,
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

pub(crate) trait HasSetDigitalConfiguration {}

pub trait SetDigitalConfiguration {
    fn set_digital_mode(&self, pin: u8, mode: IOConfig) -> Result<(), CanError>;
    fn set_digital_mode_word(&self, mode_word: u32) -> Result<(), CanError>;
}

impl<T: HasSetDigitalConfiguration + Channel> SetDigitalConfiguration for T {
    fn set_digital_mode(&self, pin: u8, mode: IOConfig) -> Result<(), CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_IO_DIGITAL_CONFIGURATION as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        let mode_word = match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => u32::from_le_bytes(data),
            Ok(CanOkError::Err(err)) => return Err(err),
            Err(_) => return Err(CanError::Unknown),
        };

        let mode_word = match mode {
            IOConfig::In => mode_word | !(1 << pin),
            IOConfig::InOut => mode_word | (1 << pin),
        };
        let mut data = mode_word.to_le_bytes();

        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_IO_DIGITAL_CONFIGURATION as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        return match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(()),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        };
    }

    fn set_digital_mode_word(&self, mode_word: u32) -> Result<(), CanError> {
        let mut data = mode_word.to_le_bytes();
        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_IO_DIGITAL_CONFIGURATION as u8,
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

/* IO DIGITAL VALUE */

#[derive(PartialEq, Debug)]
pub enum IOValue {
    Low,
    High,
}

impl TryFrom<u32> for IOValue {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(IOValue::Low),
            1 => Ok(IOValue::High),
            _ => Err(()),
        }
    }
}

impl From<IOValue> for u32 {
    fn from(value: IOValue) -> Self {
        match value {
            IOValue::Low => 0,
            IOValue::High => 1,
        }
    }
}

pub(crate) trait HasDigitalValue {}

pub trait DigitalValue {
    fn digital_value(&self, pin: u8) -> Result<IOValue, CanError>;
    fn digital_value_word(&self) -> Result<u32, CanError>;
}

impl<T: HasSetDigitalValue + Channel> DigitalValue for T {
    fn digital_value(&self, pin: u8) -> Result<IOValue, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_IO_DIGITAL_VALUE as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => {
                let mode_word = u32::from_le_bytes(data);
                let pin_enabled = mode_word & (1 << pin);

                if pin_enabled == 0 {
                    Ok(IOValue::Low)
                } else {
                    Ok(IOValue::High)
                }
            }
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        }
    }

    fn digital_value_word(&self) -> Result<u32, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_IO_DIGITAL_VALUE as u8,
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

pub(crate) trait HasSetDigitalValue {}

pub trait SetDigitalValue {
    fn set_digital_value(&self, pin: u8, value: IOValue) -> Result<(), CanError>;
    fn set_digital_value_word(&self, value_word: u32) -> Result<(), CanError>;
}

impl<T: HasSetDigitalValue + Channel> SetDigitalValue for T {
    fn set_digital_value(&self, pin: u8, value: IOValue) -> Result<(), CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_IO_DIGITAL_CONFIGURATION as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        let mode_word = match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => u32::from_le_bytes(data),
            Ok(CanOkError::Err(err)) => return Err(err),
            Err(_) => return Err(CanError::Unknown),
        };

        let mode_word = match value {
            IOValue::Low => mode_word | !(1 << pin),
            IOValue::High => mode_word | (1 << pin),
        };
        let mut data = mode_word.to_le_bytes();

        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_IO_DIGITAL_VALUE as u8,
                data.as_mut_ptr() as *mut c_void,
                data.len() as u32,
            )
        };

        return match CanOkError::try_from(code) {
            Ok(CanOkError::Ok) => Ok(()),
            Ok(CanOkError::Err(err)) => Err(err),
            Err(_) => Err(CanError::Unknown),
        };
    }

    fn set_digital_value_word(&self, value_word: u32) -> Result<(), CanError> {
        let mut data = value_word.to_le_bytes();
        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_IO_DIGITAL_VALUE as u8,
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

/* IO DIGITAL SET */

pub(crate) trait HasSetDigitalSet {}

pub trait SetDigitalSet {
    fn digital_set(&self, mask: u32) -> Result<(), CanError>;
}

impl<T: HasSetDigitalSet + Channel> SetDigitalSet for T {
    fn digital_set(&self, mask: u32) -> Result<(), CanError> {
        let mut data = mask.to_le_bytes();
        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_IO_DIGITAL_SET as u8,
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

/* IO DIGITAL CLEAR */

pub(crate) trait HasSetDigitalClear {}

pub trait SetDigitalClear {
    fn digital_clear(&self, mask: u32) -> Result<(), CanError>;
}

impl<T: HasSetDigitalClear + Channel> SetDigitalClear for T {
    fn digital_clear(&self, mask: u32) -> Result<(), CanError> {
        let mut data = mask.to_le_bytes();
        let code = unsafe {
            peak_can::CAN_SetValue(
                self.channel(),
                peak_can::PEAK_IO_DIGITAL_CLEAR as u8,
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

/* IO ANALOG VALUE */

pub(crate) trait HasAnalogValue {}

pub trait AnalogValue {
    fn analog_value(&self) -> Result<u32, CanError>;
}

impl<T: HasAnalogValue + Channel> AnalogValue for T {
    fn analog_value(&self) -> Result<u32, CanError> {
        let mut data = [0u8; 4];
        let code = unsafe {
            peak_can::CAN_GetValue(
                self.channel(),
                peak_can::PEAK_IO_ANALOG_VALUE as u8,
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
