//! Module provides two error type: [CanError] and [CanOkError].
//!
//! [CanError] models failure codes only whereas [CanOkError] also models the possibility of
//! success stated by the [Ok](CanOkError::Ok) variant.

use crate::peak_can;

///
#[derive(Debug, PartialEq)]
pub enum CanError {
    ///
    XmtFull,
    ///
    Overrun,
    ///
    BusLight,
    ///
    BusHeavy,
    ///
    BusPassive,
    ///
    BusOff,
    ///
    AnyBusErr,
    ///
    QrcvEmpty,
    ///
    QOverrun,
    ///
    QxmtFull,
    ///
    RegTest,
    ///
    NoDriver,
    ///
    HwInUse,
    ///
    NetInUse,
    ///
    IllHw,
    ///
    IllNet,
    ///
    IllClient,
    ///
    Resource,
    ///
    IllParamType,
    ///
    IllParamVal,
    ///
    Unknown,
    ///
    IllData,
    ///
    IllMode,
    ///
    Caution,
    ///
    Initialize,
    ///
    IllOperation,
}

/// Type modeling all possible states of an operation as exposed by [PEAK_basic_sys].
#[derive(Debug, PartialEq)]
pub enum CanOkError {
    /// Models the success of an operation.
    Ok,
    /// Models the failure. Similar to [CanError].
    Err(CanError),
}

impl From<CanError> for u32 {
    fn from(value: CanError) -> u32 {
        match value {
            CanError::XmtFull => peak_can::PEAK_ERROR_XMTFULL,
            CanError::Overrun => peak_can::PEAK_ERROR_OVERRUN,
            CanError::BusLight => peak_can::PEAK_ERROR_BUSLIGHT,
            CanError::BusHeavy => peak_can::PEAK_ERROR_BUSHEAVY,
            CanError::BusPassive => peak_can::PEAK_ERROR_BUSPASSIVE,
            CanError::BusOff => peak_can::PEAK_ERROR_BUSOFF,
            CanError::AnyBusErr => {
                let mut value = peak_can::PEAK_ERROR_BUSWARNING;
                value |= peak_can::PEAK_ERROR_BUSLIGHT;
                value |= peak_can::PEAK_ERROR_BUSHEAVY;
                value |= peak_can::PEAK_ERROR_BUSOFF;
                value |= peak_can::PEAK_ERROR_BUSPASSIVE;
                value
            }
            CanError::QrcvEmpty => peak_can::PEAK_ERROR_QRCVEMPTY,
            CanError::QOverrun => peak_can::PEAK_ERROR_QOVERRUN,
            CanError::QxmtFull => peak_can::PEAK_ERROR_QXMTFULL,
            CanError::RegTest => peak_can::PEAK_ERROR_REGTEST,
            CanError::NoDriver => peak_can::PEAK_ERROR_NODRIVER,
            CanError::HwInUse => peak_can::PEAK_ERROR_HWINUSE,
            CanError::NetInUse => peak_can::PEAK_ERROR_NETINUSE,
            CanError::IllHw => peak_can::PEAK_ERROR_ILLHW,
            CanError::IllNet => peak_can::PEAK_ERROR_ILLNET,
            CanError::IllClient => peak_can::PEAK_ERROR_ILLCLIENT,
            CanError::Resource => peak_can::PEAK_ERROR_RESOURCE,
            CanError::IllParamType => peak_can::PEAK_ERROR_ILLPARAMTYPE,
            CanError::IllParamVal => peak_can::PEAK_ERROR_ILLPARAMVAL,
            CanError::Unknown => peak_can::PEAK_ERROR_UNKNOWN,
            CanError::IllData => peak_can::PEAK_ERROR_ILLDATA,
            CanError::IllMode => peak_can::PEAK_ERROR_ILLMODE,
            CanError::Caution => peak_can::PEAK_ERROR_CAUTION,
            CanError::Initialize => peak_can::PEAK_ERROR_INITIALIZE,
            CanError::IllOperation => peak_can::PEAK_ERROR_ILLOPERATION,
        }
    }
}

impl From<CanOkError> for u32 {
    fn from(value: CanOkError) -> u32 {
        match value {
            CanOkError::Ok => peak_can::PEAK_ERROR_OK,
            CanOkError::Err(error) => u32::from(error),
        }
    }
}

impl TryFrom<u32> for CanError {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            peak_can::PEAK_ERROR_XMTFULL => Ok(CanError::XmtFull),
            peak_can::PEAK_ERROR_OVERRUN => Ok(CanError::Overrun),
            peak_can::PEAK_ERROR_BUSLIGHT => Ok(CanError::BusLight),
            peak_can::PEAK_ERROR_BUSHEAVY => Ok(CanError::BusHeavy),
            peak_can::PEAK_ERROR_BUSPASSIVE => Ok(CanError::BusPassive),
            peak_can::PEAK_ERROR_BUSOFF => Ok(CanError::BusOff),
            peak_can::PEAK_ERROR_ANYBUSERR => Ok(CanError::AnyBusErr),
            peak_can::PEAK_ERROR_QRCVEMPTY => Ok(CanError::QrcvEmpty),
            peak_can::PEAK_ERROR_QOVERRUN => Ok(CanError::QOverrun),
            peak_can::PEAK_ERROR_QXMTFULL => Ok(CanError::QxmtFull),
            peak_can::PEAK_ERROR_REGTEST => Ok(CanError::RegTest),
            peak_can::PEAK_ERROR_NODRIVER => Ok(CanError::NoDriver),
            peak_can::PEAK_ERROR_HWINUSE => Ok(CanError::HwInUse),
            peak_can::PEAK_ERROR_NETINUSE => Ok(CanError::NetInUse),
            peak_can::PEAK_ERROR_ILLHW => Ok(CanError::IllHw),
            peak_can::PEAK_ERROR_ILLNET => Ok(CanError::IllNet),
            peak_can::PEAK_ERROR_ILLCLIENT => Ok(CanError::IllClient),
            peak_can::PEAK_ERROR_RESOURCE => Ok(CanError::Resource),
            peak_can::PEAK_ERROR_ILLPARAMTYPE => Ok(CanError::IllParamType),
            peak_can::PEAK_ERROR_ILLPARAMVAL => Ok(CanError::IllParamVal),
            peak_can::PEAK_ERROR_UNKNOWN => Ok(CanError::Unknown),
            peak_can::PEAK_ERROR_ILLDATA => Ok(CanError::IllData),
            peak_can::PEAK_ERROR_ILLMODE => Ok(CanError::IllMode),
            peak_can::PEAK_ERROR_CAUTION => Ok(CanError::Caution),
            peak_can::PEAK_ERROR_INITIALIZE => Ok(CanError::Initialize),
            peak_can::PEAK_ERROR_ILLOPERATION => Ok(CanError::IllOperation),
            _ => Err(()),
        }
    }
}

impl TryFrom<u32> for CanOkError {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            peak_can::PEAK_ERROR_OK => Ok(CanOkError::Ok),
            _ => {
                let err = CanError::try_from(value)?;
                Ok(CanOkError::Err(err))
            }
        }
    }
}
