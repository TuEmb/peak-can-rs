//! Module provides two error type: [CanError] and [CanOkError].
//!
//! [CanError] models failure codes only whereas [CanOkError] also models the possibility of
//! success stated by the [Ok](CanOkError::Ok) variant.

use std::error::Error;
use std::fmt;
use std::sync::Arc;

use crate::peak_can;

///
#[derive(Debug, Clone)]
pub enum CanError {
    ///
    Libloading(Arc<libloading::Error>),
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
    /// A status code this crate does not recognise.
    ///
    /// PCAN status codes are a bit field, and not every value the driver can
    /// return has its own constant. `PCAN_ERROR_ANYBUSERR` is one the header
    /// does define (the union of the bus-error bits); a bus condition reported
    /// together with an orthogonal one such as `PCAN_ERROR_QOVERRUN` is
    /// another. Neither equals any single constant.
    ///
    /// Such a code is preserved here rather than collapsed into
    /// [`CanError::Unknown`], which would discard the diagnosis exactly when
    /// the most is going wrong. Inspect it with [`CanError::code`].
    Other(u32),
}

/// Type modeling all possible states of an operation as exposed by [PEAK_basic_sys].
#[derive(Debug)]
pub enum CanOkError {
    /// Models the success of an operation.
    Ok,
    /// Models the failure. Similar to [CanError].
    Err(CanError),
}

impl From<CanError> for u32 {
    fn from(value: CanError) -> u32 {
        match value {
            CanError::Libloading(_) => peak_can::PEAK_ERROR_UNKNOWN,
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
            CanError::Other(code) => code,
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
            // Not every value the driver returns has its own constant -- a bit
            // field can carry a combination, and PCAN_ERROR_ANYBUSERR is itself
            // a union. Keep the code rather than reporting `Unknown` and losing
            // what actually happened.
            peak_can::PEAK_ERROR_OK => Err(()),
            code => Ok(CanError::Other(code)),
        }
    }
}

impl CanError {
    /// The raw PCAN status code behind this error.
    ///
    /// Every variant maps back to the value the driver returned, so a caller
    /// can compare against a `PCAN_ERROR_*` constant directly -- including for
    /// [`CanError::Other`], where that is the only way to inspect it.
    ///
    /// [`CanError::Libloading`] reports `PCAN_ERROR_UNKNOWN`: the library never
    /// loaded, so no driver code was ever produced.
    pub fn code(&self) -> u32 {
        u32::from(self.clone())
    }
}

impl From<u32> for CanOkError {
    /// Decode a driver return code.
    ///
    /// Cannot fail: `PCAN_ERROR_OK` is [`CanOkError::Ok`] and every other value
    /// is an error, unrecognised codes included (see [`CanError::Other`]).
    fn from(value: u32) -> Self {
        match CanError::try_from(value) {
            Ok(err) => CanOkError::Err(err),
            Err(()) => CanOkError::Ok,
        }
    }
}


impl From<libloading::Error> for CanError {
    fn from(value: libloading::Error) -> Self {
        Self::Libloading(Arc::new(value))
    }
}

impl fmt::Display for CanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CanError::Libloading(e) => write!(f, "{e}"),
            CanError::XmtFull => write!(f, "xmt full"),
            CanError::Overrun => write!(f, "overrun"),
            CanError::BusLight => write!(f, "bus light"),
            CanError::BusHeavy => write!(f, "bus heavy"),
            CanError::BusPassive => write!(f, "bus passive"),
            CanError::BusOff => write!(f, "bus off"),
            CanError::AnyBusErr => write!(f, "any bus error"),
            CanError::QrcvEmpty => write!(f, "qrcv empty"),
            CanError::QOverrun => write!(f, "q overrun"),
            CanError::QxmtFull => write!(f, "qxmt full"),
            CanError::RegTest => write!(f, "reg test"),
            CanError::NoDriver => write!(f, "no driver"),
            CanError::HwInUse => write!(f, "hardware in use"),
            CanError::NetInUse => write!(f, "network in use"),
            CanError::IllHw => write!(f, "illegal hardware"),
            CanError::IllNet => write!(f, "illegal network"),
            CanError::IllClient => write!(f, "illegal client"),
            CanError::Resource => write!(f, "resource"),
            CanError::IllParamType => write!(f, "illegal parameter type"),
            CanError::IllParamVal => write!(f, "illegal parameter value"),
            CanError::Unknown => write!(f, "unknown"),
            CanError::IllData => write!(f, "illegal data"),
            CanError::IllMode => write!(f, "illegal mode"),
            CanError::Caution => write!(f, "caution"),
            CanError::Initialize => write!(f, "initialize"),
            CanError::IllOperation => write!(f, "illegal operation"),
            CanError::Other(code) => write!(f, "status {code:#x}"),
        }
    }
}

impl Error for CanError {}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every code with its own constant still decodes to its own variant.
    #[test]
    fn known_codes_keep_their_variant() {
        for (code, expected) in [
            (peak_can::PEAK_ERROR_BUSLIGHT, CanError::BusLight),
            (peak_can::PEAK_ERROR_BUSPASSIVE, CanError::BusPassive),
            (peak_can::PEAK_ERROR_BUSOFF, CanError::BusOff),
            (peak_can::PEAK_ERROR_QRCVEMPTY, CanError::QrcvEmpty),
            (peak_can::PEAK_ERROR_NODRIVER, CanError::NoDriver),
        ] {
            let decoded = CanError::try_from(code).expect("a non-zero code is an error");
            assert_eq!(
                decoded.code(),
                expected.code(),
                "code {code:#x} must keep its dedicated variant"
            );
        }
    }

    /// The bug this fixes: a code without its own constant used to fall through
    /// to `Err(())`, which callers turned into `Unknown` -- losing what the
    /// driver actually reported.
    #[test]
    fn an_unrecognised_code_keeps_its_value() {
        let code = peak_can::PEAK_ERROR_BUSPASSIVE | peak_can::PEAK_ERROR_QOVERRUN;
        let err = CanError::try_from(code).expect("still an error");

        assert!(matches!(err, CanError::Other(bits) if bits == code));
        assert_eq!(err.code(), code, "the round trip back to u32 is lossless");
    }

    /// `PCAN_ERROR_ANYBUSERR` is a constant the header defines, yet it equals
    /// none of the individual conditions -- it is the union of them. It used to
    /// decode to `AnyBusErr` only because that variant existed; any *other*
    /// union did not.
    #[test]
    fn any_bus_err_still_decodes_to_its_variant() {
        let err = CanError::try_from(peak_can::PEAK_ERROR_ANYBUSERR).unwrap();
        assert!(matches!(err, CanError::AnyBusErr));
    }

    /// Success is not an error.
    #[test]
    fn ok_is_not_an_error() {
        assert!(CanError::try_from(peak_can::PEAK_ERROR_OK).is_err());
        assert!(matches!(
            CanOkError::from(peak_can::PEAK_ERROR_OK),
            CanOkError::Ok
        ));
    }

    /// Decoding is now total, so an unrecognised code reaches the caller as an
    /// error rather than failing to convert.
    #[test]
    fn can_ok_error_decodes_every_code() {
        let code = peak_can::PEAK_ERROR_BUSOFF | peak_can::PEAK_ERROR_QOVERRUN;
        match CanOkError::from(code) {
            CanOkError::Err(err) => assert_eq!(err.code(), code),
            CanOkError::Ok => panic!("a non-zero code is not success"),
        }
    }

    /// `code()` answers for every variant, so a caller can always compare
    /// against a PCAN_ERROR_* constant.
    #[test]
    fn code_round_trips_for_known_variants() {
        assert_eq!(CanError::BusOff.code(), peak_can::PEAK_ERROR_BUSOFF);
        assert_eq!(CanError::QrcvEmpty.code(), peak_can::PEAK_ERROR_QRCVEMPTY);
        assert_eq!(CanError::Other(0x1234).code(), 0x1234);
    }

    #[test]
    fn unrecognised_code_displays_its_value() {
        assert_eq!(CanError::Other(0x50).to_string(), "status 0x50");
    }
}
