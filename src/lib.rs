//!
//!

#[warn(dead_code)]
pub mod bus;
mod channel;
pub mod df;
pub mod error;
pub mod hw;
pub mod info;
pub mod io;
pub mod log;
pub mod socket;
pub mod special;
pub mod trace;

use peak_can_sys as peak_can;

use std::sync::LazyLock;

static PEAK_BASIC: LazyLock<Result<peak_can::Pcan, crate::error::CanError>> = LazyLock::new(|| {
    let filename = libloading::library_filename("PCANBasic");
    Ok(unsafe { peak_can::Pcan::new(filename) }?)
});

pub(crate) fn peak_lib() -> Result<&'static peak_can::Pcan, crate::error::CanError> {
    PEAK_BASIC.as_ref().map_err(|e| e.clone())
}
