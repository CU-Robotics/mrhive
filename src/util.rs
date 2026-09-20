use thiserror::Error;

#[derive(Debug, Error)]
pub enum MrcalError {
    #[error("NulError: {0}")]
    NulError(#[from] std::ffi::NulError),
    #[error("{0}")]
    Other(String),
}
