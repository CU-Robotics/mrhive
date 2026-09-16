use crate::sys;
use std::ptr;

pub struct CameraModel {
    pub(crate) handle: ptr::NonNull<sys::mrcal_cameramodel_VOID_t>,
}
