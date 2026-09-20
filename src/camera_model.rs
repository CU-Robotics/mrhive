use crate::sys;
use crate::util::MrcalError;
use std::ffi::CString;
use std::os::unix::ffi::OsStrExt;
use std::path::Path;
use std::ptr::NonNull;

/// Loads and stores an mrcal `.cameramodel` for further use.
pub struct CameraModel {
    pub(crate) handle: NonNull<sys::mrcal_cameramodel_VOID_t>,
}

impl CameraModel {
    /// Loads a `.cameramodel` from disk.
    pub fn read_from_file(path: impl AsRef<Path>) -> Result<Self, MrcalError> {
        let c_path = CString::new(path.as_ref().as_os_str().as_bytes())?;

        let model = unsafe { sys::mrcal_read_cameramodel_file(c_path.as_ptr()) };

        if model.is_null() {
            return Err(MrcalError::Other(format!(
                "could not load cameramodel: {}",
                path.as_ref().display(),
            )));
        }

        // model was checked for NULL (failure to load) before this, so unwrapping is fine
        Ok(Self {
            handle: NonNull::new(model).unwrap(),
        })
    }

    /// Exposes the underlying lens model contained in the camera model.
    pub(crate) fn lensmodel(&self) -> &sys::mrcal_lensmodel_t {
        let model = unsafe { self.handle.as_ref() };
        let intrinsics = unsafe { model.__bindgen_anon_1.i.as_ref() };

        &intrinsics.lensmodel
    }

    /// Exposes the underlying intrinsics array contained in the camera model.
    pub(crate) fn intrinsics_ptr(&self) -> *const f64 {
        let model = unsafe { self.handle.as_ref() };
        let intrinsics = unsafe { model.__bindgen_anon_1.i.as_ref() };

        intrinsics.intrinsics.as_ptr()
    }

    /// Exposes the underlying reference frame contained in the camera model.
    pub(crate) fn rt_cam_ref(&self) -> &[f64; 6] {
        let model = unsafe { self.handle.as_ref() };
        &model.rt_cam_ref
    }
}

impl Drop for CameraModel {
    fn drop(&mut self) {
        let mut ptr = self.handle.as_ptr();
        unsafe { sys::mrcal_free_cameramodel(&raw mut ptr) };
    }
}
