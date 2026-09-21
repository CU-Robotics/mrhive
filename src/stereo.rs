use crate::{camera_model::CameraModel, sys, util::MrcalError};

pub enum RectificationProjection {
    Pinhole,
    LatLon,
}

impl Into<sys::mrcal_lensmodel_type_t> for RectificationProjection {
    fn into(self) -> sys::mrcal_lensmodel_type_t {
        match self {
            Self::Pinhole => sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_PINHOLE,
            Self::LatLon => sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_LATLON,
        }
    }
}

pub struct RectifiedSystemOptions {
    pixels_per_deg_az: f64,
    pixels_per_deg_el: f64,
    azel_fov_deg: [f64; 2],
    azel0_deg: [f64; 2],
    az_edge_margin_deg: f64,
    rectification_model_type: RectificationProjection,
    az0_deg_autodetect: bool,
}

impl Into<sys::mrcal_point2_t> for [f64; 2] {
    fn into(self) -> sys::mrcal_point2_t {
        sys::mrcal_point2_t { xy: self }
    }
}

pub struct RectifiedSystem {
    image_size: [u32; 2],
    intrinsics: [f64; 4],
    rt_rect0_ref: [f64; 6],
    baseline: f64,
}

impl RectifiedSystem {
    pub fn new(
        left: &CameraModel,
        right: &CameraModel,
        mut options: RectifiedSystemOptions,
    ) -> Result<Self, MrcalError> {
        let mut image_size = [0u32; 2];
        // rectification projections only have 4 core intrinsics (PINHOLE and LATLON)
        let mut intrinsics = [0.0f64; 4];
        let mut rt_rect0_ref = [0.0f64; 6];
        let mut baseline = 0.0f64;
        let mut azel_fov_deg = options.azel_fov_deg.into();
        let mut azel0_deg = options.azel0_deg.into();

        unsafe {
            sys::mrcal_rectified_system2(
                image_size.as_mut_ptr(),
                intrinsics.as_mut_ptr(),
                rt_rect0_ref.as_mut_ptr(),
                &raw mut baseline,
                &raw mut options.pixels_per_deg_az,
                &raw mut options.pixels_per_deg_el,
                &raw mut azel_fov_deg,
                &raw mut azel0_deg,
                options.az_edge_margin_deg,
                left.lensmodel(),
                left.intrinsics_ptr(),
                left.rt_cam_ref().as_ptr(),
                right.rt_cam_ref().as_ptr(),
                options.rectification_model_type.into(),
                options.az0_deg_autodetect,
                false, // noop
                false, // noop
                false, // noop
            )
            .ok_or(MrcalError::Other(
                "error calling mrcal_rectified_system2".into(),
            ))?;
        }

        Ok(Self {
            image_size,
            intrinsics,
            rt_rect0_ref,
            baseline,
        })
    }

    pub fn create_maps(self) -> Result<(), MrcalError> {
        unimplemented!()
    }
}

pub struct RectificationMaps {}
