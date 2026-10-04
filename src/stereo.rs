use std::path::Path;

use crate::{camera_model::CameraModel, sys, util::MrcalError};

// mrcal point to array conversions
impl From<[f64; 2]> for sys::mrcal_point2_t {
    fn from(val: [f64; 2]) -> Self {
        sys::mrcal_point2_t { xy: val }
    }
}

impl From<[f64; 3]> for sys::mrcal_point3_t {
    fn from(val: [f64; 3]) -> Self {
        sys::mrcal_point3_t { xyz: val }
    }
}

/// Hold camera models for a stereo rig.
pub struct StereoModel {
    left: CameraModel,
    right: CameraModel,
}

impl StereoModel {
    pub fn new(left: CameraModel, right: CameraModel) -> Self {
        Self { left, right }
    }

    pub fn read_from_files(
        left_path: impl AsRef<Path>,
        right_path: impl AsRef<Path>,
    ) -> Result<Self, MrcalError> {
        Ok(Self {
            left: CameraModel::read_from_file(left_path)?,
            right: CameraModel::read_from_file(right_path)?,
        })
    }

    pub fn left(&self) -> &CameraModel {
        &self.left
    }

    pub fn right(&self) -> &CameraModel {
        &self.right
    }

    pub fn rectify(&self, options: RectifiedSystemOptions) -> Result<RectifiedSystem, MrcalError> {
        RectifiedSystem::new(self, options)
    }
}

#[derive(Clone, Copy, Debug, Default)]
/// Available projections for rectified systems.
pub enum RectificationProjection {
    #[default]
    LatLon,
    Pinhole,
}

impl From<RectificationProjection> for sys::mrcal_lensmodel_type_t {
    fn from(val: RectificationProjection) -> Self {
        match val {
            RectificationProjection::Pinhole => sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_PINHOLE,
            RectificationProjection::LatLon => sys::mrcal_lensmodel_type_t_MRCAL_LENSMODEL_LATLON,
        }
    }
}

#[derive(Default)]
pub struct RectifiedSystemOptions {
    pixels_per_deg_az: Option<f64>,
    pixels_per_deg_el: Option<f64>,
    azel_fov_deg: [f64; 2],
    az0_deg: Option<f64>,
    el0_deg: Option<f64>,
    az_edge_margin_deg: Option<f64>,
    rectification_model_type: RectificationProjection,
}

impl RectifiedSystemOptions {
    pub fn new(azel_fov_deg: [f64; 2], rectification_model_type: RectificationProjection) -> Self {
        Self {
            azel_fov_deg,
            rectification_model_type,
            ..RectifiedSystemOptions::default()
        }
    }

    pub fn pixels_per_deg_az(mut self, val: f64) -> Self {
        self.pixels_per_deg_az = Some(val);
        self
    }

    pub fn pixels_per_deg_el(mut self, val: f64) -> Self {
        self.pixels_per_deg_el = Some(val);
        self
    }

    pub fn az0_deg(mut self, val: f64) -> Self {
        self.az0_deg = Some(val);
        self
    }

    pub fn el0_deg(mut self, val: f64) -> Self {
        self.el0_deg = Some(val);
        self
    }

    pub fn az_edge_margin_deg(mut self, val: f64) -> Self {
        self.az_edge_margin_deg = Some(val);
        self
    }
}

/// A particular rectification configuration for a stereo rig setup.
///
/// Intended for generating rectification maps and processing output
/// disparities.
pub struct RectifiedSystem {
    image_size: [u32; 2],
    intrinsics: [f64; 4],
    _rt_rect0_ref: [f64; 6],
    rt_cam0_rect0: [f64; 6],
    baseline: f64,
    projection: RectificationProjection,
    maps: Option<Box<[f32]>>,
}

impl RectifiedSystem {
    fn new(
        stereo_model: &StereoModel,
        options: RectifiedSystemOptions,
    ) -> Result<Self, MrcalError> {
        let mut image_size = [0u32; 2];
        // rectification projections only have 4 core intrinsics (PINHOLE and LATLON)
        let mut intrinsics = [0.0f64; 4];
        let mut rt_rect0_ref = [0.0f64; 6];
        let mut rt_cam0_rect0 = [0.0f64; 6];
        let mut baseline = 0.0f64;

        let mut pixels_per_deg_az = options.pixels_per_deg_az.unwrap_or(-1.0);
        let mut pixels_per_deg_el = options.pixels_per_deg_el.unwrap_or(-1.0);
        let mut azel_fov_deg = options.azel_fov_deg.into();
        let mut azel0_deg = [
            options.az0_deg.unwrap_or(0.0),
            options.el0_deg.unwrap_or(0.0),
        ]
        .into();
        let az0_deg_autodetect = options.az0_deg.is_none();
        let az_edge_margin_deg = options.az_edge_margin_deg.unwrap_or(10.0);

        unsafe {
            sys::mrcal_rectified_system2(
                image_size.as_mut_ptr(),
                intrinsics.as_mut_ptr(),
                rt_rect0_ref.as_mut_ptr(),
                &raw mut baseline,
                &raw mut pixels_per_deg_az,
                &raw mut pixels_per_deg_el,
                &raw mut azel_fov_deg,
                &raw mut azel0_deg,
                az_edge_margin_deg,
                stereo_model.left().lensmodel(),
                stereo_model.left().intrinsics_ptr(),
                stereo_model.left().rt_cam_ref().as_ptr(),
                stereo_model.right().rt_cam_ref().as_ptr(),
                options.rectification_model_type.into(),
                az0_deg_autodetect,
                false, // noop
                false, // noop
                false, // noop
            )
            .ok_or(MrcalError::Other(
                "error calling mrcal_rectified_system2".into(),
            ))?;
        }

        unsafe {
            sys::mrcal_compose_rt_full(
                rt_cam0_rect0.as_mut_ptr(),
                // contiguous
                0,
                // ignore all gradients
                std::ptr::null_mut(),
                0,
                0,
                std::ptr::null_mut(),
                0,
                0,
                std::ptr::null_mut(),
                0,
                0,
                std::ptr::null_mut(),
                0,
                0,
                std::ptr::null_mut(),
                0,
                0,
                std::ptr::null_mut(),
                0,
                0,
                // want cam0 <- rect0
                stereo_model.left.rt_cam_ref().as_ptr(),
                0,
                rt_rect0_ref.as_ptr(),
                0,
                false,
                true,
            );
        }

        let [width, height] = image_size;
        let map_bytes = (2 * width * height * 2) as usize;
        let mut maps = vec![0.0f32; map_bytes];

        unsafe {
            sys::mrcal_rectification_maps(
                maps.as_mut_ptr(),
                stereo_model.left().lensmodel(),
                stereo_model.left().intrinsics_ptr(),
                stereo_model.left().rt_cam_ref().as_ptr(),
                stereo_model.right().lensmodel(),
                stereo_model.right().intrinsics_ptr(),
                stereo_model.right().rt_cam_ref().as_ptr(),
                options.rectification_model_type.into(),
                intrinsics.as_ptr(),
                image_size.as_ptr(),
                rt_rect0_ref.as_ptr(),
            )
            .ok_or(MrcalError::Other(
                "error calling mrcal_rectification_maps".into(),
            ))?;
        }

        Ok(Self {
            image_size,
            intrinsics,
            _rt_rect0_ref: rt_rect0_ref,
            rt_cam0_rect0,
            baseline,
            projection: options.rectification_model_type,
            maps: Some(maps.into_boxed_slice()),
        })
    }

    /// Width of the rectified system.
    pub fn width(&self) -> usize {
        self.image_size[0] as usize
    }

    /// Height of the rectified system.
    pub fn height(&self) -> usize {
        self.image_size[1] as usize
    }

    /// Projection used by this rectification.
    pub fn projection(&self) -> RectificationProjection {
        self.projection
    }

    pub fn disparities_to_left_points(
        &self,
        disparities_and_qs: &[(f64, f64, f64)],
    ) -> Result<Vec<Option<[f64; 3]>>, MrcalError> {
        // generate rays
        let n = disparities_and_qs.len();
        let n_i32 =
            i32::try_from(n).map_err(|_| MrcalError::Other("too many disparities".into()))?;

        if n == 0 {
            return Ok(Vec::new());
        }

        let mut rays = vec![[0.0, 0.0, 0.0].into(); n];
        let qs = disparities_and_qs
            .iter()
            .map(|&(_, x, y)| [x, y].into())
            .collect::<Vec<_>>();

        let unproject = match self.projection {
            RectificationProjection::Pinhole => sys::mrcal_unproject_pinhole,
            RectificationProjection::LatLon => sys::mrcal_unproject_latlon,
        };

        unsafe {
            unproject(
                rays.as_mut_ptr(),
                std::ptr::null_mut(),
                qs.as_ptr(),
                n_i32,
                self.intrinsics.as_ptr(),
            );
        }

        // normalize all rays
        for ray in rays.iter_mut() {
            let [x, y, z] = unsafe { ray.xyz };
            let len = (x * x + y * y + z * z).sqrt();

            if !len.is_finite() || len == 0.0 {
                continue;
            }

            ray.xyz = [x / len, y / len, z / len];
        }

        // find ranges
        let mut ranges = vec![0.0; n];
        let disparities = disparities_and_qs
            .iter()
            .map(|&(d, _, _)| d)
            .collect::<Vec<_>>();

        unsafe {
            sys::mrcal_stereo_range_sparse(
                ranges.as_mut_ptr(),
                disparities.as_ptr(),
                qs.as_ptr(),
                n_i32,
                // these are just validity bounds, they don't affect range computation
                0.0,
                f64::MAX,
                self.projection.into(),
                self.intrinsics.as_ptr(),
                self.baseline,
            )
            .ok_or(MrcalError::Other(
                "error calling mrcal_stereo_range_sparse".into(),
            ))?;
        }

        // generate points
        Ok(rays
            .into_iter()
            .zip(ranges)
            .map(|(ray, range)| {
                if range == 0.0 {
                    return None;
                }

                // need to transform from rectified frame to cam0 (left) frame
                unsafe {
                    let mut point = [0.0; 3];
                    let [x, y, z] = ray.xyz;
                    let point_rect0 = [x * range, y * range, z * range];

                    sys::mrcal_transform_point_rt_full(
                        point.as_mut_ptr(),
                        // contiguous
                        0,
                        // ignore gradients
                        std::ptr::null_mut(),
                        0,
                        0,
                        std::ptr::null_mut(),
                        0,
                        0,
                        // pass transforms
                        self.rt_cam0_rect0.as_ptr(),
                        0,
                        point_rect0.as_ptr(),
                        0,
                        false,
                    );

                    Some(point)
                }
            })
            .collect())
    }

    /// Take the rectification maps.
    ///
    /// The returned memory contains two maps (left and right) stacked on top of one another.
    pub fn take_maps(&mut self) -> Option<Box<[f32]>> {
        self.maps.take()
    }
}
