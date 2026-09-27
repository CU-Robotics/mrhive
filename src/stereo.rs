use crate::{camera_model::CameraModel, sys, util::MrcalError};

// mrcal point to array conversions
impl Into<sys::mrcal_point2_t> for [f64; 2] {
    fn into(self) -> sys::mrcal_point2_t {
        sys::mrcal_point2_t { xy: self }
    }
}

impl Into<sys::mrcal_point3_t> for [f64; 3] {
    fn into(self) -> sys::mrcal_point3_t {
        sys::mrcal_point3_t { xyz: self }
    }
}

/// Hold camera models for a stereo rig.
pub struct StereoCamera {
    left: CameraModel,
    right: CameraModel,
}

impl StereoCamera {
    pub fn new(left: CameraModel, right: CameraModel) -> Self {
        Self { left, right }
    }

    pub fn left(&self) -> &CameraModel {
        &self.left
    }

    pub fn right(&self) -> &CameraModel {
        &self.right
    }
}

#[derive(Clone, Copy, Debug)]
/// Available projections for rectified systems.
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

/// A particular rectification configuration for a stereo rig setup.
///
/// Intended for generating rectification maps and processing output
/// disparities.
pub struct RectifiedSystem {
    image_size: [u32; 2],
    intrinsics: [f64; 4],
    rt_rect0_ref: [f64; 6],
    rt_cam0_rect0: [f64; 6],
    baseline: f64,
    options: RectifiedSystemOptions,
}

impl RectifiedSystem {
    pub fn new(
        stereo_camera: &StereoCamera,
        mut options: RectifiedSystemOptions,
    ) -> Result<Self, MrcalError> {
        let mut image_size = [0u32; 2];
        // rectification projections only have 4 core intrinsics (PINHOLE and LATLON)
        let mut intrinsics = [0.0f64; 4];
        let mut rt_rect0_ref = [0.0f64; 6];
        let mut rt_cam0_rect0 = [0.0f64; 6];
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
                stereo_camera.left().lensmodel(),
                stereo_camera.left().intrinsics_ptr(),
                stereo_camera.left().rt_cam_ref().as_ptr(),
                stereo_camera.right().rt_cam_ref().as_ptr(),
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
                stereo_camera.left.rt_cam_ref().as_ptr(),
                0,
                rt_rect0_ref.as_ptr(),
                0,
                false,
                true,
            );
        }

        Ok(Self {
            image_size,
            intrinsics,
            rt_rect0_ref,
            rt_cam0_rect0,
            baseline,
            options,
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

    pub fn disparities_to_left_points(
        &self,
        disparities_and_qs: &[(f64, f64, f64)],
    ) -> Result<Vec<Option<(f64, f64, f64)>>, MrcalError> {
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

        let unproject = match self.options.rectification_model_type {
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
                self.options.rectification_model_type.into(),
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
            .zip(ranges.into_iter())
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

                    Some(point.into())
                }
            })
            .collect())
    }

    /// Allocate, initialize, and return rectification maps that OpenCV can ingest.
    ///
    /// The returned memory contains two maps (left and right) stacked on top of one another.
    pub fn create_rectification_maps(
        &self,
        stereo_camera: &StereoCamera,
    ) -> Result<Box<[f32]>, MrcalError> {
        let [width, height] = self.image_size;
        let map_bytes = (2 * width * height * 2) as usize;
        let mut maps = vec![0.0f32; map_bytes];

        unsafe {
            sys::mrcal_rectification_maps(
                maps.as_mut_ptr(),
                stereo_camera.left().lensmodel(),
                stereo_camera.left().intrinsics_ptr(),
                stereo_camera.left().rt_cam_ref().as_ptr(),
                stereo_camera.right().lensmodel(),
                stereo_camera.right().intrinsics_ptr(),
                stereo_camera.right().rt_cam_ref().as_ptr(),
                self.options.rectification_model_type.into(),
                self.intrinsics.as_ptr(),
                self.image_size.as_ptr(),
                self.rt_rect0_ref.as_ptr(),
            )
            .ok_or(MrcalError::Other(
                "error calling mrcal_rectification_maps".into(),
            ))?;
        }

        Ok(maps.into_boxed_slice())
    }
}
