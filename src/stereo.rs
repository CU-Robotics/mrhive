use crate::sys;

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
