use crate::aliases::Vec3;
use crate::pdf::SingularPdf;

/// bundled informations to calculate the result of scattering, i.e., the next ray.
pub struct ScatterRecord {
    pub attenuation: Vec3,
    // A pdf for importance sampling method,
    // i.e., a function on directions whose value at d is higher when
    // d should be frequently sampled as the direction of scattered rays for efficient rendering.
    pub important_dir: SingularPdf,
}
