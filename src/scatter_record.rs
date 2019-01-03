use crate::pdf::SingularPdf;

/// bundled informations to calculate the result of scattering, i.e., the next ray.
pub struct ScatterRecord {
    // A pdf to sample a direction of the scattered ray.
    pub pdf: SingularPdf,
}
