use crate::pdf::SingularPdf;

/// Informations to calculate the scattered ray.
pub struct ScatterRecord {
    // A pdf to sample a direction of the scattered ray.
    pub pdf: SingularPdf,
}
