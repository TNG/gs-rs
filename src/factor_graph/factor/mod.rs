use nalgebra::{DMatrix, VecStorage, Dynamic};

#[derive(Debug, Clone)]
pub enum Factor {
    /// x, y, phi, information_matrix
    PositionFactor2D(f64, f64, f64, InformationMatrix),

    // TODO add PositionPriorFactor2D?

    /// x, y, phi, information_matrix
    OdometryFactor2D(f64, f64, f64, InformationMatrix),

    /// x, y, phi, information_matrix
    ObservationFactor2D(f64, f64, f64, InformationMatrix),
}

#[derive(Debug, Clone)]
pub struct InformationMatrix {
    content: DMatrix<f64>,
}

impl From<Vec<f64>> for InformationMatrix {
    fn from(content: Vec<f64>) -> Self {
        let dim = (content.len() as f64).sqrt() as usize;
        InformationMatrix {
            content: DMatrix::from_data(VecStorage::new(Dynamic::new(dim), Dynamic::new(dim), content.into()))
        }
    }
}