use nalgebra::DMatrix;

#[derive(Debug, Clone, PartialEq)]
pub enum FactorType {
    Position2D,
    // TODO add PositionPrior2D?
    Odometry2D,
    Observation2D,
}

#[derive(Debug, Clone)]
pub struct Factor {
    pub factor_type: FactorType,
    pub constraint: Vec<f64>,
    pub information_matrix: InformationMatrix,
}

#[derive(Debug, Clone)]
pub struct InformationMatrix {
    pub content: DMatrix<f64>,
}

impl From<Vec<f64>> for InformationMatrix {
    fn from(content: Vec<f64>) -> Self {
        let dim = (content.len() as f64).sqrt() as usize;
        InformationMatrix {
            content: DMatrix::from_vec(dim, dim, content),
        }
    }
}