// gs-rs - Graph SLAM in Rust
// --------------
//
// © 2020 Samuel Valenzuela (samuel.valenzuela@tngtech.com)
// © 2020 Florian Rohm (florian.rohm@tngtech.com)
// © 2020 Daniel Pape (daniel.pape@tngtech.com)
//
// This product includes software developed at
// TNG Technology Consulting GmbH (https://www.tngtech.com/).
//
// gs-rs is licensed under the Apache License, Version 2.0 (LICENSE-APACHE.md or
// http://www.apache.org/licenses/LICENSE-2.0) or the MIT license (LICENSE-MIT.md
// or http://opensource.org/licenses/MIT), at your option.
//! The internal representation of a factor graph's measurement.

use nalgebra::DMatrix;

/// Enum representing a supported factor type.
#[derive(Debug, Clone, PartialEq)]
pub enum FactorType {
    /// Vehicle pose measurement in 2D.
    Position2D,
    /// Relative measurement between two poses in 2D.
    Odometry2D,
    /// Relative measurement to an observed stationary variable in 2D.
    Observation2D,
    /// Vehicle pose measurement in 3D.
    Position3D,
    /// Relative measurement between two poses in 3D.
    Odometry3D,
    /// Relative measurement to an observed stationary variable in 3D.
    Observation3D,
}

/// Structure representing a measurement.
#[derive(Debug, Clone)]
pub struct Factor {
    /// The factor's type.
    pub factor_type: FactorType,
    /// The factor's constraint.
    ///
    /// Content for Position2D and Odometry2D: vec![position_x, position_y, rotation]
    ///
    /// Content for Observation2D: vec![position_x, position_y]
    ///
    /// Content for Position3D and Odometry3D: vec![position_x, position_y, position_z, rotation_quaternion_x, rotation_quaternion_y, rotation_quaternion_z, rotation_quaternion_w]
    ///
    /// Content for Observation3D: vec![position_x, position_y, position_z]
    pub constraint: Vec<f64>,
    /// The factor's wrapped information matrix, equalling the inverse of the factor's mean matrix.
    pub information_matrix: InformationMatrix,
}

/// Structure wrapping the information matrix of a factor.
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
