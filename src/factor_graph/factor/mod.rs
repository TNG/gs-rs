use uuid::Uuid;
use std::fmt;

pub mod position_prior_factor_2d;
pub mod position_factor_2d;
pub mod odometry_factor_2d;
pub mod observation_factor_2d;

#[derive(Debug, PartialEq)]
pub enum FactorType {
    /// Necessary to fix graph for underdetermined systems
    PositionPrior2D,
    /// Corresponds to a GPS measurement
    Position2D,
    /// Corresponds to an odometry measurement
    Odometry2D,
    /// Corresponds to an observation measurement of a landmark
    Observation2D,
}

pub trait Factor<'a>: fmt::Debug + Clone {
    fn get_id(&self) -> Uuid;
    fn get_type(&self) -> FactorType;
    fn get_restriction(&self) -> &[f64];
    fn get_information_matrix(&self) -> &[f64];
}