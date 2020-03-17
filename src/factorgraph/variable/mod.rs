use uuid::Uuid;
use std::fmt;

pub mod pose_variable_2d;
pub mod landmark_variable_2d;

#[derive(Debug, PartialEq)]
pub enum VariableType {
    /// position and rotation of the vehicle
    Pose2D,
    /// position and rotation of a landmark TODO change to position of a landmark OR include phi in Landmark2D and ObservationFactor2D
    Landmark2D,
}

pub trait Variable<'a>: fmt::Debug {
    fn get_id(&self) -> Uuid;
    fn get_type(&self) -> VariableType;
    fn get_content(&self) -> Vec<f64>;
}