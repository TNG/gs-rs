use uuid::Uuid;
use std::fmt;

pub mod vehicle_variable_2d;
pub mod landmark_variable_2d;

#[derive(Debug, PartialEq)]
pub enum VariableType {
    /// position and rotation of the vehicle
    Vehicle2D,
    /// position and rotation of a landmark
    Landmark2D,
}

pub trait Variable<'a>: fmt::Debug {
    fn get_id(&self) -> Uuid;
    fn get_type(&self) -> VariableType;
    fn get_pose(&self) -> Vec<f64>;
}