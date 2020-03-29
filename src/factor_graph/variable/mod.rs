use std::fmt;
use uuid::Uuid;

pub mod landmark_variable_2d;
pub mod vehicle_variable_2d;

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
    fn update_pose(&self, update: Vec<f64>);
}
