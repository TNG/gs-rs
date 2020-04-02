//! The internal representation of a factor graph's optimizable variable.

use std::fmt;
use uuid::Uuid;

pub mod landmark_variable_2d;
pub mod vehicle_variable_2d;

/// Enum representing a supported variable type.
#[derive(Debug, PartialEq)]
pub enum VariableType {
    /// Vehicle pose (position and rotation) in 2D.
    Vehicle2D,
    /// Vehicle pose (position and rotation) in 2D.
    Landmark2D,
}

/// Trait with expected functions that all variables should implement.
pub trait Variable<'a>: fmt::Debug {
    /// The variable's ID. Should not change.
    fn get_id(&self) -> Uuid;
    /// The variable's type. Should not change.
    fn get_type(&self) -> VariableType;
    /// The variable's pose. May be subject to change, unless the variable is fixed.
    /// Content for 2D variables: vec![pose_position_x, pose_position_y, pose_rotation]
    fn get_pose(&self) -> Vec<f64>;
    /// Whether the variable is fixed or not. Fixed variables' poses are not subject to change.
    fn is_fixed(&self) -> bool;
    // TODO rename to set_pose or maybe even change interface completely?
    /// Sets a new variable pose.
    fn update_pose(&self, update: Vec<f64>);
    /// Make the variable fixed. Fixed variables' poses are not subject to change.
    fn make_fixed(&self);
}
