//! The internal representation of a factor graph's optimizable variable.

use std::fmt;
use std::ops::Range;

pub mod vehicle_variable_2d;
pub mod landmark_variable_2d;

pub mod vehicle_variable_3d;
pub mod landmark_variable_3d;

/// Enum representing a supported variable type.
#[derive(Debug, PartialEq)]
pub enum VariableType {
    /// Vehicle pose (position and rotation) in 2D.
    Vehicle2D,
    /// Landmark position in 2D.
    Landmark2D,
    /// Vehicle pose (position and rotation) in 3D.
    Vehicle3D,
    /// Landmark position in 3D.
    Landmark3D,
}

/// Trait with expected functions that all variables should implement.
pub trait Variable<'a>: fmt::Debug {
    /// The variable's ID. Should not change.
    fn get_id(&self) -> usize;
    /// The variable's type. Should not change.
    fn get_type(&self) -> VariableType;
    /// The variable's pose or position. May be subject to change, unless the variable is fixed.
    ///
    /// Content for 2D vehicle variables: vec![position_x, position_y, rotation]
    ///
    /// Content for 2D landmark variables: vec![position_x, position_y]
    ///
    /// Content for 3D vehicle variables: vec![position_x, position_y, position_z, rotation_quaternion_x, rotation_quaternion_y, rotation_quaternion_z, rotation_quaternion_w]
    ///
    /// Content for 3D landmark variables: vec![position_x, position_y, position_z]
    fn get_content(&self) -> Vec<f64>;
    /// Whether the variable is fixed or not. Fixed variables' poses are not subject to change.
    fn is_fixed(&self) -> bool;
    /// The variable's index range in H and b. Fixed variables do not have such a range.
    fn get_range(&self) -> Option<Range<usize>>;
    /// Sets a new variable pose.
    fn set_content(&self, update: Vec<f64>);
}
