use uuid::Uuid;
use std::fmt;

pub enum VariableType {
    /// position and rotation of the vehicle
    Pose2D,
    /// position and rotation of a landmark
    Landmark2D,
}

pub trait Variable<'a>: fmt::Debug {
    fn get_id(&self) -> Uuid;
    fn get_type(&self) -> VariableType;

    // TODO design interface for content
    fn get_content(&self);
}