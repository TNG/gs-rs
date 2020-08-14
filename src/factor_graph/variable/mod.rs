//! The internal representation of a factor graph's optimizable variable.

use std::fmt;
use std::ops::Range;

pub mod vehicle_variable_2d;
pub mod landmark_variable_2d;

pub mod vehicle_variable_3d;
pub mod landmark_variable_3d;

use std::cell::RefCell;
use std::rc::Rc;

#[derive(Debug, Eq, PartialEq)]
pub enum FixedType {
    Fixed,
    NonFixed(Range<usize>),
}


/// Representation of an optimizable vehicle variable.
#[derive(Debug)]
pub struct VehicleVariable2D {
    id: usize,
    pose: Rc<RefCell<[f64; 3]>>,
    fixed_type: FixedType,
}

/// Representation of an optimizable landmark variable.
#[derive(Debug)]
pub struct LandmarkVariable2D {
    id: usize,
    position: Rc<RefCell<[f64; 2]>>,
    fixed_type: FixedType,
}

/// Representation of an optimizable vehicle variable.
#[derive(Debug)]
pub struct VehicleVariable3D {
    id: usize,
    pose: Rc<RefCell<[f64; 7]>>,
    fixed_type: FixedType,
}

/// Representation of an optimizable landmark variable.
#[derive(Debug)]
pub struct LandmarkVariable3D {
    id: usize,
    pose: Rc<RefCell<[f64; 3]>>,
    fixed_type: FixedType,

}




impl VehicleVariable2D {
    /// Returns a new variable from a 2D pose, a given ID and whether the variable is fixed.
    pub fn new(id: usize, x: f64, y: f64, phi: f64, fixed_type: FixedType) -> Self {
        VehicleVariable2D {
            id,
            pose: Rc::new(RefCell::new([x, y, phi])),
            fixed_type,

        }
    }
}

impl LandmarkVariable2D {
    /// Returns a new variable from a 2D position, a given ID and whether the variable is fixed.
    pub fn new(id: usize, x: f64, y: f64, fixed_type: FixedType) -> Self {
        LandmarkVariable2D {
            id,
            position: Rc::new(RefCell::new([x, y])),
            fixed_type,
        }
    }
}

impl VehicleVariable3D {
    /// Returns a new variable from a 3D pose, a given ID and whether the variable is fixed.
    pub fn new(id: usize, x: f64, y: f64, z: f64, rot_x: f64, rot_y: f64, rot_z: f64, rot_w: f64, fixed_type: FixedType) -> Self {
        VehicleVariable3D {
            id,
            pose: Rc::new(RefCell::new([x, y, z, rot_x, rot_y, rot_z, rot_w])),
            fixed_type,
        }
    }
}

impl LandmarkVariable3D {
    /// Returns a new variable from a 3D position, a given ID and whether the variable is fixed.
    pub fn new(id: usize, x: f64, y: f64, z: f64, fixed_type: FixedType) -> Self {
        LandmarkVariable3D {
            id,
            pose: Rc::new(RefCell::new([x, y, z])),
            fixed_type,
        }
    }
}
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
    /// Whether the variable is fixed or not. Fixed variables' poses are not subject to change whereas non fixed variables allow for an index range in H and b
    fn get_fixed_type(&self) -> &FixedType;
    /// Sets a new variable pose.
    fn set_content(&self, update: Vec<f64>);
}


impl Variable<'_> for VehicleVariable3D {
    fn get_id(&self) -> usize {
        self.id
    }

    fn get_type(&self) -> VariableType {
        VariableType::Vehicle3D
    }

    fn get_content(&self) -> Vec<f64> {
        (*self.pose.borrow_mut()).to_vec()
    }

    fn get_fixed_type(&self) -> &FixedType {
        &self.fixed_type
    }

    fn set_content(&self, update: Vec<f64>) {
        *self.pose.borrow_mut() = [update[0], update[1], update[2], update[3], update[4], update[5], update[6]];
    }
}

impl Variable<'_> for VehicleVariable2D {
    fn get_id(&self) -> usize {
        self.id
    }

    fn get_type(&self) -> VariableType {
        VariableType::Vehicle2D
    }

    fn get_content(&self) -> Vec<f64> {
        (*self.pose.borrow_mut()).to_vec()
    }


    fn get_fixed_type(&self) -> &FixedType {
        &self.fixed_type
    }

    fn set_content(&self, update: Vec<f64>) {
        *self.pose.borrow_mut() = [update[0], update[1], update[2]];
    }
}
impl Variable<'_> for LandmarkVariable3D {
    fn get_id(&self) -> usize {
        self.id
    }

    fn get_type(&self) -> VariableType {
        VariableType::Landmark3D
    }

    fn get_content(&self) -> Vec<f64> {
        (*self.pose.borrow_mut()).to_vec()
    }


    fn get_fixed_type(&self) -> &FixedType {
        &self.fixed_type
    }

    fn set_content(&self, update: Vec<f64>) {
        *self.pose.borrow_mut() = [update[0], update[1], update[2]];
    }
}


impl Variable<'_> for LandmarkVariable2D {
    fn get_id(&self) -> usize {
        self.id
    }

    fn get_type(&self) -> VariableType {
        VariableType::Landmark2D
    }

    fn get_content(&self) -> Vec<f64> {
        (*self.position.borrow_mut()).to_vec()
    }

    fn get_fixed_type(&self) -> &FixedType {
        &self.fixed_type
    }

    fn set_content(&self, update: Vec<f64>) {
        *self.position.borrow_mut() = [update[0], update[1]];
    }
}