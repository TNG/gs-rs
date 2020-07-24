//! The internal representation of a factor graph's optimizable variable.

use std::cell::RefCell;
use std::ops::Range;
use std::rc::Rc;

pub mod landmark_variable_2d;
pub mod vehicle_variable_2d;

pub mod landmark_variable_3d;
pub mod vehicle_variable_3d;

#[derive(Debug, Eq, PartialEq)]
pub enum FixedType {
    Fixed,
    NonFixed(Range<usize>),
}


/// Representation of an optimizable vehicle variable.
#[derive(Debug)]
pub struct VehicleVariable2D {
    pub id: usize,
    pub pose: Rc<RefCell<[f64; 3]>>,
    pub fixed_type: FixedType,
}

/// Representation of an optimizable landmark variable.
#[derive(Debug)]
pub struct LandmarkVariable2D {
    pub id: usize,
    pub position: Rc<RefCell<[f64; 2]>>,
    pub fixed_type: FixedType,
}

/// Representation of an optimizable vehicle variable.
#[derive(Debug)]
pub struct VehicleVariable3D {
    pub id: usize,
    pub pose: Rc<RefCell<[f64; 7]>>,
    pub fixed_type: FixedType,
}

/// Representation of an optimizable landmark variable.
#[derive(Debug)]
pub struct LandmarkVariable3D {
    pub id: usize,
    pub pose: Rc<RefCell<[f64; 3]>>,
    pub fixed_type: FixedType,
}

/// Enum representing a supported variable type.
#[derive(Debug, PartialEq)]
pub enum Variable {
    /// Vehicle pose (position and rotation) in 2D.
    Vehicle2D(VehicleVariable2D),
    /// Landmark position in 2D.
    Landmark2D(LandmarkVariable2D),
    /// Vehicle pose (position and rotation) in 3D.
    Vehicle3D(VehicleVariable3D),
    /// Landmark position in 3D.
    Landmark3D(LandmarkVariable3D),
}

impl VehicleVariable2D {
    /// Returns a new variable from a 2D pose, a given ID and whether the variable is fixed.
    pub fn new(id: usize, x: f64, y: f64, phi: f64, fixed: bool, optional_range: Option<Range<usize>>) -> Self {
        VehicleVariable2D {
            id,
            pose: Rc::new(RefCell::new([x, y, phi])),
            fixed_type: if fixed { FixedType::Fixed } else { FixedType::NonFixed(optional_range.unwrap()) }
        }
    }
}
impl LandmarkVariable2D {
    /// Returns a new variable from a 2D position, a given ID and whether the variable is fixed.
    pub fn new(id: usize, x: f64, y: f64, fixed: bool, optional_range: Option<Range<usize>>) -> Self {
        LandmarkVariable2D {
            id,
            position: Rc::new(RefCell::new([x, y])),
            fixed_type: if fixed { FixedType::Fixed } else { FixedType::NonFixed(optional_range.unwrap()) }
        }
    }
}

impl VehicleVariable3D {
    /// Returns a new variable from a 3D pose, a given ID and whether the variable is fixed.
    pub fn new(id: usize, x: f64, y: f64, z: f64, rot_x: f64, rot_y: f64, rot_z: f64, rot_w: f64, fixed: bool, optional_range: Option<Range<usize>>) -> Self {
        VehicleVariable3D {
            id,
            pose: Rc::new(RefCell::new([x, y, z, rot_x, rot_y, rot_z, rot_w])),
            fixed_type: if fixed { FixedType::Fixed } else { FixedType::NonFixed(optional_range.unwrap()) }
        }
    }
}

impl LandmarkVariable3D {
    /// Returns a new variable from a 3D position, a given ID and whether the variable is fixed.
    pub fn new(id: usize, x: f64, y: f64, z: f64, fixed: bool, optional_range: Option<Range<usize>>) -> Self {
        LandmarkVariable3D {
            id,
            pose: Rc::new(RefCell::new([x, y, z])),
            fixed_type: if fixed { FixedType::Fixed } else { FixedType::NonFixed(optional_range.unwrap()) }
        }
    }
}
