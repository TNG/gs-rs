use crate::factor_graph::variable::{Variable, VariableType, FixedType};
use std::cell::RefCell;
use std::rc::Rc;

/// Representation of an optimizable vehicle variable.
#[derive(Debug)]
pub struct VehicleVariable3D {
    id: usize,
    pose: Rc<RefCell<[f64; 7]>>,
    fixed_type: FixedType,
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
