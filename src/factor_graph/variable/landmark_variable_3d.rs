use crate::factor_graph::variable::{Variable, VariableType, FixedType};
use std::cell::RefCell;
use std::rc::Rc;

/// Representation of an optimizable landmark variable.
#[derive(Debug)]
pub struct LandmarkVariable3D {
    id: usize,
    pose: Rc<RefCell<[f64; 3]>>,
    fixed_type: FixedType,

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
