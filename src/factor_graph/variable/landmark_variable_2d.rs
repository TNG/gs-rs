use crate::factor_graph::variable::{Variable, VariableType, FixedType};
use std::cell::RefCell;
use std::rc::Rc;
use std::ops::Range;

/// Representation of an optimizable landmark variable.
#[derive(Debug)]
pub struct LandmarkVariable2D {
    id: usize,
    position: Rc<RefCell<[f64; 2]>>,
    fixed_type: FixedType,
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
