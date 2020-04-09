use crate::factor_graph::variable::{Variable, VariableType};
use std::cell::RefCell;
use std::rc::Rc;

/// Representation of an optimizable landmark variable.
#[derive(Debug)]
pub struct LandmarkVariable2D {
    id: usize,
    pose: Rc<RefCell<[f64; 3]>>,
    fixed: bool,
    index: Option<usize>,
}

impl LandmarkVariable2D {
    /// Returns a new variable from a 2D pose, a given ID and whether the variable is fixed.
    pub fn new(id: usize, x: f64, y: f64, phi: f64, fixed: bool, some_index: usize) -> Self {
        LandmarkVariable2D {
            id,
            pose: Rc::new(RefCell::new([x, y, phi])),
            fixed,
            index: if fixed { None } else { Some(some_index) },
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

    fn get_pose(&self) -> Vec<f64> {
        (*self.pose.borrow_mut()).to_vec()
    }

    fn is_fixed(&self) -> bool {
        self.fixed
    }

    fn get_index(&self) -> Option<usize> {
        self.index
    }

    fn set_pose(&self, update: Vec<f64>) {
        *self.pose.borrow_mut() = [update[0], update[1], update[2]];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use log::LevelFilter;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn test_new_fixed() {
        init();

        let test_variable = LandmarkVariable2D::new(1, 3.0, 5.0, 0.1, true, 0);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Landmark2D);
        assert_eq!(test_variable.is_fixed(), true);
        assert_eq!(test_variable.get_index(), None);
    }

    #[test]
    fn test_new_dynamic() {
        init();

        let test_variable = LandmarkVariable2D::new(1, 3.0, 5.0, 0.1, false, 0);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Landmark2D);
        assert_eq!(test_variable.is_fixed(), false);
        assert_eq!(test_variable.get_index(), Some(0));
    }

    #[test]
    fn test_update_pose() {
        init();

        let test_variable = LandmarkVariable2D::new(1, 1.0, 1.0, 0.1, false, 0);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[1.0, 1.0, 0.1]);
        test_variable.set_pose(vec![2.0, 3.0, 0.5]);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[2.0, 3.0, 0.5]);
    }
}
