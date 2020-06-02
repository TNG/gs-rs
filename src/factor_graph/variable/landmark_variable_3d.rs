use crate::factor_graph::variable::{Variable, VariableType};
use std::cell::RefCell;
use std::rc::Rc;
use std::ops::Range;

/// Representation of an optimizable landmark variable.
#[derive(Debug)]
pub struct LandmarkVariable3D {
    id: usize,
    pose: Rc<RefCell<[f64; 3]>>,
    fixed: bool,
    range: Option<Range<usize>>,
}

impl LandmarkVariable3D {
    /// Returns a new variable from a 3D position, a given ID and whether the variable is fixed.
    pub fn new(id: usize, x: f64, y: f64, z: f64, fixed: bool, optional_range: Option<Range<usize>>) -> Self {
        LandmarkVariable3D {
            id,
            pose: Rc::new(RefCell::new([x, y, z])),
            fixed,
            range: optional_range,
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

    fn is_fixed(&self) -> bool {
        self.fixed
    }

    fn get_range(&self) -> Option<Range<usize>> {
        self.range.clone()
    }

    fn set_content(&self, update: Vec<f64>) {
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

        let test_variable = LandmarkVariable3D::new(1, 3.0, 5.0, 0.1, true, None);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Landmark3D);
        assert_eq!(test_variable.is_fixed(), true);
        assert_eq!(test_variable.get_range(), None);
    }

    #[test]
    fn test_new_dynamic() {
        init();

        let test_variable = LandmarkVariable3D::new(1, 3.0, 5.0, 0.1, false, Some(0..3));
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Landmark3D);
        assert_eq!(test_variable.is_fixed(), false);
        assert_eq!(test_variable.get_range(), Some(0..3));
    }

    #[test]
    fn test_update_content() {
        init();

        let test_variable = LandmarkVariable3D::new(1, 1.0, 1.0, 0.1, false, Some(0..3));
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[1.0, 1.0, 0.1]);
        test_variable.set_content(vec![2.0, 3.0, 0.5]);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[2.0, 3.0, 0.5]);
    }
}
