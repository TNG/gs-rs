use crate::factor_graph::variable::{Variable, VariableType};
use std::cell::RefCell;
use std::rc::Rc;
use std::ops::Range;

/// Representation of an optimizable vehicle variable.
#[derive(Debug)]
pub struct VehicleVariable3D {
    id: usize,
    pose: Rc<RefCell<[f64; 7]>>,
    fixed: bool,
    range: Option<Range<usize>>,
}

impl VehicleVariable3D {
    /// Returns a new variable from a 3D pose, a given ID and whether the variable is fixed.
    pub fn new(id: usize, x: f64, y: f64, z: f64, rot_x: f64, rot_y: f64, rot_z: f64, rot_w: f64, fixed: bool, optional_range: Option<Range<usize>>) -> Self {
        VehicleVariable3D {
            id,
            pose: Rc::new(RefCell::new([x, y, z, rot_x, rot_y, rot_z, rot_w])),
            fixed,
            range: optional_range,
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

    fn is_fixed(&self) -> bool {
        self.fixed
    }

    fn get_range(&self) -> Option<Range<usize>> {
        self.range.clone()
    }

    fn set_content(&self, update: Vec<f64>) {
        *self.pose.borrow_mut() = [update[0], update[1], update[2], update[3], update[4], update[5], update[6]];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use log::LevelFilter;
    use log::info;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn test_new_fixed() {
        init();

        let test_variable = VehicleVariable3D::new(1, 3.0, 5.0, 0.1, 0.0, 0.0, 0.0, 1.0, true, None);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[3.0, 5.0, 0.1, 0.0, 0.0, 0.0, 1.0]);
        assert_eq!(test_variable.get_type(), VariableType::Vehicle3D);
        assert_eq!(test_variable.is_fixed(), true);
        assert_eq!(test_variable.get_range(), None);
    }

    #[test]
    fn test_new_dynamic() {
        init();

        let test_variable = VehicleVariable3D::new(1, 3.0, 5.0, 0.1, 0.0, 0.0, 0.0, 1.0, false, Some(0..7));
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[3.0, 5.0, 0.1, 0.0, 0.0, 0.0, 1.0]);
        assert_eq!(test_variable.get_type(), VariableType::Vehicle3D);
        assert_eq!(test_variable.is_fixed(), false);
        assert_eq!(test_variable.get_range(), Some(0..7));
    }

    #[test]
    fn test_update_content() {
        init();

        let test_variable = VehicleVariable3D::new(1, 1.0, 1.0, 0.1, 0.0, 0.0, 0.0, 1.0, false, Some(0..7));
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[1.0, 1.0, 0.1, 0.0, 0.0, 0.0, 1.0]);
        test_variable.set_content(vec![2.0, 3.0, 0.5, 0.0, 0.0, 0.0, 1.0]);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[2.0, 3.0, 0.5, 0.0, 0.0, 0.0, 1.0]);
    }
}
