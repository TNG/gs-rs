use crate::factor_graph::variable::{Variable, VariableType};
use std::cell::RefCell;
use std::rc::Rc;
use uuid::Uuid;

/// Representation of an optimizable vehicle variable.
#[derive(Debug)]
pub struct VehicleVariable2D {
    id: Uuid,
    pose: Rc<RefCell<[f64; 3]>>,
    fixed: bool,
    index: Option<usize>,
}

// TODO remove all unused constructors?
impl VehicleVariable2D {
    /// Returns a new variable from a 2D position.
    pub fn from_position(x: f64, y: f64) -> Self {
        VehicleVariable2D {
            id: Uuid::new_v4(),
            pose: Rc::new(RefCell::new([x, y, 0.0])),
            fixed: false,
            index: None,
        }
    }

    /// Returns a new variable from a 2D pose.
    pub fn from_pose(x: f64, y: f64, phi: f64) -> Self {
        VehicleVariable2D {
            id: Uuid::new_v4(),
            pose: Rc::new(RefCell::new([x, y, phi])),
            fixed: false,
            index: None,
        }
    }

    /// Returns a new variable from a 2D pose and a given ID.
    pub fn from_pose_and_id(id: usize, x: f64, y: f64, phi: f64) -> Self {
        VehicleVariable2D {
            id: Uuid::from_u128(id as u128),
            pose: Rc::new(RefCell::new([x, y, phi])),
            fixed: false,
            index: None,
        }
    }

    /// Returns a new variable from a 2D pose, a given ID and whether the variable is fixed.
    pub fn from_full_config(id: usize, x: f64, y: f64, phi: f64, fixed: bool, some_index: usize) -> Self {
        VehicleVariable2D {
            id: Uuid::from_u128(id as u128),
            pose: Rc::new(RefCell::new([x, y, phi])),
            fixed,
            index: if fixed { None } else { Some(some_index) },
        }
    }
}

impl Variable<'_> for VehicleVariable2D {
    fn get_id(&self) -> Uuid {
        self.id
    }

    fn get_type(&self) -> VariableType {
        VariableType::Vehicle2D
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

    fn update_pose(&self, update: Vec<f64>) {
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
    fn test_from_position() {
        init();

        let test_variable = VehicleVariable2D::from_position(3.0, 5.0);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[3.0, 5.0, 0.0]);
        assert_eq!(test_variable.get_type(), VariableType::Vehicle2D);
        assert_eq!(test_variable.is_fixed(), false);
    }

    #[test]
    fn test_from_pose() {
        init();

        let test_variable = VehicleVariable2D::from_pose(3.0, 5.0, 0.1);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Vehicle2D);
        assert_eq!(test_variable.is_fixed(), false);
    }

    #[test]
    fn test_from_pose_and_id() {
        init();

        let test_variable = VehicleVariable2D::from_pose_and_id(1, 3.0, 5.0, 0.1);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Vehicle2D);
        assert_eq!(test_variable.is_fixed(), false);
    }

    #[test]
    fn test_from_full_config_fixed() {
        init();

        let test_variable = VehicleVariable2D::from_full_config(1, 3.0, 5.0, 0.1, true, 0);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Vehicle2D);
        assert_eq!(test_variable.is_fixed(), true);
        assert_eq!(test_variable.get_index(), None);
    }

    #[test]
    fn test_from_full_config_dynamic() {
        init();

        let test_variable = VehicleVariable2D::from_full_config(1, 3.0, 5.0, 0.1, false, 0);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Vehicle2D);
        assert_eq!(test_variable.is_fixed(), false);
        assert_eq!(test_variable.get_index(), Some(0));
    }

    #[test]
    fn test_update_pose() {
        init();

        let test_variable = VehicleVariable2D::from_pose(1.0, 1.0, 0.1);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[1.0, 1.0, 0.1]);
        test_variable.update_pose(vec![2.0, 3.0, 0.5]);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[2.0, 3.0, 0.5]);
    }
}
