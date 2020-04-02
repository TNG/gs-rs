use crate::factor_graph::variable::{Variable, VariableType};
use std::cell::RefCell;
use std::rc::Rc;
use uuid::Uuid;

/// Representation of an optimizable landmark variable.
#[derive(Debug)]
pub struct LandmarkVariable2D {
    id: Uuid,
    pose: Rc<RefCell<[f64; 3]>>,
    fixed: bool,
}

impl LandmarkVariable2D {
    /// Returns a new variable from a 2D position.
    pub fn from_position(x: f64, y: f64) -> Self {
        LandmarkVariable2D {
            id: Uuid::new_v4(),
            pose: Rc::new(RefCell::new([x, y, 0.0])),
            fixed: false,
        }
    }

    /// Returns a new variable from a 2D pose.
    pub fn from_pose(x: f64, y: f64, phi: f64) -> Self {
        LandmarkVariable2D {
            id: Uuid::new_v4(),
            pose: Rc::new(RefCell::new([x, y, phi])),
            fixed: false,
        }
    }

    /// Returns a new variable from a 2D pose and a given ID.
    pub fn from_pose_and_id(id: usize, x: f64, y: f64, phi: f64) -> Self {
        LandmarkVariable2D {
            id: Uuid::from_u128(id as u128),
            pose: Rc::new(RefCell::new([x, y, phi])),
            fixed: false,
        }
    }

    /// Returns a new variable from a 2D pose, a given ID and whether the variable is fixed.
    pub fn from_pose_and_id_and_fixed(id: usize, x: f64, y: f64, phi: f64, fixed: bool) -> Self {
        LandmarkVariable2D {
            id: Uuid::from_u128(id as u128),
            pose: Rc::new(RefCell::new([x, y, phi])),
            fixed,
        }
    }
}

impl Variable<'_> for LandmarkVariable2D {
    fn get_id(&self) -> Uuid {
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

        let test_variable = LandmarkVariable2D::from_position(3.0, 5.0);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[3.0, 5.0, 0.0]);
        assert_eq!(test_variable.get_type(), VariableType::Landmark2D);
        assert_eq!(test_variable.is_fixed(), false);
    }

    #[test]
    fn test_from_pose() {
        init();

        let test_variable = LandmarkVariable2D::from_pose(3.0, 5.0, 0.1);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Landmark2D);
        assert_eq!(test_variable.is_fixed(), false);
    }

    #[test]
    fn test_from_pose_and_id() {
        init();

        let test_variable = LandmarkVariable2D::from_pose_and_id(1, 3.0, 5.0, 0.1);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Landmark2D);
        assert_eq!(test_variable.is_fixed(), false);
    }

    #[test]
    fn test_from_pose_and_id_and_fixed() {
        init();

        let test_variable = LandmarkVariable2D::from_pose_and_id_and_fixed(1, 3.0, 5.0, 0.1, true);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Landmark2D);
        assert_eq!(test_variable.is_fixed(), true);
    }

    #[test]
    fn test_update_pose() {
        init();

        let test_variable = LandmarkVariable2D::from_pose(1.0, 1.0, 0.1);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[1.0, 1.0, 0.1]);
        test_variable.update_pose(vec![2.0, 3.0, 0.5]);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[2.0, 3.0, 0.5]);
    }
}
