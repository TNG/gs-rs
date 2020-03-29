use crate::factor_graph::variable::{Variable, VariableType};
use std::cell::RefCell;
use std::rc::Rc;
use uuid::Uuid;

#[derive(Debug)]
pub struct VehicleVariable2D {
    id: Uuid,
    /// [x, y, phi], phi defaulting to 0
    pose: Rc<RefCell<[f64; 3]>>,
}

impl VehicleVariable2D {
    pub fn from_position(x: f64, y: f64) -> Self {
        VehicleVariable2D {
            id: Uuid::new_v4(),
            pose: Rc::new(RefCell::new([x, y, 0.0])),
        }
    }

    pub fn from_pose(x: f64, y: f64, phi: f64) -> Self {
        VehicleVariable2D {
            id: Uuid::new_v4(),
            pose: Rc::new(RefCell::new([x, y, phi])),
        }
    }

    pub fn from_pose_and_id(id: usize, x: f64, y: f64, phi: f64) -> Self {
        VehicleVariable2D {
            id: Uuid::from_u128(id as u128),
            pose: Rc::new(RefCell::new([x, y, phi])),
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

    /// method for testing how to edit the content
    /// TODO create more appropriate editing interface
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
    }

    #[test]
    fn test_from_pose() {
        init();

        let test_variable = VehicleVariable2D::from_pose(3.0, 5.0, 0.1);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Vehicle2D);
    }

    #[test]
    fn test_from_pose_and_id() {
        init();

        let test_variable = VehicleVariable2D::from_pose_and_id(1, 3.0, 5.0, 0.1);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_pose(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Vehicle2D);
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
