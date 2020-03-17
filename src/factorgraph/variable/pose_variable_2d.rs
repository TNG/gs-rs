use uuid::Uuid;
use crate::factorgraph::variable::{Variable, VariableType};
use std::rc::Rc;
use std::cell::RefCell;

#[derive(Debug)]
pub struct PoseVariable2D {
    id: Uuid,
    /// [x, y, phi], phi defaulting to 0
    content: Rc<RefCell<[f64; 3]>>,
}

impl PoseVariable2D {
    pub fn from_position(x: f64, y: f64) -> Self {
        PoseVariable2D {
            id: Uuid::new_v4(),
            content: Rc::new(RefCell::new([x, y, 0.0])),
        }
    }

    pub fn from_pose(x: f64, y: f64, phi: f64) -> Self {
        PoseVariable2D {
            id: Uuid::new_v4(),
            content: Rc::new(RefCell::new([x, y, phi])),
        }
    }

    /// method for testing how to edit the content
    /// TODO create more appropriate editing interface
    pub fn set_content(&self, x: f64, y: f64, phi: f64) {
        *self.content.borrow_mut() = [x, y, phi];
    }
}

impl Variable<'_> for PoseVariable2D {
    fn get_id(&self) -> Uuid { self.id }

    fn get_type(&self) -> VariableType { VariableType::Pose2D }

    fn get_content(&self) -> Vec<f64> { (*self.content.borrow_mut()).to_vec() }
}

#[cfg(test)]
mod tests {
    use log::LevelFilter;
    use super::*;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn test_from_position() {
        init();

        let test_variable = PoseVariable2D::from_position(3.0, 5.0);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[3.0, 5.0, 0.0]);
        assert_eq!(test_variable.get_type(), VariableType::Pose2D);
    }

    #[test]
    fn test_from_pose() {
        init();

        let test_variable = PoseVariable2D::from_pose(3.0, 5.0, 0.1);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_variable.get_type(), VariableType::Pose2D);
    }

    #[test]
    fn test_set_content() {
        init();

        let test_variable = PoseVariable2D::from_pose(1.0, 1.0, 0.1);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[1.0, 1.0, 0.1]);
        test_variable.set_content(2.0, 3.0, 0.5);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[2.0, 3.0, 0.5]);
    }
}