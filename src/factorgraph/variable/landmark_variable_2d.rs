use uuid::Uuid;
use crate::factorgraph::variable::{Variable, VariableType};
use std::rc::Rc;
use std::cell::RefCell;

#[derive(Debug)]
pub struct LandmarkVariable2D {
    id: Uuid,
    /// [x, y]
    content: Rc<RefCell<[f64; 2]>>,
}

impl LandmarkVariable2D {
    pub fn from_position(x: f64, y: f64) -> Self {
        LandmarkVariable2D {
            id: Uuid::new_v4(),
            content: Rc::new(RefCell::new([x, y])),
        }
    }

    /// method for testing how to edit the content
    /// TODO create more appropriate editing interface
    pub fn set_content(&self, x: f64, y: f64) {
        *self.content.borrow_mut() = [x, y];
    }
}

impl Variable<'_> for LandmarkVariable2D {
    fn get_id(&self) -> Uuid { self.id }

    fn get_type(&self) -> VariableType { VariableType::Landmark2D }

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

        let test_variable = LandmarkVariable2D::from_position(3.0, 5.0);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[3.0, 5.0]);
        assert_eq!(test_variable.get_type(), VariableType::Landmark2D);
    }

    #[test]
    fn test_set_content() {
        init();

        let test_variable = LandmarkVariable2D::from_position(1.0, 1.0);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.get_content(), &[1.0, 1.0]);
        test_variable.set_content(2.0, 3.0);
        assert_eq!(test_variable.get_content(), &[2.0, 3.0]);
    }
}