use uuid::Uuid;
use crate::factorgraph::variable::{Variable, VariableType};

#[derive(Debug)]
pub struct PoseVariable2D {
    id: Uuid,
    /// [x, y, phi], phi defaulting to 0
    content: [f64; 3],
}

impl PoseVariable2D {
    pub fn from_position(x: f64, y: f64) -> Self {
        PoseVariable2D {
            id: Uuid::new_v4(),
            content: [x, y, 0.0],
        }
    }

    pub fn from_pose(x: f64, y: f64, phi: f64) -> Self {
        PoseVariable2D {
            id: Uuid::new_v4(),
            content: [x, y, phi],
        }
    }
}

impl Variable<'_> for PoseVariable2D {
    fn get_id(&self) -> Uuid {
        self.id
    }

    fn get_type(&self) -> VariableType {
        VariableType::Pose2D
    }

    // TODO implement after deciding on design
    fn get_content(&self) {
        unimplemented!()
    }
}

// TODO write tests