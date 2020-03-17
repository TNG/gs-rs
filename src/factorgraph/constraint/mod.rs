use uuid::Uuid;

use crate::factorgraph::factor::{Factor, /*TODO uncomment: position_prior_factor_2d::PositionPriorFactor2D,*/ position_factor_2d::PositionFactor2D, odometry_factor_2d::OdometryFactor2D, observation_factor_2d::ObservationFactor2D};
use crate::factorgraph::variable::{Variable, pose_variable_2d::PoseVariable2D, landmark_variable_2d::LandmarkVariable2D};

#[derive(Debug)]
pub struct Constraint<'a> {
    id: Uuid,
    factor: &'a dyn Factor<'a>,
    variables: Vec<&'a dyn Variable<'a>>,
}

impl Constraint<'_> {
    fn get_id(&self) -> Uuid {
        self.id
    }

    // TODO implement methods without lifetime mismatch
    // fn get_factor(&self) -> &dyn Factor<'_> { self.factor }
    //
    // fn get_variables(&self) -> &Vec<&dyn Variable> { &self.variables }

    // TODO implement after PositionPriorFactor2D
    fn new_position_prior_constraint_2d(/*factor: &PositionPriorFactor2D, [variables]*/) -> Self {
        unimplemented!();
    }

    // TODO fix lifetime conflicts
    // fn new_position_constraint_2d(factor: &PositionFactor2D, vehicle: &PoseVariable2D) -> Self {
    //     Constraint {
    //         id: Uuid::new_v4(),
    //         factor,
    //         variables: vec![vehicle],
    //     }
    // }
    //
    // fn new_odometry_constraint_2d(factor: &OdometryFactor2D, vehicle_start: &PoseVariable2D, vehicle_end: &PoseVariable2D) -> Self {
    //     Constraint {
    //         id: Uuid::new_v4(),
    //         factor,
    //         variables: vec![vehicle_start, vehicle_end],
    //     }
    // }
    //
    // fn new_observation_constraint_2d(factor: &ObservationFactor2D, vehicle: &PoseVariable2D, landmark: &LandmarkVariable2D) -> Self {
    //     Constraint {
    //         id: Uuid::new_v4(),
    //         factor,
    //         variables: vec![vehicle, landmark],
    //     }
    // }
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
    fn test_new_position_prior_constraint_2d() {
        init();

        // TODO test after implementing new_position_prior_constraint_2d()
    }

    #[test]
    fn test_new_position_constraint_2d() {
        init();

        // TODO uncomment and test asserts once lifetime conflicts are removed
        // let factor = PositionFactor2D::from_position(1.0, 2.0);
        // let vehicle = PoseVariable2D::from_position(1.0, 1.0);
        // let test_constraint = Constraint::new_position_constraint_2d(&factor, &vehicle);
        // info!("{:?}", &test_constraint);
    }

    // TODO test new_{odometry, observation}_constraint_2d
}