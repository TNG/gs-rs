use uuid::Uuid;

use crate::factor_graph::factor::{Factor, /*TODO uncomment: position_prior_factor_2d::PositionPriorFactor2D,*/ observation_factor_2d::ObservationFactor2D, odometry_factor_2d::OdometryFactor2D, position_factor_2d::PositionFactor2D};
use crate::factor_graph::variable::{landmark_variable_2d::LandmarkVariable2D, vehicle_variable_2d::VehicleVariable2D, Variable};

#[derive(Debug)]
pub struct Constraint<'a> {
    id: Uuid,
    factor: &'a dyn Factor<'a>,
    variables: Vec<&'a dyn Variable<'a>>,
}

impl<'a> Constraint<'a> {
    fn get_id(&self) -> Uuid {
        self.id
    }

    fn get_factor(&self) -> &dyn Factor<'a> { self.factor }

    fn get_variables(&self) -> &Vec<&dyn Variable<'a>> { &self.variables }

    // TODO implement after PositionPriorFactor2D
    fn new_position_prior_constraint_2d(/*factor: &PositionPriorFactor2D, [variables]*/) -> Self {
        unimplemented!();
    }

    fn new_position_constraint_2d(factor: &'a PositionFactor2D, vehicle: &'a VehicleVariable2D) -> Self {
        Constraint {
            id: Uuid::new_v4(),
            factor,
            variables: vec![vehicle],
        }
    }

    fn new_odometry_constraint_2d(factor: &'a OdometryFactor2D, vehicle_start: &'a VehicleVariable2D, vehicle_end: &'a VehicleVariable2D) -> Self {
        Constraint {
            id: Uuid::new_v4(),
            factor,
            variables: vec![vehicle_start, vehicle_end],
        }
    }

    fn new_observation_constraint_2d(factor: &'a ObservationFactor2D, vehicle: &'a VehicleVariable2D, landmark: &'a LandmarkVariable2D) -> Self {
        Constraint {
            id: Uuid::new_v4(),
            factor,
            variables: vec![vehicle, landmark],
        }
    }
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

        let factor = PositionFactor2D::from_position(1.0, 2.0);
        let vehicle = VehicleVariable2D::from_position(1.0, 1.0);
        let test_constraint = Constraint::new_position_constraint_2d(&factor, &vehicle);
        info!("{:?}", &test_constraint);
    }

    // TODO test new_{odometry, observation}_constraint_2d, add assertions
}