use uuid::Uuid;

pub enum FactorType {
    /// Necessary to fix graph for underdetermined systems
    PositionPrior2D,
    /// Corresponds to a GPS measurement
    Gps2D,
    /// Corresponds to an odometry measurement
    Odometry2D,
    /// Corresponds to an observation measurement of a landmark
    Observation2D,
}

pub trait Factor<'a> {
    fn get_id(&self) -> Uuid;
    fn get_type(&self) -> FactorType;

    // TODO design interface for content
    fn get_content(&self);
}