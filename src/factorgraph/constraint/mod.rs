use uuid::Uuid;

use crate::factorgraph::{factor::Factor, variable::Variable};

pub struct Constraint<'a> {
    id: Uuid,
    variables: Vec<&'a dyn Variable<'a>>,
    factor: &'a dyn Factor<'a>,
}