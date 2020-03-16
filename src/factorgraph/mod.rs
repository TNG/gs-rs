use crate::factorgraph::factor::Factor;
use crate::factorgraph::variable::Variable;
use crate::factorgraph::constraint::Constraint;

pub mod variable;
pub mod factor;
pub mod constraint;

#[derive(Debug)]
pub struct FactorGraph<'a> {
    variables: Vec<&'a dyn Variable<'a>>,
    factors: Vec<&'a dyn Factor<'a>>,
    constraints: Vec<&'a Constraint<'a>>,
}

#[cfg(test)]
mod tests {
    use crate::factorgraph::FactorGraph;
    use log::LevelFilter;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn empty() {
        init();

        let empty = FactorGraph {
            factors: vec![],
            variables: vec![],
            constraints: vec![]
        };

        info!("{:?}", empty);
    }

}