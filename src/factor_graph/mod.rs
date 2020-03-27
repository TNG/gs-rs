use petgraph::csr::{Csr, NodeIndex};
use petgraph::Directed;
use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::Variable;

pub mod factor;
pub mod variable;

pub type FactorGraphCsr<'a> = Csr<Box<dyn Variable<'a>>, Factor, Directed, usize>;

#[derive(Debug)]
pub struct FactorGraph<'a> {
    pub csr: FactorGraphCsr<'a>,
    pub node_indices: Vec<NodeIndex<usize>>,
}

// TODO implement fn get_node_at(index: NodeIndex<usize>) -> Box<dyn Variable<'a>>?