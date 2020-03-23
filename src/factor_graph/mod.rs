use petgraph::csr::{Csr, NodeIndex};
use petgraph::Undirected;
use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::Variable;

pub mod factor;
pub mod variable;

pub type FactorGraphCsr<'a> = Csr<Box<dyn Variable<'a>>, Factor, Undirected, usize>;

#[derive(Debug)]
pub struct FactorGraph<'a> {
    pub csr: FactorGraphCsr<'a>,
    pub node_indices: Vec<NodeIndex>,
}