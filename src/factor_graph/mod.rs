use petgraph::csr::Csr;
use petgraph::Undirected;
use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::Variable;

pub mod factor;
pub mod variable;

pub type FactorGraph<'a> = Csr<Box<dyn Variable<'a>>, Factor, Undirected, usize>;