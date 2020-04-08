//! The internal representation of a factor graph.

use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::Variable;
use petgraph::csr::{Csr, NodeIndex};
use petgraph::Directed;
use std::ops::Index;
use std::collections::HashMap;

pub mod factor;
pub mod variable;

/// A CSR (compressed sparse row) representation of a factor graph.
pub type FactorGraphCsr<'a> = Csr<Box<dyn Variable<'a>>, Factor, Directed, usize>;

/// Structure representing the factor graph internally.
#[derive(Debug)]
pub struct FactorGraph<'a> {
    /// The factor graph's CSR (compressed sparse row) representation.
    pub csr: FactorGraphCsr<'a>,
    /// The indices at which the factor graph's nodes can be found in csr.index(/*node_index*/).
    pub node_indices: Vec<NodeIndex<usize>>,
    /// Map from custom IDs as stated in the parsed file to internal CSR indices.
    pub custom_to_csr_id_map: HashMap<usize, NodeIndex<usize>>, // TODO combine with node_indices for less redundancy
    /// The number of nodes which are dynamic, i.e. the number of fixed nodes subtracted of the total number of nodes.
    pub number_of_dynamic_nodes: usize,
}

impl<'a> FactorGraph<'a> {
    pub fn get_var_at_csr_index(&self, csr_index: usize) -> &Box<dyn Variable<'a>> {
        self.csr.index(csr_index)
    }
}
