//! The internal representation of a factor graph.

use petgraph::csr::{Csr, NodeIndex};
use petgraph::Directed;
use std::collections::HashMap;
use std::ops::Index;

pub mod factor;
pub mod variable;

use factor::Factor;
use variable::Variable;

/// A CSR (compressed sparse row) representation of a factor graph.
pub type FactorGraphCsr<'a> = Csr<Variable, Factor, Directed, usize>;

/// Structure representing the factor graph internally.
#[derive(Debug)]
pub struct FactorGraph {
    /// The factor graph's CSR (compressed sparse row) representation.
    pub csr: Csr<Variable, Factor, Directed, usize>,
    /// The indices at which the factor graph's nodes can be found in get_var(/*node_index*/).
    pub node_indices: Vec<NodeIndex<usize>>,
    /// Map from custom IDs as stated in the parsed file to internal CSR indices.
    pub custom_to_csr_id_map: HashMap<usize, NodeIndex<usize>>,
    /// The number of nodes which are dynamic, i.e. the number of fixed nodes subtracted of the total number of nodes.
    pub matrix_dim: usize,
}

impl FactorGraph {
    /// Returns the variable at the corresponding internal CSR index.
    pub fn get_var(&self, csr_index: usize) -> &Variable {
        self.csr.index(csr_index)
    }
}
