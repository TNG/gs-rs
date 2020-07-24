//! The internal representation of a factor graph.
// I have never seen a '//!' in rust, is this some special syntax?

use petgraph::csr::{Csr, NodeIndex};
use petgraph::Directed;
use std::ops::Index;
use std::collections::HashMap;

pub mod factor;
pub mod variable;

// I would not construct the path through the crate root to increase modularity and readability (when someone does not know the crate too well one might get confused)
// (path from crate root should not matter, when one needs something from a sibling crate I would at least question the design)
// fashion one: use self::
use self::factor::Factor;
// fashion two: no self, works only in 2018 editon
use variable::Variable;
// personal taste: I would not use self but pack the relative use statements under the mod section

// reference: https://doc.rust-lang.org/reference/items/use-declarations.html

/// A CSR (compressed sparse row) representation of a factor graph.
pub type FactorGraphCsr<'a> = Csr<Box<dyn Variable<'a>>, Factor, Directed, usize>;

/// Structure representing the factor graph internally.
#[derive(Debug)]
pub struct FactorGraph<'a> {
    /// The factor graph's CSR (compressed sparse row) representation.
    pub csr: FactorGraphCsr<'a>,
    /// The indices at which the factor graph's nodes can be found in get_var(/*node_index*/).
    pub node_indices: Vec<NodeIndex<usize>>,
    /// Map from custom IDs as stated in the parsed file to internal CSR indices.
    pub custom_to_csr_id_map: HashMap<usize, NodeIndex<usize>>,
    /// The number of nodes which are dynamic, i.e. the number of fixed nodes subtracted of the total number of nodes.
    pub matrix_dim: usize,
}

impl<'a> FactorGraph<'a> {
    /// Returns the variable at the corresponding internal CSR index.
    pub fn get_var(&self, csr_index: usize) -> &Box<dyn Variable<'a>> {
        self.csr.index(csr_index)
    }
}
