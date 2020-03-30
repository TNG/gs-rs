//! Conversion between factor graph structures and G2O files.

use crate::parser::model::FactorGraphModel;
use crate::parser::Parser;

/// Implements G2O specific functions for parsing and composing files.
pub struct G2oParser;

// TODO: impl Parser for JsonParser { }