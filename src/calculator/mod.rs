#![allow(non_snake_case)]

use crate::factor_graph::FactorGraph;
use nalgebra::{DMatrix, DVector, VecStorage, Dynamic, U1, Vector3, Matrix3x6, Matrix6x3};
use crate::factor_graph::factor::{Factor, FactorType::*};
use std::ops::Index;
use std::collections::HashMap;
use petgraph::csr::NodeIndex;
use petgraph::visit::EdgeRef;

fn calculate_H_b(factor_graph: &FactorGraph) -> (DMatrix<f64>, DVector<f64>) {
    let dim = factor_graph.node_indices.len() * 3;
    // TODO optimize H with CsMatrix; suspicion: not possible till after H has been calculated -> remove "sparse" feature in Cargo.toml if never used
    // let max_non_zeros = factor_graph.csr.edge_count() * 18;
    // let mut H = CsMatrix::new_uninitialized_generic(Dynamic::new(dim), Dynamic::new(dim), max_non_zeros);
    let mut H = DMatrix::from_vec(dim, dim, vec![0.0; dim * dim]);
    let mut b = DVector::from_vec(vec![0.0; dim]);
    let index_map: HashMap<&usize, usize> = factor_graph.node_indices.iter().enumerate().map(|(a,b)| (b,a)).collect();

    for variable_index in &factor_graph.node_indices {
        let var_vector = &Vector3::from_vec(factor_graph.csr.index(*variable_index).get_pose());

        let jacobi = Matrix3x6::from_vec(vec![-1.0,  0.0,  0.0,   // transposed jacobi is displayed
                                              0.0, -1.0,  0.0,
                                              0.0,  0.0, -1.0,
                                              1.0,  0.0,  0.0,
                                              0.0,  1.0,  0.0,
                                              0.0,  0.0,  1.0]);
        let jacobi_t = Matrix6x3::from_vec(vec![-1.0,  0.0,  0.0,  1.0,  0.0,  0.0,   // transposed jacobi_t is displayed
                                                 0.0, -1.0,  0.0,  0.0,  1.0,  0.0,
                                                 0.0,  0.0, -1.0,  0.0,  0.0,  1.0]);

        for edge in factor_graph.csr.edges(*variable_index) {
            let factor: &Factor = edge.weight();
            let omega_k = &factor.information_matrix.content;
            let var_matr_index = index_map[variable_index];
            let var_range = 3*var_matr_index..3*(var_matr_index +1);
            match factor.factor_type {
                Position2D => {
                    let H_k = omega_k;
                    let updated_H_k = &(H.index((var_range.clone(), var_range.clone())) + H_k);
                    H.index_mut((var_range.clone(), var_range.clone())).copy_from(updated_H_k);
                    let e_k_t = (&Vector3::from_vec(factor.constraint.clone()) - var_vector).transpose();
                    let b_k = e_k_t * omega_k;
                    let updated_b_k = &(b.index((.., var_range.clone())) + b_k);
                    b.index_mut((.., var_range)).copy_from(updated_b_k);
                },
                Odometry2D | Observation2D => {
                    let right_mult = omega_k * jacobi;
                    let target_vector = &Vector3::from_vec(factor_graph.csr.index(edge.target()).get_pose());
                    let H_k = jacobi_t * &right_mult;
                    let target_matr_index = index_map[&edge.target()];
                    let target_range = 3*target_matr_index..3*(target_matr_index +1);
                    let H_i_i = H_k.index((..3, ..3));
                    let updated_H_i_i = &(H.index((var_range.clone(), var_range.clone())) + H_i_i);
                    H.index_mut((var_range.clone(), var_range.clone())).copy_from(updated_H_i_i);
                    let H_i_j = H_k.index((..3, 3..));
                    let updated_H_i_j = &(H.index((var_range.clone(), target_range.clone())) + H_i_j);
                    H.index_mut((var_range.clone(), target_range.clone())).copy_from(updated_H_i_j);
                    let H_j_i = H_k.index((3.., ..3));
                    let updated_H_j_i = &(H.index((target_range.clone(), var_range.clone())) + H_j_i);
                    H.index_mut((target_range.clone(), var_range.clone())).copy_from(updated_H_j_i);
                    let H_j_j = H_k.index((3.., 3..));
                    let updated_H_j_j = &(H.index((target_range.clone(), target_range.clone())) + H_j_j);
                    H.index_mut((target_range.clone(), target_range.clone())).copy_from(updated_H_j_j);
                    let e_k_t = (&Vector3::from_vec(factor.constraint.clone()) - (target_vector - var_vector)).transpose();
                    let b_k = e_k_t * &right_mult;
                    let b_i = b_k.index((.., ..3));
                    let updated_b_i = &(b.index((.., var_range.clone())) + b_i);
                    b.index_mut((.., var_range)).copy_from(updated_b_i);
                    let b_j = b_k.index((.., 3..));
                    let updated_b_j = &(b.index((.., target_range.clone())) + b_j);
                    b.index_mut((.., target_range)).copy_from(updated_b_j);
                },
            };
        }
    }
    (H, b)
}

// TODO test calculate_H_b()