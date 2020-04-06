//! This file should replace linear_system.rs once completed.

use crate::factor_graph::FactorGraph;
use nalgebra::{DMatrix, DVector, Vector2, Rotation2, Rotation3, Vector3, Matrix3x6, Matrix6x3, Matrix3, Matrix, Dynamic, SliceStorage, U1, U6};
use crate::factor_graph::variable::Variable;
use petgraph::csr::{Edges, EdgeReference};
use petgraph::visit::EdgeRef;
use crate::factor_graph::factor::{Factor, FactorType::*};
use petgraph::Directed;
use std::env::var;

pub fn calculate_H_b(factor_graph: &FactorGraph) -> (DMatrix<f64>, DVector<f64>) {
    let dim = factor_graph.number_of_dynamic_nodes * 3;
    let mut H = DMatrix::from_vec(dim, dim, vec![0.0; dim * dim]);
    let mut b = DVector::from_vec(vec![0.0; dim]);

    factor_graph.node_indices.iter()
        .map(|i| factor_graph.csr.edges(*i))
        .for_each(|edges| edges.into_iter()
            .map(|edge| update_H_b(factor_graph, &mut H, &mut b, edge))
            .for_each(drop));

    (H, b)
}

fn update_H_b(factor_graph: &FactorGraph, H: &mut DMatrix<f64>, b: &mut DVector<f64>, edge: EdgeReference<Factor, Directed, usize>) {
    let factor = edge.weight();
    match factor.factor_type {
        Position2D => (),
        Odometry2D | Observation2D => update_two_vars_2d(H, b, factor, &factor_graph.get_var_at_csr_index(edge.source()), &factor_graph.get_var_at_csr_index(edge.target())),
    };
}

// TODO seperate files for each comnbination of number of vars and dimension?

fn update_one_var_2d(H: &mut DMatrix<f64>, b: &mut DVector<f64>) {
    // TODO implement
}

fn calc_one_var_jacobians_2d() {
    // TODO implement
}

fn update_two_vars_2d(H: &mut DMatrix<f64>, b: &mut DVector<f64>, factor: &Factor, var_i: &Box<dyn Variable>, var_j: &Box<dyn Variable>) {
    let (pos_i, rot_i) = get_pos_and_rot(&var_i.get_pose());
    let (pos_j, rot_j) = get_pos_and_rot(&var_j.get_pose());
    let (pos_ij, rot_ij) = get_pos_and_rot(&factor.constraint); // TODO rename _ij so that it's clearer that measurements are meant?
    let (jacobi, jacobi_T) = calc_two_var_jacobians_2d(pos_i, rot_i, pos_j, rot_ij);
    let right_mult = &factor.information_matrix.content * jacobi;

    let H_updates = jacobi_T * right_mult;
    update_H_submatrix(H, &H_updates.index((..3, ..3)), var_i, var_i);
    update_H_submatrix(H, &H_updates.index((..3, 3..)), var_i, var_j);
    update_H_submatrix(H, &H_updates.index((3.., ..3)), var_j, var_i);
    update_H_submatrix(H, &H_updates.index((3.., 3..)), var_j, var_j);

    // TODO update b
}


fn calc_two_var_jacobians_2d(pos_i: Vector2<f64>, rot_i: f64, pos_j: Vector2<f64>, rot_ij: f64) -> (Matrix3x6<f64>, Matrix6x3<f64>) {
    let rot_obj = Rotation3::from_axis_angle(&Vector3::z_axis(), -rot_ij);
    let R_ij_T = rot_obj.matrix();
    let delta_pos_vec = pos_i - pos_j;
    let delta_pos = delta_pos_vec.data.as_slice();
    let sin_i = rot_i.sin();
    let cos_i = rot_i.cos();
    let last_column_top = -sin_i * delta_pos[0] + cos_i * delta_pos[1];
    let last_column_mid = -cos_i * delta_pos[0] - sin_i * delta_pos[1];
    let A_ij = R_ij_T * &Matrix3::from_vec(vec![         -cos_i,           sin_i,  0.0,   // transposed matrix is displayed
                                                         -sin_i,          -cos_i,  0.0,
                                                last_column_top, last_column_mid, -1.0,]);
    let B_ij = R_ij_T * &Matrix3::from_vec(vec![cos_i, -sin_i, 0.0,    // transposed matrix is displayed
                                                sin_i,  cos_i, 0.0,
                                                  0.0,    0.0, 1.0,]);

    let mut jacobian = Matrix3x6::from_vec(vec![0.0; 18]);
    jacobian.index_mut((.., ..3)).copy_from(&A_ij);
    jacobian.index_mut((.., 3..)).copy_from(&B_ij);
    (jacobian, jacobian.transpose())
}

fn get_pos_and_rot(pose: &[f64]) -> (Vector2<f64>, f64) {
    (Vector2::new(pose[0], pose[1],), pose[2])
}

fn update_H_submatrix(H: &mut DMatrix<f64>, added_matrix: &Matrix<f64, Dynamic, Dynamic, SliceStorage<f64,Dynamic,Dynamic,U1,U6>>, var_row: &Box<dyn Variable>, var_col: &Box<dyn Variable>) {
    if var_row.is_fixed() || var_col.is_fixed() {
        return;
    }
    let row_index = var_row.get_index().unwrap();
    let col_index = var_col.get_index().unwrap();
    let row_range = 3*row_index..3*row_index+3;
    let col_range = 3*col_index..3*col_index+3;
    let updated_submatrix = &(H.index((row_range.clone(), col_range. clone())) + added_matrix);
    H.index_mut((row_range, col_range)).copy_from(updated_submatrix);
}
