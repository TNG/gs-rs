use crate::factor_graph::factor::{Factor, FactorType::*};
use crate::factor_graph::FactorGraph;
use nalgebra::{DMatrix, DVector, Matrix3, Matrix3x6, Matrix6x3, Point2, Rotation2, RowVector3, Vector2, Vector3};
use petgraph::visit::EdgeRef;
use std::collections::hash_map::RandomState;
use std::collections::HashMap;
use std::ops::{Index, Range};

pub fn calculate_H_b(factor_graph: &FactorGraph) -> (DMatrix<f64>, DVector<f64>) {
    let dim = factor_graph.node_indices.len() * 3;
    let mut H = DMatrix::from_vec(dim, dim, vec![0.0; dim * dim]);
    let mut b = DVector::from_vec(vec![0.0; dim]);

    let index_map: HashMap<&usize, usize> = factor_graph
        .node_indices
        .iter()
        .enumerate()
        .map(|(a, b)| (b, a))
        .collect();

    for variable_index in factor_graph.node_indices.iter() {
        update_H_b_for_variable(factor_graph, &mut H, &mut b, &index_map, *variable_index)
    }
    (H, b)
}

fn update_H_b_for_variable(
    factor_graph: &FactorGraph,
    mut H: &mut DMatrix<f64>,
    mut b: &mut DVector<f64>,
    index_map: &HashMap<&usize, usize, RandomState>,
    variable_index: usize,
) {
    let variable = factor_graph.csr.index(variable_index);
    let var_point = &Point2::new(variable.get_pose()[0], variable.get_pose()[1]);
    let var_rotation = variable.get_pose()[2];
    for edge in factor_graph.csr.edges(variable_index) {
        let factor: &Factor = edge.weight();
        let omega_k = &factor.information_matrix.content;
        let var_matr_index = index_map[&variable_index];
        let var_range = 3 * var_matr_index..3 * (var_matr_index + 1);
        match factor.factor_type {
            Position2D => handle_position_2d(
                &mut H,
                &mut b,
                var_point,
                var_rotation,
                factor,
                omega_k,
                &var_range,
            ),
            Odometry2D | Observation2D => handle_odometry_or_observation_2d(
                factor_graph,
                &mut H,
                &mut b,
                &index_map,
                var_point,
                var_rotation,
                edge.target(),
                factor,
                omega_k,
                &var_range,
            ),
        };
    }
}

fn handle_odometry_or_observation_2d(
    factor_graph: &FactorGraph,
    H: &mut DMatrix<f64>,
    b: &mut DVector<f64>,
    index_map: &HashMap<&usize, usize, RandomState>,
    var_point: &Point2<f64>,
    var_rotation: f64,
    target_id: usize,
    factor: &Factor,
    omega_k: &DMatrix<f64>,
    var_range: &Range<usize>,
) {
    let target = factor_graph.csr.index(target_id);
    let target_point = &Point2::new(target.get_pose()[0], target.get_pose()[1]);
    let target_rotation = target.get_pose()[2];

    let (jacobi, jacobi_t) = calculate_2d_jacobians_for_two_vars(
        var_point,
        var_rotation,
        target_point,
        factor.constraint[2],
    );
    let right_mult = omega_k * jacobi;
    let H_k = jacobi_t * &right_mult;
    let target_matr_index = index_map[&target_id];
    let target_range = 3 * target_matr_index..3 * (target_matr_index + 1);
    let H_ii = H_k.index((..3, ..3));
    let updated_H_ii = &(H.index((var_range.clone(), var_range.clone())) + H_ii);
    H.index_mut((var_range.clone(), var_range.clone()))
        .copy_from(updated_H_ii);
    let H_ij = H_k.index((..3, 3..));
    let updated_H_ij = &(H.index((var_range.clone(), target_range.clone())) + H_ij);
    H.index_mut((var_range.clone(), target_range.clone()))
        .copy_from(updated_H_ij);
    let H_ji = H_k.index((3.., ..3));
    let updated_H_ji = &(H.index((target_range.clone(), var_range.clone())) + H_ji);
    H.index_mut((target_range.clone(), var_range.clone()))
        .copy_from(updated_H_ji);
    let H_jj = H_k.index((3.., 3..));
    let updated_H_jj = &(H.index((target_range.clone(), target_range.clone())) + H_jj);
    H.index_mut((target_range.clone(), target_range.clone()))
        .copy_from(updated_H_jj);
    dbg!(H); // TODO remove once bug is fixed

    // e_ij_x = (R_ij_T (R_i_T (x_j - x_i) - x_ij))
    let e_ij_x = Rotation2::new(-factor.constraint[2])
        * (Rotation2::new(-var_rotation) * (target_point - var_point)
            - Vector2::new(factor.constraint[0], factor.constraint[1]));
    // e_ij_r = (phi_j - phi_i - phi_ij)
    let e_ij_r = target_rotation - var_rotation - factor.constraint[2];
    let e_k_t = RowVector3::from_vec(vec![
        e_ij_x.data.to_vec()[0],
        e_ij_x.data.to_vec()[1],
        e_ij_r,
    ]);
    let b_k = e_k_t * &right_mult;
    let b_i = b_k.index((.., ..3));
    let updated_b_i = &(b.index((var_range.clone(), ..)) + b_i.transpose());
    b.index_mut((var_range.clone(), ..)).copy_from(updated_b_i);
    let b_j = b_k.index((.., 3..));
    let updated_b_j = &(b.index((target_range.clone(), ..)) + b_j.transpose());
    b.index_mut((target_range, ..)).copy_from(updated_b_j);
}

fn handle_position_2d(
    H: &mut DMatrix<f64>,
    b: &mut DVector<f64>,
    var_point: &Point2<f64>,
    var_rotation: f64,
    factor: &Factor,
    omega_k: &DMatrix<f64>,
    var_range: &Range<usize>,
) {
    // TODO implement jacobian (copy paste from trash branch should work)
    let updated_H_k = &(H.index((var_range.clone(), var_range.clone())) + omega_k);
    H.index_mut((var_range.clone(), var_range.clone()))
        .copy_from(updated_H_k);

    // TODO implement e_k_t like the following:
    // void computeError()
    // {
    //   const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
    //   SE2 delta = _inverseMeasurement * v1->estimate();
    //   _error = delta.toVector();
    // }
    let var_position = var_point.coords.data;
    let var_vector = Vector3::new(
        var_position.as_slice()[0],
        var_position.as_slice()[1],
        var_rotation,
    );
    let e_k_t = (Vector3::from_vec(factor.constraint.clone()) - var_vector).transpose();
    let b_k = e_k_t * omega_k;
    let updated_b_k = &(b.index((var_range.clone(), ..)) + b_k.transpose());
    b.index_mut((var_range.clone(), ..)).copy_from(updated_b_k);
}

fn calculate_2d_jacobians_for_two_vars(
    point_i: &Point2<f64>,
    rotation_i: f64,
    point_j: &Point2<f64>,
    rotation_meas: f64,
) -> (Matrix3x6<f64>, Matrix6x3<f64>) {
    // TODO remove once bug is fixed
    // let manifold_jacobian = Matrix3x6::from_vec(vec![
    //     -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    // ]);
    // (manifold_jacobian, manifold_jacobian.transpose())

    let sin_rot_meas = rotation_meas.sin();
    let cos_rot_meas = rotation_meas.cos();
    let R_ij_t = &Matrix3::from_vec(vec![
        cos_rot_meas,
        -sin_rot_meas,
        0.0, // transposed R_ij_t (so R_ij) is displayed
        sin_rot_meas,
        cos_rot_meas,
        0.0,
        0.0,
        0.0,
        1.0,
    ]);
    let delta_pos = (point_j - point_i).data;
    let sin_rot_i = rotation_i.sin();
    let cos_rot_i = rotation_i.cos();
    let last_column_top =
        -sin_rot_i * delta_pos.as_slice()[0] + cos_rot_i * delta_pos.as_slice()[1];
    let last_column_mid =
        -cos_rot_i * delta_pos.as_slice()[0] - sin_rot_i * delta_pos.as_slice()[1];
    let A_ij = R_ij_t
        * &Matrix3::from_vec(vec![
            -cos_rot_i,
            sin_rot_i,
            0.0, // transposed A_ij is displayed
            -sin_rot_i,
            -cos_rot_i,
            0.0,
            last_column_top,
            last_column_mid,
            -1.0,
        ]);
    let B_ij = R_ij_t
        * &Matrix3::from_vec(vec![
            cos_rot_i, -sin_rot_i, 0.0, // transposed B_ij is displayed
            sin_rot_i, cos_rot_i, 0.0, 0.0, 0.0, 1.0,
        ]);

    let mut jacobian = Matrix3x6::from_vec(vec![0.0; 18]);
    jacobian.index_mut((.., ..3)).copy_from(&A_ij);
    jacobian.index_mut((.., 3..)).copy_from(&B_ij);
    (jacobian, jacobian.transpose())
}

#[cfg(test)]
mod tests {
    use crate::optimizer::linear_system::calculate_H_b;
    use crate::parser::json::JsonParser;
    use crate::parser::Parser;

    #[test]
    fn simple_H_b() {
        let graph = JsonParser::parse_file_to_model("test_files/testTrajectory2DAngle.json")
            .unwrap()
            .into();

        let (H, b) = calculate_H_b(&graph);

        dbg!(H);
        dbg!(b);
    }
}
