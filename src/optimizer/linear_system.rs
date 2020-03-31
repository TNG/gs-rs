use crate::factor_graph::factor::{Factor, FactorType::*};
use crate::factor_graph::FactorGraph;
use nalgebra::{DMatrix, DVector, Matrix3x6, Matrix6x3, Vector3, Rotation2, Point2, RowVector2, Vector2, RowVector3};
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
    let var_point = &Point2::new(
        variable.get_pose()[0],
        variable.get_pose()[1],
    );
    let var_rotation = variable.get_pose()[2];
    for edge in factor_graph.csr.edges(variable_index) {
        let factor: &Factor = edge.weight();
        let omega_k = &factor.information_matrix.content;
        let var_matr_index = index_map[&variable_index];
        let var_range = 3 * var_matr_index..3 * (var_matr_index + 1);
        match factor.factor_type {
            Position2D => {
                handle_position_2d(&mut H, &mut b, var_point, var_rotation, factor, omega_k, &var_range)
            }
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
    let (jacobi, jacobi_t) = calculate_2d_jacobians();
    let right_mult = omega_k * jacobi;
    let H_k = jacobi_t * &right_mult;
    let target_matr_index = index_map[&target_id];
    let target_range = 3 * target_matr_index..3 * (target_matr_index + 1);
    let H_i_i = H_k.index((..3, ..3));
    let updated_H_i_i = &(H.index((var_range.clone(), var_range.clone())) + H_i_i);
    H.index_mut((var_range.clone(), var_range.clone()))
        .copy_from(updated_H_i_i);
    let H_i_j = H_k.index((..3, 3..));
    let updated_H_i_j = &(H.index((var_range.clone(), target_range.clone())) + H_i_j);
    H.index_mut((var_range.clone(), target_range.clone()))
        .copy_from(updated_H_i_j);
    let H_j_i = H_k.index((3.., ..3));
    let updated_H_j_i = &(H.index((target_range.clone(), var_range.clone())) + H_j_i);
    H.index_mut((target_range.clone(), var_range.clone()))
        .copy_from(updated_H_j_i);
    let H_j_j = H_k.index((3.., 3..));
    let updated_H_j_j = &(H.index((target_range.clone(), target_range.clone())) + H_j_j);
    H.index_mut((target_range.clone(), target_range.clone()))
        .copy_from(updated_H_j_j);

    let target = factor_graph.csr.index(target_id);
    let target_point = Point2::new(
        target.get_pose()[0],
        target.get_pose()[1],
    );
    let target_rotation = target.get_pose()[2];
    // e_ij_x = (R_ij_T (R_i_T (x_j - x_i) - x_ij))
    let e_ij_x = Rotation2::new(-factor.constraint[2]) * (Rotation2::new(-var_rotation) * (target_point - var_point) - Vector2::new(factor.constraint[0], factor.constraint[1]));
    // e_ij_r = (phi_j - phi_i - phi_ij)
    let e_ij_r = target_rotation - var_rotation - factor.constraint[2];
    let e_k_t = RowVector3::from_vec(vec![e_ij_x.data.to_vec()[0], e_ij_x.data.to_vec()[1], e_ij_r]);
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
    let updated_H_k = &(H.index((var_range.clone(), var_range.clone())) + omega_k);
    H.index_mut((var_range.clone(), var_range.clone()))
        .copy_from(updated_H_k);

    let var_position = var_point.coords.data;
    let var_vector = Vector3::new(var_position.as_slice()[0], var_position.as_slice()[1], var_rotation);
    let e_k_t = (Vector3::from_vec(factor.constraint.clone()) - var_vector).transpose();
    let b_k = e_k_t * omega_k;
    let updated_b_k = &(b.index((var_range.clone(), ..)) + b_k.transpose());
    b.index_mut((var_range.clone(), ..)).copy_from(updated_b_k);
}

fn calculate_2d_jacobians() -> (Matrix3x6<f64>, Matrix6x3<f64>) {
    let jacobi = Matrix3x6::from_vec(vec![
        -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    ]);
    (jacobi, jacobi.transpose())
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
