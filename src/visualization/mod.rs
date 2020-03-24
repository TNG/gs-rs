use crate::factor_graph::FactorGraph;
use crate::factor_graph::variable::Variable;
use petgraph::Directed;
use crate::factor_graph::factor::Factor;
use petgraph::graph::Graph;
use std::ops::Index;
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use nalgebra::{Translation3, distance, Point2, UnitQuaternion, Vector3};
use petgraph::csr::{Edges, EdgeReference};
use std::env::var;
use crate::factor_graph::factor::Factor::{PositionFactor2D, OdometryFactor2D, ObservationFactor2D};

/// currently only supports 2D graphs
fn add_factor_graph_to_window(window: &mut Window, factor_graph: &FactorGraph) -> SceneNode {
    let mut scene_node = window.add_group();
    factor_graph.node_indices.iter()
        .map(|i| add_variable_with_factors(&mut scene_node, factor_graph.csr.index(i.to_owned()), factor_graph.csr.edges(i.to_owned())))
        .for_each(drop);
    scene_node
}

fn add_variable_with_factors(scene_node: &mut SceneNode, variable: &Box<dyn Variable>, edges: Edges<Factor, Directed, usize>) {
    // add variable position
    let mut variable_object = scene_node.add_sphere(0.1);
    let (var_x, var_y, var_z, var_phi) = (variable.get_pose()[0] as f32, variable.get_pose()[1] as f32, 0.0 as f32, variable.get_pose()[2] as f32);
    variable_object.set_local_translation(Translation3::new(var_x, var_y, var_z));
    match variable.get_type() {
        Vehicle2D => variable_object.set_color(1.0, 0.0, 0.0),
        Landmark2D => variable_object.set_color(0.0, 1.0, 0.0),
    };

    // TODO add variable rotation (display e.g. as arrow)

    // add measurements
    for edge in edges {
        let c = 0.16;
        let mut measurement_object = scene_node.add_cube(c, c, c);

        let factor: &Factor = edge.weight();
        let (meas_x, meas_y, meas_z, meas_phi) = match factor {
            PositionFactor2D(x, y, phi, information_matrix) => (*x as f32, *y as f32, 0.0, *phi as f32),
            OdometryFactor2D(x, y, phi, information_matrix) => (*x as f32 + var_x, *y as f32 + var_y, 0.0, *phi as f32 + var_phi),
            ObservationFactor2D(x, y, phi, information_matrix) => (*x as f32 + var_x, *y as f32 + var_y, 0.0, *phi as f32 + var_phi),
            _ => panic!("Unsupported edge type found in FactorGraph"),
        };
        measurement_object.set_local_translation(Translation3::new(meas_x, meas_y, meas_z));

        match factor {
            PositionFactor2D(x, y, phi, information_matrix) => measurement_object.set_color(1.0, 0.3, 0.3),
            OdometryFactor2D(x, y, phi, information_matrix) => measurement_object.set_color(1.0, 0.6, 0.6),
            ObservationFactor2D(x, y, phi, information_matrix) => measurement_object.set_color(0.45, 1.0, 0.45),
        };

        // TODO decide whether to use this flawed approach to create a line from the source to the relative observed pose
        // let capsule_length = distance(&Point2::new(variable.get_pose()[0], variable.get_pose()[1]), &Point2::new(*x, *y)) * 10.0;
        // let mut capsule = scene_node.add_capsule(0.02, capsule_length as f32);
        // capsule.set_local_translation(Translation3::new((variable.get_pose()[0]) as f32, (variable.get_pose()[1]) as f32, 0.0));
        // capsule.set_local_rotation(UnitQuaternion::from_scaled_axis(Vector3::new((variable.get_pose()[0]) as f32, (variable.get_pose()[1]) as f32, 0.0)))
    }

}



#[cfg(test)]
mod test {
    use log::LevelFilter;
    use super::*;
    use crate::parser::json::JsonParser;
    use crate::parser::Parser;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn test_render_valid_file() {
        init();

        let mut window = Window::new("gs-rs");
        let mut scene_node: SceneNode = match JsonParser::parse_file("test_files/testTrajectory2DAngle.json") {
            Ok(factor_graph) => add_factor_graph_to_window(&mut window, &factor_graph),
            Err(str) => panic!(str),
        };

        while window.render() {

        }
    }
}