use crate::factor_graph::FactorGraph;
use crate::factor_graph::variable::Variable;
use petgraph::Directed;
use crate::factor_graph::factor::Factor;
use petgraph::graph::Graph;
use std::ops::Index;
use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use nalgebra::{Translation3, distance, Point2, UnitQuaternion, Vector3, Point3};
use petgraph::csr::{Edges, EdgeReference};
use std::env::var;
use crate::factor_graph::factor::Factor::{PositionFactor2D, OdometryFactor2D, ObservationFactor2D};
use petgraph::visit::EdgeRef;

struct VisualFactorGraph {
    scene_node: SceneNode,
    lines: Vec<[Point3<f32>; 3]>,
}

/// currently only supports 2D graphs
fn add_factor_graph_to_window(window: &mut Window, factor_graph: &FactorGraph) -> VisualFactorGraph {
    let mut visual_factor_graph = VisualFactorGraph {
        scene_node: window.add_group(),
        lines: vec![],
    };
    add_variables_and_factors(&mut visual_factor_graph, factor_graph);
    visual_factor_graph
}

fn add_variables_and_factors(visual_factor_graph: &mut VisualFactorGraph, factor_graph: &FactorGraph) {
    for variable_index in &factor_graph.node_indices {
        let variable: &Box<dyn Variable> = factor_graph.csr.index(variable_index.to_owned());

        let mut variable_object = visual_factor_graph.scene_node.add_sphere(0.1);
        let (var_x, var_y, var_z, var_phi) = (variable.get_pose()[0] as f32, variable.get_pose()[1] as f32, 0.0 as f32, variable.get_pose()[2] as f32);
        variable_object.set_local_translation(Translation3::new(var_x, var_y, var_z));
        let mut rotation_object = variable_object.add_capsule(0.02, 2.0);
        rotation_object.set_local_rotation(UnitQuaternion::from_axis_angle(&Vector3::z_axis(), var_phi));
        rotation_object.prepend_to_local_translation(&Translation3::new(0.0, 0.20, 0.0));
        match variable.get_type() {
            Vehicle2D => variable_object.set_color(1.0, 0.0, 0.0),
            Landmark2D => variable_object.set_color(0.0, 1.0, 0.0),
        };

        // TODO adjust what to display individually for each factor type
        for edge in factor_graph.csr.edges(variable_index.to_owned()) {
            let mut measurement_object = visual_factor_graph.scene_node.add_cube(0.16, 0.16, 0.16);
            let factor: &Factor = edge.weight();
            let (meas_x, meas_y, meas_z, meas_phi) = match factor { // TODO consider variable's rotation for Odometry/Observation
                PositionFactor2D(x, y, phi, information_matrix) => (*x as f32, *y as f32, 0.0, *phi as f32),
                OdometryFactor2D(x, y, phi, information_matrix) => (*x as f32 + var_x, *y as f32 + var_y, 0.0, *phi as f32 + var_phi),
                ObservationFactor2D(x, y, phi, information_matrix) => (*x as f32 + var_x, *y as f32 + var_y, 0.0, *phi as f32 + var_phi),
                _ => panic!("Unsupported edge type found in FactorGraph"),
            };
            measurement_object.set_local_translation(Translation3::new(meas_x, meas_y, meas_z));
            let mut measured_rotation_object = measurement_object.add_capsule(0.04, 1.5);
            measured_rotation_object.set_local_rotation(UnitQuaternion::from_axis_angle(&Vector3::z_axis(), meas_phi));
            measured_rotation_object.prepend_to_local_translation(&Translation3::new(0.0, 0.15, 0.0));
            let (r, g, b) = match factor {
                PositionFactor2D(x, y, phi, information_matrix) => (1.0, 0.3, 0.3),
                OdometryFactor2D(x, y, phi, information_matrix) => (1.0, 0.6, 0.6),
                ObservationFactor2D(x, y, phi, information_matrix) => (0.45, 1.0, 0.45),
            };
            measurement_object.set_color(r, g, b);

            let meas_point = Point3::new(meas_x, meas_y, meas_z);
            let source_point = Point3::new(var_x, var_y, var_z);
            let target = factor_graph.csr.index(edge.target());
            let target_point = Point3::new(target.get_pose()[0] as f32, target.get_pose()[1] as f32, 0.0 as f32);
            let color_point = Point3::new(r, g, b);
            visual_factor_graph.lines.push([meas_point, source_point, color_point]);
            visual_factor_graph.lines.push([meas_point, target_point, color_point]);
            visual_factor_graph.lines.push([source_point, target_point, color_point]);
        }
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
        let mut visual_factor_graph: VisualFactorGraph = match JsonParser::parse_file("test_files/testTrajectory2DAngle.json") {
            Ok(factor_graph) => add_factor_graph_to_window(&mut window, &factor_graph),
            Err(str) => panic!(str),
        };

        while window.render() {
            visual_factor_graph.lines.iter()
                .map(|line| window.draw_line(&line[0], &line[1], &line[2]))
                .for_each(drop);
        }
    }
}