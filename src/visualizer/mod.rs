//! Handles the graphical user interface.

use kiss3d::scene::SceneNode;
use nalgebra::{Point3, UnitQuaternion, Vector3, Translation3};
use kiss3d::window::Window;
use kiss3d::camera::ArcBall;
use crate::factor_graph::FactorGraph;
use crate::factor_graph::factor::{Factor, FactorType::*};
use crate::factor_graph::variable::{Variable, VariableType::*};

struct VisualFactorGraph {
    scene_node: SceneNode,
    lines: Vec<[Point3<f32>; 3]>,
}

// TODO does not work multiple times in a single execution of the program
// Things tried so far:
// - Google -> nobody seems to have mentioned this problem before
// - make window static -> causes problems as it has to be mutable and can not be moved
// Possible solutions:
// - do not tackle this problem, but instead support being able to visualize several factor graphs and jump between them
/// Displays the visualization of the given factor graph in a new window.
/// Does not work multiple times in a single execution of the program.
pub fn visualize(factor_graph: &FactorGraph) {
    let mut window = Window::new("gs-rs");
    let visual_factor_graph = add_factor_graph_to_window(&mut window, &factor_graph);
    let mut cam = ArcBall::new(Point3::new(0.0, 0.0, 50.0), Point3::new(0.0, 0.0, 0.0)); // TODO dynamic initial position
    while window.render_with_camera(&mut cam) {
        visual_factor_graph.lines.iter()
            .for_each(|line| window.draw_line(&line[0], &line[1], &line[2]));
    }
}

fn add_factor_graph_to_window(window: &mut Window, factor_graph: &FactorGraph) -> VisualFactorGraph {
    let mut visual_factor_graph = VisualFactorGraph {
        scene_node: window.add_group(),
        lines: vec![],
    };

    factor_graph.node_indices.iter()
        .for_each(|i| add_var(&mut visual_factor_graph, factor_graph.get_var_at_csr_index(*i)));

    visual_factor_graph
}

fn add_var(visual_factor_graph: &mut VisualFactorGraph, var: &Box<dyn Variable>) {
    let var_point = get_point_from_2d(&var.get_content());
    let mut var_object = add_var_core(visual_factor_graph, &var_point);
    handle_var_rotation(var, &mut var_object);
    color_var_object(var, &mut var_object);
}

fn add_var_core(visual_factor_graph: &mut VisualFactorGraph, var_point: &Point3<f32>) -> SceneNode {
    let mut var_object = visual_factor_graph.scene_node.add_sphere(0.1);
    var_object.set_local_translation(var_point.coords.into());
    var_object
}

fn handle_var_rotation(var: &Box<dyn Variable>, var_object: &mut SceneNode) {
    if var.get_type() == Vehicle2D {
        let mut rot_object = var_object.add_capsule(0.02, 2.0);
        rot_object.set_local_rotation(
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), get_rot_from_2d(&var.get_content()))
        );
        rot_object.prepend_to_local_translation(&Translation3::new(0.0, 0.20, 0.0));
    }
}

fn color_var_object(var: &Box<dyn Variable>, var_object: &mut SceneNode) {
    match var.get_type() {
        Vehicle2D | Vehicle3D => var_object.set_color(1.0, 0.0, 0.0),
        Landmark2D => var_object.set_color(0.0, 1.0, 0.0),
    };
}

fn get_point_from_2d(content: &[f64]) -> Point3<f32> {
    Point3::new(
        content[0] as f32,
        content[1] as f32,
        0.0 as f32,
    )
}

fn get_rot_from_2d(content: &[f64]) -> f32 {
    content[2] as f32
}

#[cfg(test)]
mod test {
    use log::LevelFilter;
    use crate::parser::json::JsonParser;
    use crate::parser::Parser;
    use super::*;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn test_visualize() {
        init();

        let factor_graph = JsonParser::parse_file("data_files/fullTestGraph.json").unwrap();
        visualize(&factor_graph);
    }
}
