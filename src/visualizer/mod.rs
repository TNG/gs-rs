//! Handles the graphical user interface.

use kiss3d::scene::SceneNode;
use nalgebra::{Point3, UnitQuaternion, Vector3, Translation3, Rotation3};
use kiss3d::window::Window;
use kiss3d::camera::ArcBall;
use petgraph::visit::EdgeRef;
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
        .for_each(|i| add_var(&mut visual_factor_graph, factor_graph.get_var(*i)));

    factor_graph.node_indices.iter()
        .for_each(|i| factor_graph.csr.edges(*i)
            .for_each(|edge| add_factor(&mut visual_factor_graph, edge.weight(), factor_graph.get_var(edge.source()), factor_graph.get_var(edge.target()))));

    visual_factor_graph
}

fn add_var(visual_factor_graph: &mut VisualFactorGraph, var: &Box<dyn Variable>) {
    let var_point = get_point_from_2d(&var.get_content());
    let mut var_object = add_var_core(visual_factor_graph, &var_point);
    handle_var_rotation(var, &mut var_object);
    color_var_object(var, &mut var_object);
}

fn add_factor(visual_factor_graph: &mut VisualFactorGraph, factor: &Factor, source: &Box<dyn Variable>, target: &Box<dyn Variable>) {
    let meas_point = calc_meas_point(factor, source);
    let mut meas_object = add_factor_core(visual_factor_graph, &meas_point);
    handle_factor_rotation(factor, &mut meas_object, source);
    color_meas_object(factor, &mut meas_object);
    add_factor_lines(visual_factor_graph, factor, meas_point, get_point_from_2d(&source.get_content()), get_point_from_2d(&target.get_content()));
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

fn calc_meas_point(factor: &Factor, source: &Box<dyn Variable>) -> Point3<f32> {
    let factor_point = get_point_from_2d(&factor.constraint);
    match factor.factor_type {
        Position2D => factor_point,
        Odometry2D | Observation2D => {
            let source_rot = get_rot_from_2d(&source.get_content());
            let local_point = Rotation3::new(Vector3::z() * source_rot) * factor_point;
            (get_point_from_2d(&source.get_content()).coords + local_point.coords).into()
        },
        Odometry3D => unimplemented!(),
    }
}

fn add_factor_core(visual_factor_graph: &mut VisualFactorGraph, meas_point: &Point3<f32>) -> SceneNode {
    let mut meas_object = visual_factor_graph.scene_node.add_cube(0.16, 0.16, 0.16);
    meas_object.set_local_translation(meas_point.coords.into());
    meas_object
}

fn handle_factor_rotation(factor: &Factor, meas_object: &mut SceneNode, source: &Box<dyn Variable>) {
    if factor.factor_type == Position2D || factor.factor_type == Odometry2D {
        let factor_rot = get_rot_from_2d(&factor.constraint);
        let meas_rot = match factor.factor_type {
            Position2D => factor_rot,
            Odometry2D => factor_rot + get_rot_from_2d(&source.get_content()),
            _ => panic!("Internal Error at visualization of unsupported rotation."),
        };
        let mut meas_rot_object = meas_object.add_capsule(0.04, 1.5);
        meas_rot_object.set_local_rotation(
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), meas_rot)
        );
        meas_rot_object.prepend_to_local_translation(&Translation3::new(0.0, 0.15, 0.0));
    }
}

fn color_meas_object(factor: &Factor, meas_object: &mut SceneNode) {
    let (r, g, b) = get_factor_color(factor);
    meas_object.set_color(r, g, b);
}

fn add_factor_lines(visual_factor_graph: &mut VisualFactorGraph, factor: &Factor, meas_point: Point3<f32>, source_point: Point3<f32>, target_point: Point3<f32>) {
    let (r, g, b) = get_factor_color(factor);
    visual_factor_graph.lines.push([meas_point, source_point, Point3::new(r, g, b)]);
    if factor.factor_type == Observation2D {
        visual_factor_graph.lines.push([meas_point, target_point, Point3::new(r, g, b)]);
    } else if factor.factor_type == Odometry2D {
        visual_factor_graph.lines.push([source_point, target_point, Point3::new(1.0, 1.0, 1.0)]);
    }
}

fn get_factor_color(factor: &Factor) -> (f32, f32, f32) {
    match factor.factor_type {
        Position2D => (1.0, 0.5, 0.5),
        Odometry2D | Odometry3D => (0.5, 0.5, 1.0),
        Observation2D => (0.5, 1.0, 0.5),
    }
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
    #[ignore]
    fn test_visualize() {
        init();

        let factor_graph = JsonParser::parse_file("data_files/fullTestGraph.json").unwrap();
        visualize(&factor_graph);
    }
}
