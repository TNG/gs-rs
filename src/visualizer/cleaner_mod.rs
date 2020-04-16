//! Handles the graphical user interface.

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
    // add_variables_and_factors(&mut visual_factor_graph, factor_graph);
    visual_factor_graph
}

// fn add_variable() and fn add_factor()?



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
