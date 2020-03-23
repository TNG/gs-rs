use kiss3d::window::Window;
use crate::factor_graph::FactorGraph;
use crate::factor_graph::variable::Variable;
use petgraph::Undirected;
use crate::factor_graph::factor::Factor;
use petgraph::graph::Graph;
use std::ops::Index;

struct VisualFactorGraph {
    window: Window
}

// TODO implement for updated FactorGraph type
// impl From<FactorGraph<'_>> for VisualFactorGraph {
//     fn from(factor_graph: FactorGraph<'_>) -> Self {
//         let visual_factor_graph = VisualFactorGraph{ window: Window::new("gs-rs") };
//         for i in 0..factor_graph.node_count() {
//             let node = factor_graph.index(i).get_pose();
//             unimplemented!()
//         }
//         visual_factor_graph
//     }
// }



#[cfg(test)]
mod test {
    use log::LevelFilter;
    use super::*;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn solver_invertible_test() {
        init();

        let mut window: Window = Window::new("Graph");
        window.add_circle(5.0);
        for _n in 0..100 {
            window.render();
        }
    }
}