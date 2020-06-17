#[macro_use]
extern crate log;
#[macro_use]
extern crate itertools;
#[macro_use]
extern crate approx;
extern crate kiss3d;
extern crate nalgebra;
extern crate petgraph;

// TODO @Samuel: remove all compiler warnings
pub mod factor_graph;
pub mod optimizer;
pub mod parser;
pub mod visualizer;

// TODO @Daniel: can the interface fn and/or tests be removed?

pub fn interface() {
    info!("Hello, I'm gs-rs. I will find your path for you.");
}

#[cfg(test)]
mod tests {
    use log::LevelFilter;

    use crate::interface;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn say_hello() {
        init();
        interface();
    }
}
