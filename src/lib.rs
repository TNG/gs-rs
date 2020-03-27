#[macro_use]
extern crate log;
#[macro_use]
extern crate approx;
extern crate nalgebra;
extern crate uuid;
extern crate petgraph;
extern crate kiss3d;

pub mod factor_graph;
pub mod solver;
pub mod parser;
pub mod visualization;
pub mod calculator;

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
