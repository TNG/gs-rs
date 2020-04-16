use criterion::{criterion_group, criterion_main, Criterion};
use gs_rs::parser::g2o::G2oParser;
use gs_rs::parser::Parser;
use gs_rs::optimizer::optimize;
use std::time::Duration;

fn bench_optimization(file_name: &str, iterations: usize) {
    let factor_graph = G2oParser::parse_file(&["data_files/benchmark_input/", file_name, ".g2o"].concat()).unwrap();
    optimize(&factor_graph, iterations);
}

fn bench_mit_2d_1(c: &mut Criterion) {
    c.bench_function("MIT_2D", |b| b.iter(|| bench_optimization("MIT_2D", 1)));
}

fn bench_mit_2d_50(c: &mut Criterion) {
    c.bench_function("MIT_2D", |b| b.iter(|| bench_optimization("MIT_2D", 50)));
}

criterion_group!{
    name = benches;
    config = Criterion::default().sample_size(10)/*.measurement_time(Duration::from_secs(15))*/;
    targets = bench_mit_2d_1, bench_mit_2d_50
}
criterion_main!(benches);
