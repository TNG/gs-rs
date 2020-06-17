# gs-rs [ˈdʒiːzrs]

> "it leadeth me in the paths of righteousness for its name's sake"

gs-rs is an implementation of a monocular graph based SLAM backend in pure rust.


## Example Usage

```
use gs_rs::parser::json::JsonParser;
use gs_rs::parser::Parser;
use gs_rs::optimizer::optimize;
use gs_rs::visualizer::visualize;

fn main() {

    // parse file at "file_path.json" to internal factor graph representation
    let factor_graph = JsonParser::parse_file("file_path.json").unwrap();

    // optimize the factor graph's variables with 5 iterations
    optimize(&factor_graph, 5);

    // display the improved factor graph in a new window
    visualize(&factor_graph);

}
```