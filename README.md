# gs-rs [ˈdʒiːzrs]

gs-rs is an implementation of a monocular graph based SLAM backend in pure rust.


## Example Usage

In addition to the code snippet below, further examples for different use cases can be found in examples/. Information regarding that directory can be found in its respective README file.

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

## Attribution

The file format .g2o as well as the initial implementation in C++ were initially part of the project g2o: https://github.com/RainerKuemmerle/g2o

## Datasets

The datasets used in data_files/ and examples/ have been described by Carlone et al. [1] and adapted to include fixed vertices. Other modifications have been performed as well to demonstrate the different vertex and edge types supported in gs-rs. The original datasets as well as more examples can be found online [2].

[1] L. Carlone, R. Tron, K. Daniilidis, and F. Dellaert. Initialization Techniques for 3D SLAM: a Survey on Rotation Estimation and its Use in Pose Graph Optimization. In IEEE Intl. Conf. on Robotics and Automation (ICRA), pages 4597-4604, 2015.

[2] L. Carlone, "Datasets." [Online]. Available: https://lucacarlone.mit.edu/datasets/


## License

Copyright 2020 Samuel Valenzuela, TNG Technology Consulting GmbH

Copyright 2020 Daniel Pape, TNG Technology Consulting GmbH

gs-rs is licensed under the Apache License, Version 2.0 (LICENSE-APACHE or
http://www.apache.org/licenses/LICENSE-2.0) or the MIT license (LICENSE-MIT or http://opensource.org/licenses/MIT), at your option.