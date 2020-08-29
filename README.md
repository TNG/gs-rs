# gs-rs [ˈdʒiːzrs] - GraphSLAM in Rust
[![Travis CI](https://travis-ci.com/deltapi/gs-rs-test.png?branch=master)](https://travis-ci.com/deltapi/gs-rs-test)
[![Coverage Status](https://coveralls.io/repos/deltapi/gs-rs-test/badge.png?branch=master)](https://coveralls.io/r/deltapi/gs-rs-test)
[![License-MIT](https://img.shields.io/github/license/deltapi/gs-rs-test.svg)](https://github.com/deltapi/gs-rs-test/blob/master/LICENSE-MIT.md)
[![License-Apache](https://img.shields.io/github/license/deltapi/gs-rs-test.svg)](https://github.com/deltapi/gs-rs-test/blob/master/LICENSE-APACHE.md)
[![Issues](https://img.shields.io/github/issues/deltapi/gs-rs-test.svg)](https://github.com/deltapi/gs-rs-test/issues)
[![Forks](https://img.shields.io/github/forks/deltapi/gs-rs-test.svg)](https://github.com/deltapi/gs-rs-test/network)
[![Stars](https://img.shields.io/github/stars/deltapi/gs-rs-test.svg)](https://github.com/deltapi/gs-rs-test/stargazers)
[![Crates.io Version](https://img.shields.io/crates/v/gs-rs.svg)](https://crates.io/crates/gs-rs)



**gs-rs** is an implementation of a monocular graph based SLAM backend in pure rust. It is largely inspired 
by the General Graph Optimizer [g2o](https://github.com/RainerKuemmerle/g2o) of Rainer Kümmerle, though it is giving it 
a rusty touch.

## Overview

* [Examples](examples/README.md)
* [Data Files](data_files/README.md)
* [Contributing](CONTRIBUTING.md)
* [Code of Conduct](CODE_OF_CONDUCT.md)
* [MIT License](LICENSE-MIT.md)
* [Apache License](LICENSE-APACHE.md)

## Documentation
A brief introduction to the algorithms used is available as a Jupyter Notebook [here](doc/documentation.ipynb)

## Building Instructions
* Install the [rust toolchain](https://www.rust-lang.org/learn/get-started)
* Clone the repository
* Execute `cargo build --release` in the root directory

## Example Usage

In addition to the code snippet below, further examples for 
different use cases can be found in `examples/`. 
Information regarding that directory can be found in its respective 
[README](examples/README.md) file.

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

The file format `.g2o` as well as the initial implementation in `C++` were 
initially part of the project [g2o](https://github.com/RainerKuemmerle/g2o).

The datasets used in `data_files/` and `examples/` have been described by Carlone et al. and adapted to include fixed vertices. Other modifications have been performed as well to demonstrate the different vertex and edge types supported in gs-rs. 
The original datasets as well as more examples can be found online.
Attribution and details may also be found in the [data sets section](data_files/README.md).

## Note
We, the contributers are fully aware of the fact that we cannot prohibit military usage of open source software. 
But we want to state very clearly that we discourage any use of our software with the intention of harming people.  

## License

© 2020 Samuel Valenzuela, TNG Technology Consulting GmbH

© 2020 Florian Rohm, TNG Technology Consulting GmbH

© 2020 Daniel Pape, TNG Technology Consulting GmbH

**gs-rs** is licensed under the Apache License, Version 2.0 (cf. [here](LICENSE-APACHE.md) or
http://www.apache.org/licenses/LICENSE-2.0) or the MIT license (cf. [here](LICENSE-MIT.md) or http://opensource.org/licenses/MIT), at your option.
