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


## License

Copyright 2020 Samuel Valenzuela <!-- TODO or TNG Technology Consulting GmbH? Or something else? -->
 
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
 
    http://www.apache.org/licenses/LICENSE-2.0
 
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.