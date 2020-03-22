
/*
use crate::factor_graph::FactorGraph;
use crate::factor_graph::variable::landmark_variable_2d::LandmarkVariable2D;
use crate::factor_graph::variable::Variable;
use crate::factor_graph::variable::vehicle_variable_2d::VehicleVariable2D;
use crate::parser::model::{FactorGraphModel, Vertex};
use std::borrow::Borrow;
use std::ops::Deref;

pub fn convert(model: &FactorGraphModel) -> FactorGraph {
    let vertices: Vec<Box<dyn Variable>> = model.vertices.iter()
        .map(|x| convert_vertex(&x))
        .filter_map(Result::ok)
        .collect();

  //  FactorGraph::new(vertices, vec![], vec![] );
    unimplemented!();
}


fn convert_vertex(vertex: &Vertex) -> Result<Box<dyn Variable>, String> {
    match vertex.vertex_type.as_str() {
        "POSE2D_ANGLE" => Ok(Box::new(VehicleVariable2D::from_pose_and_id(vertex.id, vertex.position[0], vertex.position[1], vertex.rotation[0]))),
        _ => Err(format!("Could not identify vertex type {:?}", vertex.vertex_type.as_str())),
    }
}


#[cfg(test)]
mod tests {
    use crate::parser::model::{FactorGraphModel, Vertex};
    use crate::factor_graph::variable::vehicle_variable_2d::VehicleVariable2D;
    use crate::parser::converter::convert_vertex;
    use std::ops::Deref;
    use crate::factor_graph::variable::Variable;
    use crate::factor_graph::variable::VariableType::Vehicle2D;

    #[test]
    fn convert_simple_vertex() {
        let model = Vertex::new( 1, String::from("POSE2D_ANGLE"),  [1.0, 1.0], [3.1415]);
        let vehicle_pose = VehicleVariable2D::from_pose_and_id(1, 1.0, 1.0, 3.1415);

        let vehicle_pose_converted = convert_vertex(&model).unwrap();

        assert_eq!(vehicle_pose_converted.deref().get_id(), vehicle_pose.get_id());
        assert_eq!(vehicle_pose_converted.deref().get_type(), Vehicle2D);
        assert_eq!(vehicle_pose_converted.deref().get_pose(), vec![1.0, 1.0, 3.1415]);

    }
}
*/
