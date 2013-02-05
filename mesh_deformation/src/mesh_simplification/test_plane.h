//
//  test_cube.h
//  KinectHands
//
//  Created by Jonathan Tompson on 6/21/12.
//  A densely meshed cube to test the mesh simplification routines
//

#ifndef MESH_SIMPLIFICATION_TEST_CUBE_HEADER
#define MESH_SIMPLIFICATION_TEST_CUBE_HEADER

#include "data_str/vector.h"
#include "math/math_types.h"
#include "mesh_simplification/edge.h"

namespace mesh_simplification {
  
  // See EdgeDiagram.ppt for details on the layout
  class TestPlane {
  public:
    // simple_plane = true --> just a uniform vertex distribution
    TestPlane(uint32_t subdivisions_per_edge, bool simple_plane, 
              float plane_fraction_holes);
    
    data_str::Vector<math::Float3>* vertices() { return &vertices_; }
    data_str::Vector<math::Float3>* vertices_simplified() { return &vertices_simplified_; }
    data_str::Vector<uint32_t>* indices() { return &indices_; }
    data_str::Vector<uint32_t>* indices_simplified() { return &indices_simplified_; }    
    data_str::Vector<math::Float3>* normals() { return &normals_; }
    
    
  private:
    data_str::Vector<math::Float3> vertices_;
    data_str::Vector<math::Float3> vertices_simplified_;
    data_str::Vector<uint32_t> indices_;
    data_str::Vector<uint32_t> indices_simplified_;
    data_str::Vector<math::Float3> normals_;
  };
};  // namespace mesh_simplification

#endif  // MESH_SIMPLIFICATION_TEST_CUBE_HEADER
