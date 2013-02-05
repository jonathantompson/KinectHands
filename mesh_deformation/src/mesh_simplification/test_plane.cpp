//
//  test_cube.cpp
//  KinectHands
//
//  Created by Jonathan Tompson on 6/21/12.
//  A densely meshed cube to test the mesh simplification routines
//

#include "mesh_simplification/test_plane.h"

using math::Float3;

namespace mesh_simplification {
  
  Float3 tmp1;
  Float3 tmp2;
  void calculateNormal(Float3* normal, Float3* pt0, Float3* pt1, Float3* pt2) {
    tmp1.sub(pt0, pt1);
    tmp2.sub(pt2, pt1);
    normal->cross(&tmp1, &tmp2);
    normal->normalize();
    if (normal->m[2] > 0) {
      normal->scale(-1);
    }
  }
  
  TestPlane::TestPlane(uint32_t subdivisions_per_edge, bool simple_plane,
                       float plane_fraction_holes) {
    uint32_t vertices_per_face = (subdivisions_per_edge + 1) * 
                                 (subdivisions_per_edge + 1);
    vertices_.capacity(vertices_per_face);
    Float3 normal;
    for (uint32_t v = 0; v <= subdivisions_per_edge; v++) {
      for (uint32_t u = 0; u <= subdivisions_per_edge; u++) {
        float u_f = static_cast<float>(u) / 
        static_cast<float>(subdivisions_per_edge);
        float v_f = static_cast<float>(v) / 
        static_cast<float>(subdivisions_per_edge);
        float X, Y, Z;
        if (!simple_plane) {
          X = u_f * u_f * u_f;
          Y = v_f * v_f;
          Z = (1.0f - v_f - u_f);
          Z = Z*Z;
        } else {
          X = u_f;
          Y = v_f;
          Z = 0;
        }
        vertices_.pushBack(Float3(2.0f*X, 2.0f*Y, 1.0f*Z));
      }
    }
    
    for (uint32_t v = 0; v < subdivisions_per_edge; v++) {
      for (uint32_t u = 0; u < subdivisions_per_edge; u++) {
        uint32_t v0 = (subdivisions_per_edge+1)*v + u;
        uint32_t v1 = (subdivisions_per_edge+1)*v + u+1;
        uint32_t v2 = (subdivisions_per_edge+1)*(v+1) + u;
        uint32_t v3 = (subdivisions_per_edge+1)*(v+1) + u+1;
        calculateNormal(&normal, &vertices_[v0], 
                        &vertices_[v1], &vertices_[v3]);
        normals_.pushBack(normal);
        indices_.pushBack(v0);
        indices_.pushBack(v1);
        indices_.pushBack(v3);
        calculateNormal(&normal, &vertices_[v0], 
                        &vertices_[v3], &vertices_[v2]);        
        normals_.pushBack(normal);
        indices_.pushBack(v0);
        indices_.pushBack(v3);
        indices_.pushBack(v2);        
      }
    }
    
    uint32_t starting_num_faces = indices_.size() / 3;
    uint32_t num_faces = starting_num_faces;
    
    /*
    // Remove a specific face:
    uint32_t remove_face = 450;
    indices_[remove_face*3] = indices_[(num_faces-1)*3];
    indices_[remove_face*3+1] = indices_[(num_faces-1)*3+1];
    indices_[remove_face*3+2] = indices_[(num_faces-1)*3+2];
    indices_.popBack();
    indices_.popBack();
    indices_.popBack();
    num_faces--;
     */
    
    // now randomly remove 1/5 of the faces
    srand (0);
    for (uint32_t i = 0; i < starting_num_faces * plane_fraction_holes; i ++) {
      uint32_t rand_face = rand() % num_faces;
      // Swap the random face with the last face and reduce the aray size
      indices_[rand_face*3] = indices_[(num_faces-1)*3];
      indices_[rand_face*3+1] = indices_[(num_faces-1)*3+1];
      indices_[rand_face*3+2] = indices_[(num_faces-1)*3+2];
      indices_.popBack();
      indices_.popBack();
      indices_.popBack();
      num_faces--;
    }
  }
}  // namespace mesh_simplification