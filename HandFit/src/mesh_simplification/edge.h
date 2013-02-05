//
//  edge.h
//  KinectHands
//
//  Created by Jonathan Tompson on 6/21/12.
//  Implements the winged edge data structure
//

#ifndef MESH_SIMPLIFICATION_EDGE_HEADER
#define MESH_SIMPLIFICATION_EDGE_HEADER

#include "math/math_types.h"

namespace data_str { template <typename T> class Vector; }

namespace mesh_simplification {
  typedef enum {
    EdgeLengthEdgeCost,  // cost = edge_length
    EdgeLengthWithCurvature,  // cost = (edge_length) / (dot(n_f1, n_f2) + 1.05)
  } EdgeCostFunction;

  typedef enum {
    NoEdgeRemovalConstraints,  // Faster, most edges are OK (excluding boundry)
    EdgeRemovalConstrainNonManifold,  // Prevent non-manifold output geometry
  } EdgeRemovalConstraint;
  
  // See EdgeDiagram.ppt for details on the layout
  class Edge {
  public:
    Edge(uint32_t v1, uint32_t v2);
    Edge();
    uint32_t v1;  // v1 -> v2 is anticlockwise edge direction
    uint32_t v2;
    Edge* e1a;  
    Edge* e1b;
    Edge* e2a;
    Edge* e2b;
    bool face_a;
    bool face_b;
    float v1_angle;
    float v2_angle;
    float cost;
    uint32_t heap_index;
    uint32_t num_reinsertions;

    void calcCost(data_str::Vector<math::Float3>* vertices, 
      EdgeCostFunction edge_func);
    bool partOfAnEdgeTriangle();
    
    Edge* nextClockwiseAround(uint32_t vertex);
    Edge* nextAnticlockwiseAround(uint32_t vertex);
    
    void printEdgesAnticlockwiseAround(uint32_t vertex);
    
    uint32_t numEdgesOnVertex(uint32_t vertex);
    
  private:
    static math::Float3 v;
    static math::Float3 w;
    static math::Float3 tmp;
    static math::Float3 normal_f1;
    static math::Float3 normal_f2;

    void calculateNormals(data_str::Vector<math::Float3>* vertices);
  };
};  // namespace mesh_simplification

#endif  // MESH_SIMPLIFICATION_EDGE_HEADER
