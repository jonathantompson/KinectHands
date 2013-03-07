;//
//  edge.cpp
//  KinectHands
//
//  Created by Jonathan Tompson on 6/21/12.
//  Implements the winged edge data structure
//

#include "mesh_simplification/edge.h"
#include "data_str/vector.h"
#include "mesh_simplification/mesh_simplification.h"
#include "exceptions/wruntime_error.h"

using data_str::Vector;
using math::Float3;
using std::wruntime_error;
using std::string;

namespace mesh_simplification {
  math::Float3 Edge::v;
  math::Float3 Edge::w;
  math::Float3 Edge::tmp;
  math::Float3 Edge::normal_f1;
  math::Float3 Edge::normal_f2;

  Edge::Edge(uint32_t v1, uint32_t v2) {
    this->v1 = v1;
    this->v2 = v2;
    this->face_a = true;
    this->face_b = false;
    num_reinsertions = 0;
  }
  
  Edge::Edge() {
    this->face_a = false;
    this->face_b = false;   
    num_reinsertions = 0;
  }
  
  bool serachForHole(Edge* start_edge, uint32_t vertex_to_search) {
    Edge* cur_edge = start_edge;
    do {
      if (!cur_edge->face_a || !cur_edge->face_b) {
        return true;
      }
      cur_edge = cur_edge->nextAnticlockwiseAround(vertex_to_search);
    } while (cur_edge != start_edge);
    return false;
  }
  
  bool Edge::partOfAnEdgeTriangle() {
    if (serachForHole(this, v1)) {
      return true;
    }
    if (serachForHole(this, v2)) {
      return true;
    }
    return false;
  }

  void Edge::calculateNormals(Vector<Float3>* vertices) {
    uint32_t v3;
    v.sub(vertices->at(v2), vertices->at(v1));
    if (face_a) {
      v3 = e1a->v1 == v1 ? e1a->v2 : e1a->v1;
      w.sub(vertices->at(v3), vertices->at(v1));
      Float3::cross(&normal_f1, &w, &v);
      normal_f1.normalize();
    }

    if (face_b) {
      v3 = e1b->v1 == v1 ? e1b->v2 : e1b->v1;
      w.sub(vertices->at(v3), vertices->at(v1));
      Float3::cross(&normal_f2, &v, &w);
      normal_f2.normalize();
    }
  }

  
  void Edge::calcCost(Vector<Float3>* vertices, EdgeCostFunction edge_func) {
    bool part_of_an_edge_triangle = this->partOfAnEdgeTriangle();
    
    if (part_of_an_edge_triangle) {
      cost = MESH_SIMPLIFICATION_CONTOUR_PENALTY;
      return;
    }

    // If we haven't already, calculate the normals
    if (edge_func == EdgeLengthWithCurvature) {
      calculateNormals(vertices);
    }

    switch (edge_func) {
    case EdgeCostFunction::EdgeLengthWithCurvature:
      // Fast and non-linear --> This works best I find.
      cost = Float3::dot(&v, &v) / 
        (Float3::dot(&normal_f1, &normal_f2) + 1.05f);
      break;
    case EdgeCostFunction::EdgeLengthEdgeCost:
      // EDGE LENGTH COST
      v.sub(vertices->at(v2), vertices->at(v1));
      cost = Float3::dot(&v, &v);  // length squared
      break;
    }
  }
  
  Edge* Edge::nextClockwiseAround(uint32_t vertex) {
#if defined(DEBUG) || defined(_DEBUG)
    if (v1 == vertex && v2 == vertex) {
      throw wruntime_error(string("ERROR: nextClockwiseAround() both ") + 
                           string("vertices match the input vertex"));          
    }
    if (v1 != vertex && v2 != vertex) {
      throw wruntime_error(string("ERROR: nextClockwiseAround() neither ") + 
                           string("vertices match the input vertex"));
    }   
#endif
    if (v1 == vertex) {
      return e1b;
    } else {
      return e2a;
    }
  }
  
  Edge* Edge::nextAnticlockwiseAround(uint32_t vertex) {
#if defined(DEBUG) || defined(_DEBUG)
    if (v1 == vertex && v2 == vertex) {
      throw wruntime_error(string("ERROR: nextAnticlockwiseAround() both ") + 
                           string("vertices match the input vertex"));          
    }
    if (v1 != vertex && v2 != vertex) {
      throw wruntime_error(string("ERROR: nextAnticlockwiseAround() neither ") + 
                           string("vertices match the input vertex"));
    }   
#endif
    
    if (v1 == vertex) {
      return e1a;
    } else {
      return e2b;
    }
  }
  
  // print edges --> used for manual debugging
  void Edge::printEdgesAnticlockwiseAround(uint32_t vertex) {
    printf("Edges around vertex %d\n", vertex);
    Edge* cur_edge = this;
    do {
      printf("%d -> %d", cur_edge->v1, cur_edge->v2);
      if (cur_edge->v1 == vertex) {
        printf(" (v1_angle = %.2f deg)\n", cur_edge->v1_angle);
      } else {
        printf(" (v2_angle = %.2f deg)\n", cur_edge->v2_angle);
      }
      cur_edge = cur_edge->nextAnticlockwiseAround(vertex);
    } while (cur_edge != this);
  }
  
  uint32_t Edge::numEdgesOnVertex(uint32_t vertex) {
    uint32_t ret_val = 0;
    Edge* cur_edge = this;
    do {
      ret_val++;
      cur_edge = cur_edge->nextAnticlockwiseAround(vertex);
    } while (cur_edge != this);
    return ret_val;
  }
  
}  // namespace mesh_simplification