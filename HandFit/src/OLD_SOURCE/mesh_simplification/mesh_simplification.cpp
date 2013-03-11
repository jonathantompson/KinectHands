//
//  mesh_simplification.cpp
//  KinectHands
//
//  Created by Jonathan Tompson on 6/21/12.
//

#include <iostream>
#include <string>
#include "mesh_simplification/mesh_simplification.h"
#include "mesh_simplification/min_heap_edges.h"
#include "exceptions/wruntime_error.h"

using data_str::Vector;
using math::Float3;
using math::Float2;
using std::wruntime_error;
using std::wstring;

namespace mesh_simplification {

  MeshSettings::MeshSettings() {
    // this->normal_method = RobustNormalApproximation;
    this->normal_method = SimpleNormalApproximation;

    // this->edge_cost_func = EdgeLengthWithCurvature;
    this->edge_cost_func = EdgeLengthEdgeCost;

    // this->vertex_merge_method = MidpointVertexMerge;
    this->vertex_merge_method = StaticVertexMerge;

    // this->edge_constraint = EdgeRemovalConstrainNonManifold;
    this->edge_constraint = NoEdgeRemovalConstraints;
  }
  
  MeshSettings& MeshSettings::operator= (const MeshSettings& other) {
    if (this == &other) {  // if both point to the same memory
      return *this; 
    }
    this->normal_method = other.normal_method;
    this->edge_cost_func = other.edge_cost_func;
    this->vertex_merge_method = other.vertex_merge_method;
    this->edge_constraint = other.edge_constraint;
    return *this;
  }

  MeshSimplification::MeshSimplification(const MeshSettings& settings) {
    settings_ = settings;
  }

  void MeshSimplification::simplifyMesh(uint32_t edge_reduction, 
    Vector<Float3>* vertices, Vector<uint32_t>* indices, 
    Vector<Float3>* normals_return, data_str::Vector<math::Float3>* colors) {
    facesToWingedEdge(&we_, normals_return, vertices, indices);
    cullEdges(&we_, normals_return, vertices, colors, edge_reduction);
    wingedEdgeToFaces(&we_, normals_return, vertices, indices);
  }
  
  // Some assumptions.
  // indices - contain only valid vertices in the vertex list
  // The input mesh is LOCALLY MANIFOLD.  This is very important.  There can be
  // holes in the mesh, and the mesh can have an arbitrary number of contour
  // edges, but the normals of the faces surrounding a vertex must not flip.
  void MeshSimplification::facesToWingedEdge(Vector<Edge>* we, 
                                             Vector<Float3>* normals_return,
                                             Vector<Float3>* vertices, 
                                             Vector<uint32_t>* indices) {
    if (we->capacity() < indices->size()) {
      we->capacity(indices->size());  // allocate for the worst case num edges
    }
    we->resize(0);
    
    if (vertex_edges_.capacity() < vertices->size()) {
      vertex_edges_.capacity(vertices->size());  // Resize it if necessary
    }
    vertex_edges_.resize(vertices->size());
    
    if (vertex_reference_vectors_.capacity() < vertices->size()) {
      vertex_reference_vectors_.capacity(vertices->size());
    }
    vertex_reference_vectors_.resize(vertices->size());
    
    // Set all the vertices as being attached to NO edges
    memset(vertex_edges_.at(0), NULL, 
           sizeof(vertex_edges_[0]) * vertex_edges_.size());
    
    calcNormalsPerVertex(normals_return, vertices, indices);
    
    // For each face in the index array add an edge
    for (uint32_t i = 0; i < indices->size(); i+=3) {
      // Edge: V1 -> V2, edge V2 -> V3, edge V3 -> V1
      insertEdges(we, normals_return, vertices, indices, indices->at(i));
    }
  }

  float MeshSimplification::calcAngleSafe(Float3* pt0, Float3* pt1, 
    Float3* pt2)  {
    tmp1_.sub(pt0, pt1);
    tmp2_.sub(pt2, pt1);
    tmp1_.normalize();
    tmp2_.normalize();
    float dot = Float3::dot(&tmp1_, &tmp2_);
    dot = dot > 1 ? 1 : dot;
    dot = dot < -1 ? -1 : dot;
    return acosf(dot);
  }
  
  void MeshSimplification::calcNormalsPerVertex(Vector<Float3>* normals,
                                                Vector<Float3>* vertices, 
                                                Vector<uint32_t>* indices) {
    if (normals->capacity() < vertices->size()) {
      normals->capacity(vertices->size());
    }
    normals->resize(vertices->size());
    
    // Set all the normals as 0
    for (uint32_t i = 0; i < normals->size(); i++) {
      normals->at(i)->zeros();
    }
    
    // For each face in the index array, calculate its normal and add it to the
    // normal accumulation
    Float3 cur_normal;
    Float3* norm;
    for (uint32_t i = 0; i < indices->size(); i+=3) {
      uint32_t v1 = (*indices)[i];
      uint32_t v2 = (*indices)[i+1];
      uint32_t v3 = (*indices)[i+2];
      
      switch (settings_.normal_method) {
      case SimpleNormalApproximation:
        // METHOD 1 --> FAST
        // If you sum the normal weighted by the tri area, then you get a better
        // average normal.  More importantly, if you calculate the normal by 
        // cross product then you get a normal whos lenght is 2 x the triangle 
        // area, so summing these gives us the correct weights.
        calcNormalUnNormalized(&cur_normal, vertices->at(v1), vertices->at(v2), 
                             vertices->at(v3));
        Float3::add(normals->at(v1), normals->at(v1), &cur_normal);
        Float3::add(normals->at(v2), normals->at(v2), &cur_normal);
        Float3::add(normals->at(v3), normals->at(v3), &cur_normal);
        break;
      case RobustNormalApproximation:
        // METHOD 2 --> EXPENSIVE
        // Weight the normals by the angle they form at the vertex.
        float angle = calcNormalAndAngle(&cur_normal, vertices->at(v1), 
          vertices->at(v2), vertices->at(v3));
        norm = normals->at(v2);
        norm->m[0] += cur_normal[0] * angle;
        norm->m[1] += cur_normal[1] * angle;
        norm->m[2] += cur_normal[2] * angle;
        angle = calcAngleSafe(vertices->at(v3), vertices->at(v1), 
          vertices->at(v2));
        norm = normals->at(v1);
        norm->m[0] += cur_normal[0] * angle;
        norm->m[1] += cur_normal[1] * angle;
        norm->m[2] += cur_normal[2] * angle;
        angle = calcAngleSafe(vertices->at(v2), vertices->at(v3), 
          vertices->at(v1));
        norm = normals->at(v3);
        norm->m[0] += cur_normal[0] * angle;
        norm->m[1] += cur_normal[1] * angle;
        norm->m[2] += cur_normal[2] * angle;
        break;
      }
    }
    
    for (uint32_t i = 0; i < normals->size(); i++) { 
      // No need to divide by the number of faces...  Just normalize
      normals->at(i)->normalize();
    }
  }

  void MeshSimplification::updateVertexNormal(Vector<Float3>* normals, 
    Vector<Float3>* vertices, Edge* edge, uint32_t vertex_to_patch) {
    Edge* cur_edge = edge;
    Float3 cur_normal;
    uint32_t v1, v2, v3;
    do {
      v1 = MAX_UINT32;
      // Add the anti-clockwise face to the running sum
      if (cur_edge->v1 == vertex_to_patch) {
        if (cur_edge->face_a) {
          v1 = cur_edge->v1;
          v2 = cur_edge->v2;
          v3 = (cur_edge->e1a->v1) == v1 ? cur_edge->e1a->v2 : cur_edge->e1a->v1;
        } else if (cur_edge->face_b) {
          v1 = cur_edge->v2;
          v2 = cur_edge->v1;
          v3 = (cur_edge->e2b->v1) == v1 ? cur_edge->e2b->v2 : cur_edge->e2b->v1;
        }
      }
      if (v1 != MAX_UINT32) {
        switch (settings_.normal_method) {
        case SimpleNormalApproximation:
          calcNormalUnNormalized(&cur_normal, vertices->at(v1), vertices->at(v2), 
            vertices->at(v3));
          Float3::add(normals->at(vertex_to_patch), 
                      normals->at(vertex_to_patch), 
                      &cur_normal);
          break;
        case RobustNormalApproximation:
          calcNormal(&cur_normal, vertices->at(v1), vertices->at(v2), 
            vertices->at(v3));
          float angle = calcAngleSafe(vertices->at(v3), vertices->at(v1), 
            vertices->at(v2));
          Float3* norm = normals->at(vertex_to_patch);
          norm->m[0] += cur_normal[0] * angle;
          norm->m[1] += cur_normal[1] * angle;
          norm->m[2] += cur_normal[2] * angle;
          break;
        }
      }
      cur_edge = cur_edge->nextAnticlockwiseAround(vertex_to_patch);
    } while (cur_edge != edge);

    normals->at(vertex_to_patch)->normalize();
  }
  
  float MeshSimplification::calcEdgeAngle(Float3* v1, Float3* v2, 
                                          Float3* normal, 
                                          Float3* reference_vector) {
    // Project the edge onto the normal
    tmp1_.sub(v2, v1);  // Vector from v1 -> v2
    float proj_vec_onto_normal = Float3::dot(&tmp1_, normal);
    tmp1_[0] -= (normal->m[0] * proj_vec_onto_normal);
    tmp1_[1] -= (normal->m[1] * proj_vec_onto_normal);
    tmp1_[2] -= (normal->m[2] * proj_vec_onto_normal);     
    tmp1_.normalize();
    
    // tmp1_ is now the normalized projection of the vector onto the plane, now 
    // get the angle between tmp1_ and the reference vector
    float dot = Float3::dot(&tmp1_, reference_vector);
    // CLAMP THE DOT PRODUCT BETWEEN -1 AND 1 OTHERWISE ACOSF RETURNS NAN!
    if (dot > 1) {
      dot = 1;
    }
    if (dot < -1) {
      dot = -1;
    }
    float angle = acosf(dot);  // 0 --> pi
    
    // But we also need 0 --> 2pi, so cross with the reference vector and 
    // check the magnitude of the dot product to the normal get the sign of the 
    // angle
    Float3::cross(&tmp2_, reference_vector, &tmp1_);
    if (Float3::dot(&tmp2_, normal) < 0) {
      angle = (2.0f*static_cast<float>(M_PI)) - angle;
    }
    // The above routines result in the angle going clockwise around the vertex
    // I want angle to be anti-clockwise positive so invert
    angle = (2.0f*static_cast<float>(M_PI)) - angle;
    return angle;
  }
  
  // pt1, pt2 and pt3 must be counter clockwise
  void MeshSimplification::calcNormalUnNormalized(Float3* normal, Float3* pt0, 
                                                  Float3* pt1, Float3* pt2) {
    tmp1_.sub(pt0, pt1);
    tmp2_.sub(pt2, pt1);
    normal->cross(&tmp1_, &tmp2_);
  }

  // pt1, pt2 and pt3 must be counter clockwise
  float MeshSimplification::calcNormalAndAngle(Float3* normal, Float3* pt0, 
                                               Float3* pt1, Float3* pt2) {
    tmp1_.sub(pt0, pt1);
    tmp2_.sub(pt2, pt1);
    normal->cross(&tmp1_, &tmp2_);
    normal->normalize();
    // While we have pt0 and pt1 calculated, calculate the angle between them
    // (which is used later)
    tmp1_.normalize();
    tmp2_.normalize();
    float dot = Float3::dot(&tmp1_, &tmp2_);
    dot = dot > 1 ? 1 : dot;  // clamp the dot product to avoid nan
    dot = dot < -1 ? -1 : dot;
    return acosf(dot);
  }

  // pt1, pt2 and pt3 must be counter clockwise
  void MeshSimplification::calcNormal(Float3* normal, Float3* pt0, 
                                      Float3* pt1, Float3* pt2) {
    tmp1_.sub(pt0, pt1);
    tmp2_.sub(pt2, pt1);
    normal->cross(&tmp1_, &tmp2_);
    normal->normalize();
  }

  // Some assumptions.
  // we - is empty
  // indices - contain only valid vertices in the vertex list
  // vertex_edges - has all NULL elements and is the size of the vertices
  // v123 - describes 3 indices which is ALWAYS anticlockwise with edges:
  //        v123[0]-->v123[1], v123[1]-->v123[2], v123[2]-->v123[0]
  void MeshSimplification::insertEdges(Vector<Edge>* we, 
                                       Vector<Float3>* normals,
                                       Vector<Float3>* vertices, 
                                       Vector<uint32_t>* indices, 
                                       uint32_t* v123) {
    Edge* edges[3];

    // First check for non-manifold toplogy.
    // Calculate the normal of this triangle and test it against the vertex
    // normals.  If any of them are > 90, then just throw the edges out.
    Float3 cur_normal;
    calcNormal(&cur_normal, vertices->at(v123[0]), 
               vertices->at(v123[1]), vertices->at(v123[2]));
    
    if (Float3::dot(&cur_normal, normals->at(v123[0])) < EPSILON || 
        Float3::dot(&cur_normal, normals->at(v123[1])) < EPSILON ||
        Float3::dot(&cur_normal, normals->at(v123[2])) < EPSILON) {
      return;
    } 
    
    // First find or create the 3 edges (don't worry about their pointers into
    // the rest of the data structure yet (ie E1a, E1b, E2a, E2b).
    for (uint32_t cur_pair = 0; cur_pair < 3; cur_pair++) {
      uint32_t v1 = v123[cur_pair];
      uint32_t v2 = v123[(cur_pair+1)%3];
      // Find the lowest of the two vertices: not strictly necessary but makes 
      // debug easier -> Gives a consistent access patterns when querying edges
      // attached to vertices
      uint32_t v_lowest = v1 <= v2 ? v1 : v2;  
      
      // First check if the edge exists already (potentially in the other 
      // direction). ie, we may have found a face that shares an edge.
      Edge* cur_edge = vertex_edges_[v_lowest];
      Edge* start_edge = cur_edge;
      
      // Check the first edge
      int edge_found = NO_EDGE_FOUND;
      if (cur_edge != NULL) {
        if ((cur_edge->v1 == v1 && cur_edge->v2 == v2)) {
          edge_found = FORWARD_EDGE_FOUND;
        } else if(cur_edge->v1 == v2 && cur_edge->v2 == v1) {
          edge_found = BACKWARD_EDGE_FOUND;
        } else {
          // Keep looking
          cur_edge = cur_edge->nextAnticlockwiseAround(v_lowest);
          while (cur_edge != start_edge) {
            if (cur_edge->v1 == v1 && cur_edge->v2 == v2) {
              edge_found = FORWARD_EDGE_FOUND;
              break;
            } else if(cur_edge->v1 == v2 && cur_edge->v2 == v1) {
              edge_found = BACKWARD_EDGE_FOUND;
              break;
            } else {
              // Keep looking
              cur_edge = cur_edge->nextAnticlockwiseAround(v_lowest);
            }
          }
        }
      }
      
      if (edge_found == BACKWARD_EDGE_FOUND) {
        edges[cur_pair] = cur_edge;
        cur_edge->face_b = true;
      } else if (edge_found == NO_EDGE_FOUND) {
        // Need to insert a new forward edge
        we->pushBack(Edge(v1, v2));
        edges[cur_pair] = we->at(we->size()-1);
      } else if (edge_found == FORWARD_EDGE_FOUND) {
        // Our triangle mesh shouldn't have an edge in the same direction
        // This implies non-manifold geometry --> but we should have avoided 
        // this already.
        throw wruntime_error(std::wstring(L"insertEdge() ERROR: found a") +
                                  std::wstring(L" forward edge!"));         
      }
    }
    
    // Next stich the edges into the data structure by cleaning up their Edge
    // pointers.
    // The big idea here is that we assume the mesh is locally manifold, in
    // this case we can project each edge onto the plane formed by v1 and it's
    // vertex normal (which has been precomputed) and v2 and it's vertex normal
    // and use this to order the vertices
       
    // Edge 1:
    stichEdge(edges[0], v123[0], vertices, normals);
    // Edge 2:
    stichEdge(edges[1], v123[1], vertices, normals);
    // Edge 3:
    stichEdge(edges[2], v123[2], vertices, normals);  
  }
  
  // edge -> curent edge to stich
  // ind_v1 -> first vertex from the perspective of the caller function (actual
  // edge may be stored as backwards).
  void MeshSimplification::stichEdge(Edge* edge, uint32_t ind_v1,
    Vector<Float3>* vertices, Vector<Float3>* normals) {
    if (edge->v1 == ind_v1) { // Forward edge --> Face A
      stichForwardEdge(edge, ind_v1, vertices, normals);
    } else {  // backward edge --> Face b
      // Nothing to do for backward edges: the edge has already been inserted
      // so the pointers wont be updated.
    }
  }
  
  
  // Steps around all the edges on the vertex until it finds the correct
  // insertion point for the new edge.  Note: edge angles on the vertex are
  // angles in the plane
  void MeshSimplification::findInsertionPoint(uint32_t vindex, float new_angle, 
    Edge** cur_edge, Edge** prev_edge) {
    Edge* start_edge = vertex_edges_[vindex];  
    *prev_edge = start_edge;
    (*cur_edge) = start_edge->nextAnticlockwiseAround(vindex);
    
    while ((*cur_edge) != start_edge) {
      if ((*cur_edge)->v1 == vindex) {
        if ((*cur_edge)->v1_angle > new_angle) {
          break;
        } else {
          (*prev_edge) = (*cur_edge);
          (*cur_edge) = (*cur_edge)->nextAnticlockwiseAround(vindex);
        }
      } else {
        if ((*cur_edge)->v2_angle > new_angle) {
          break;
        } else {
          (*prev_edge) = (*cur_edge);
          (*cur_edge) = (*cur_edge)->nextAnticlockwiseAround(vindex);
        }
      }
    }
  }
  
  void MeshSimplification::stichForwardEdge(Edge* edge, uint32_t ind_v1,
    Vector<Float3>* vertices, Vector<Float3>* normals) {
    Float3* vertex1 = vertices->at(edge->v1);
    Float3* vertex2 = vertices->at(edge->v2);
    
    // Note: A forward edge is potentially the first edge for some vertex,
    // so we may have to update the global vertex_edges_ array.  This can never
    // be the case for a backward edge.
    
    // Stitch the edge into V1's data structure
    if (vertex_edges_[edge->v1] == NULL) {
      // FIRST edge attached to this vertex
      vertex_edges_[edge->v1] = edge;
      edge->e1a = edge;  // Point back to my own triangle edge
      edge->e1b = edge;  
      edge->v1_angle = 0;  // My edge is the reference edge, so the angle is 0
      vertex_reference_vectors_[edge->v1].sub(vertex2, vertex1);
      vertex_reference_vectors_[edge->v1].normalize();
    } else {
      // Calculate the edge angle in the vertex plane
      edge->v1_angle = calcEdgeAngle(vertex1, vertex2,
                                     normals->at(edge->v1),
                                     vertex_reference_vectors_.at(edge->v1));
      // Now insert into the doubly linked list of edges, based on normal
      Edge* prev_edge;
      Edge* cur_edge;
      findInsertionPoint(edge->v1, edge->v1_angle, &cur_edge, &prev_edge);
     
      edge->e1a = cur_edge;  // anti-clockwise
      edge->e1b = prev_edge;  // clockwise
      
      // We need to insert the edge BEFORE cur_edge and AFTER prev_edge
      // cur_edge is anti-clockwise to edge input
      if (cur_edge->v1 == edge->v1) {
        cur_edge->e1b = edge;  // clockwise winding
      } else {
        cur_edge->e2a = edge;  // clockwise winding
      }
      if (prev_edge->v1 == edge->v1) {
        prev_edge->e1a = edge;  // anti-clockwise winding
      } else {
        prev_edge->e2b = edge;  // anti-clockwise winding
      }
    }
    
    // Stitch the edge into V2's data structure
    if (vertex_edges_[edge->v2] == NULL) {
      // FIRST edge attached to this vertex
      vertex_edges_[edge->v2] = edge;
      edge->e2a = edge;  // Point back to my own triangle edge
      edge->e2b = edge->e2a; 
      edge->v2_angle = 0;  // My edge is the reference edge, so the angle is 0
      vertex_reference_vectors_[edge->v2].sub(vertex1, vertex2);
      vertex_reference_vectors_[edge->v2].normalize();
    } else {
      // Calculate the edge angle in the vertex plane
      edge->v2_angle = calcEdgeAngle(vertex2, vertex1,
                                     normals->at(edge->v2),
                                     vertex_reference_vectors_.at(edge->v2));
      // Now insert into the doubly linked list of edges, based on normal
      Edge* prev_edge;
      Edge* cur_edge;
      findInsertionPoint(edge->v2, edge->v2_angle, &cur_edge, &prev_edge);
      
      // We need to insert the edge BEFORE cur_edge and AFTER prev_edge
      // cur_edge is anti-clockwise to edge input
      edge->e2b = cur_edge;  // anti-clockwise
      edge->e2a = prev_edge;  // clockwise
      if (cur_edge->v1 == edge->v2) {
        cur_edge->e1b = edge;  // clockwise winding
      } else {
        cur_edge->e2a = edge;  // clockwise winding
      }
      if (prev_edge->v1 == edge->v2) {
        prev_edge->e1a = edge;  // anti-clockwise winding
      } else {
        prev_edge->e2b = edge;  // anti-clockwise winding
      }      
    }    
  }
  
  void MeshSimplification::wingedEdgeToFaces(Vector<Edge>* we, 
    Vector<Float3>* normals, Vector<Float3>* vertices,
    Vector<uint32_t>* indices) {
    // We need to be able to mark if edges have been processed -> we can either
    // destroy the we connectivity (by setting face_a and face_b), or we can
    // reuse existing data in the edge structure.  Since it would be nice to
    // not destroy we --> I use heap_index to indicate that faces have been
    // processed.  It incurres a little overhead, but I think it's OK since
    // wingedEdgeToFaces is usually called on the reduced mesh.
    
    for (uint32_t i = 0; i < we->size(); i ++) {
      we->at(i)->heap_index = 0;  // indicates nothing has been processed
    }
    
    indices->resize(0);
    for (uint32_t i = 0; i < we->size(); i ++) {
      Edge* e0 = we->at(i);
      
      // Process face A
      if (e0->face_a && !(e0->heap_index & 1)) { 
        // forward edge face exists and it hasn't already been processed
        Edge* e1 = e0->e2a;
        Edge* e2 = e0->e1a;  // e0, e1 and e2 should be in anticlockwise order
        indices->pushBack(e0->v1);
        indices->pushBack(e0->v2);
        if (e1->v2 != e0->v2) {
          indices->pushBack(e1->v2);
        } else {
          indices->pushBack(e1->v1);
        }
        // Now mark those faces as not existing
        // For e1 and e2, we don't know which face we were counted as, so
        // we have to check
        if (e1->v1 == e0->v2) {
          e1->heap_index |= 1;  // face A has been procesed
        } else {
          e1->heap_index |= 2;  // face B has been procesed
        }
        if (e2->v2 == e0->v1) {
          e2->heap_index |= 1;  // face A has been procesed
        } else {
          e2->heap_index |= 2;  // face B has been procesed
        }
        e0->heap_index |= 1;  // face A has been procesed
      }
      // Process face B
      if (e0->face_b && !(e0->heap_index & 2)) {
        // backward edge face exists and it hasn't already been processed
        Edge* e1 = e0->e1b;
        Edge* e2 = e0->e2b;  // e0, e1 and e2 should be in anticlockwise order
        indices->pushBack(e0->v2);
        indices->pushBack(e0->v1);
        if (e1->v1 != e0->v1) {
          indices->pushBack(e1->v1);
        } else {
          indices->pushBack(e1->v2);
        }
        // Now mark those faces as not existing
        // For e1 and e2, we don't know which face we were counted as, so
        // we have to check
        if (e1->v1 == e0->v1) {
          e1->heap_index |= 1;  // face A has been procesed
        } else {
          e1->heap_index |= 2;  // face B has been procesed
        }
        if (e2->v1 == e0->v2) {
          e2->heap_index |= 2;  // face B has been procesed
        } else {
          e2->heap_index |= 1;  // face A has been procesed
        }    
        e0->heap_index |= 2;  // face B has been procesed
      }
    }
    
    calcNormalsPerVertex(normals, vertices, indices);
  }
  
  void MeshSimplification::cullEdges(Vector<Edge>* we, Vector<Float3>* normals,
    Vector<Float3>* vertices, Vector<Float3>* colors,  // can be NULL
    uint32_t edge_reduction) {
    // For each edge, calculate it's cost
    for (uint32_t i = 0; i < we->size(); i ++) {
      we->at(i)->calcCost(vertices, settings_.edge_cost_func);
    }
    
    // Create a min-heap of edges:
    MinHeapEdges heap(we);

    for (uint32_t i = 0; i < edge_reduction && heap.size() > 3; i++) {
      Edge* min_edge = heap.removeMin();
      if (min_edge->partOfAnEdgeTriangle()) {
#if defined(DEBUG) || defined(_DEBUG)
        std::cout << "Cannot simplify mesh further.  Trying to remove a ";
        std::cout << "contour edge" << std::endl;
#endif
        return;
      }

      // Now check for any edges which would cause the edge collapse to create
      // duplicate edges, and disallow those collapses.
      Edge* cur_edge = min_edge->e1a;
      uint32_t v1 = min_edge->v1;
      uint32_t v2 = min_edge->v2;
      uint32_t v3;
      bool contraction_valid = true;
      do {
        if (cur_edge != min_edge->e1a && 
            cur_edge != min_edge->e1b) {
          Edge* search_edge = cur_edge;
          v3 = (search_edge->v1 == v1) ? search_edge->v2 : search_edge->v1;
          do {
            if (search_edge->v1 == v2 || search_edge->v2 == v2) {
              // Disallow this contraction:
              contraction_valid = false;
            }
            search_edge = search_edge->nextAnticlockwiseAround(v3);
          } while (search_edge != cur_edge && contraction_valid);
        }
        cur_edge = cur_edge->nextAnticlockwiseAround(v1);
      } while (cur_edge != min_edge && contraction_valid);

      if (!contraction_valid) {
        // Removing this edge would have caused the mesh to produce redundant
        // faces, increase the cost and try again later
        if (min_edge->num_reinsertions < MAX_NUM_EDGE_REINSERTIONS) {
          min_edge->num_reinsertions++;
          min_edge->cost *= EDGE_REINSERTION_PENALTY;
          heap.insert(min_edge);
        }
        continue;
      }

      // Finally, we can remove the current vertex
      EdgeRemovalResult res = removeEdge(min_edge, normals, vertices, colors, 
        &heap);
      if (res == EdgeRemovalDisallowed) {
        // Removing this edge would have caused the mesh to be non-manifold
        // increase the cost by a large penalty and try again later
        if (min_edge->num_reinsertions < MAX_NUM_EDGE_REINSERTIONS) {
          min_edge->num_reinsertions++;
          min_edge->cost *= EDGE_REINSERTION_PENALTY;
          heap.insert(min_edge);
        }
      }
      if (res == EdgeRemovalError) {
        throw wruntime_error(wstring(L"simplifyMesh() - ERROR: Edge removal ") +
          wstring(L"failed!"));
        return;
      }

#ifdef DEBUG_MODE_VALIDATE_HEAP
      if (!heap.validate()) {
        printf("ERROR: The heap is corrupt!");
        printf("exiting on edge iteration %d\n", i);
        return;
      }
#endif

#ifdef DEBUG_MODE_VALIDATE_WE_STRUCTURE
      if (!validateWEStructure(we)) {
        printf("exiting on edge iteration %d\n", i);
        return;
      }
#endif
    }  // end for
  }

  EdgeRemovalResult MeshSimplification::removeEdge(Edge* edge_to_remove, 
    Vector<Float3>* normals, Vector<Float3>* vertices, Vector<Float3>* colors, 
    MinHeapEdges* heap) {
    Float3 edge_vec;

    uint32_t v1 = edge_to_remove->v1;
    uint32_t v2 = edge_to_remove->v2;

    uint32_t num_edges_on_v2 = edge_to_remove->numEdgesOnVertex(v2);

    Float3 new_vertex_pos;

    if (settings_.vertex_merge_method == MidpointVertexMerge) {
      new_vertex_pos.add(vertices->at(v2), vertices->at(v1));
      new_vertex_pos.scale(0.5f);
    } else {
      new_vertex_pos.set(vertices->at(v1));
    }

    // TEMP CODE
    // edge_to_remove->printEdgesAnticlockwiseAround(v1);
    // edge_to_remove->printEdgesAnticlockwiseAround(v2);
    // END TEMP CODE

    // check if removing the edge will create a non-manifold triangle
    // do this by checking if the normal would flip after the operation.
    if (settings_.edge_constraint == EdgeRemovalConstrainNonManifold) {
      // Check the triangle fin case (no surviving edge on v2)
      Float3 new_normal;
      uint32_t v2_new;
      uint32_t v3_new;
      Edge* cur_edge;
      Edge* stop_edge;
      if (num_edges_on_v2 == 3) {
        v2_new = edge_to_remove->e1b->v1 == v1 ? edge_to_remove->e1b->v2 : edge_to_remove->e1b->v1;
        v3_new = edge_to_remove->e1a->v1 == v1 ? edge_to_remove->e1a->v2 : edge_to_remove->e1a->v1;
        calcNormal(&new_normal, &new_vertex_pos, vertices->at(v2_new), vertices->at(v3_new));
        // Check against v2 normal
        if (Float3::dot(&new_normal, normals->at(v2_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
          return EdgeRemovalDisallowed;
        }
        // Check against v3 normal
        if (Float3::dot(&new_normal, normals->at(v3_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
          return EdgeRemovalDisallowed;
        }
      } else {
        // First check the special cases of the edges that are being removed.
        // E2A:
        stop_edge = edge_to_remove->nextClockwiseAround(v2);
        v3_new = (stop_edge->v1 == v2) ? stop_edge->v2 : stop_edge->v1;
        stop_edge = stop_edge->nextClockwiseAround(v2);
        v2_new = (stop_edge->v1 == v2) ? stop_edge->v2 : stop_edge->v1;
        calcNormal(&new_normal, &new_vertex_pos, vertices->at(v2_new), vertices->at(v3_new));
        // Check against v2 normal
        if (Float3::dot(&new_normal, normals->at(v2_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
          return EdgeRemovalDisallowed;
        }
        // Check against v3 normal
        if (Float3::dot(&new_normal, normals->at(v3_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
          return EdgeRemovalDisallowed;
        }
        // E2B:
        cur_edge = edge_to_remove->nextAnticlockwiseAround(v2);
        v2_new = (cur_edge->v1 == v2) ? cur_edge->v2 : cur_edge->v1;
        cur_edge = cur_edge->nextAnticlockwiseAround(v2);
        v3_new = (cur_edge->v1 == v2) ? cur_edge->v2 : cur_edge->v1;
        calcNormal(&new_normal, &new_vertex_pos, vertices->at(v2_new), vertices->at(v3_new));
        // Check against v2 normal
        if (Float3::dot(&new_normal, normals->at(v2_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
          return EdgeRemovalDisallowed;
        }
        // Check against v3 normal
        if (Float3::dot(&new_normal, normals->at(v3_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
          return EdgeRemovalDisallowed;
        }
        // Now check all other triangles that will change on v2
        while (cur_edge != stop_edge) {
          // Check the face anti-clockwise around v2
          if (cur_edge->v1 == v2) {  // forward edge around v2
            v2_new = cur_edge->v2;
            v3_new = (cur_edge->e1a->v1 == v2) ? cur_edge->e1a->v2 : cur_edge->e1a->v1;
            calcNormal(&new_normal, &new_vertex_pos, vertices->at(v2_new), vertices->at(v3_new));
            // Check against v2 normal
            if (Float3::dot(&new_normal, normals->at(v2_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
              return EdgeRemovalDisallowed;
            }
            // Check against v3 normal
            if (Float3::dot(&new_normal, normals->at(v3_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
              return EdgeRemovalDisallowed;
            }
          } else {
            v2_new = cur_edge->v1;
            v3_new = (cur_edge->e2b->v1 == v2) ? cur_edge->e2b->v2 : cur_edge->e2b->v1;
            calcNormal(&new_normal, &new_vertex_pos, vertices->at(v2_new), vertices->at(v3_new));
            // Check against v2 normal
            if (Float3::dot(&new_normal, normals->at(v2_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
              return EdgeRemovalDisallowed;
            }
            // Check against v3 normal
            if (Float3::dot(&new_normal, normals->at(v3_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
              return EdgeRemovalDisallowed;
            }
          }
          cur_edge = cur_edge->nextAnticlockwiseAround(v2);
        }
      }

      // Now check triangles that will change on v1
      cur_edge = edge_to_remove->e1a;
      stop_edge = edge_to_remove->e1b;
      while (cur_edge != stop_edge) {
        // Check the face clockwise around v1
        if (cur_edge->v1 == v1) {  // forward edge around v2
          v2_new = cur_edge->v2;
          v3_new = (cur_edge->e1a->v1 == v1) ? cur_edge->e1a->v2 : cur_edge->e1a->v1;
          calcNormal(&new_normal, &new_vertex_pos, vertices->at(v2_new), vertices->at(v3_new));
          // Check against v2 normal
          if (Float3::dot(&new_normal, normals->at(v2_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
            return EdgeRemovalDisallowed;
          }
          // Check against v3 normal
          if (Float3::dot(&new_normal, normals->at(v3_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
            return EdgeRemovalDisallowed;
          }
        } else {
          v2_new = cur_edge->v1;
          v3_new = (cur_edge->e2b->v1 == v1) ? cur_edge->e2b->v2 : cur_edge->e2b->v1;
          calcNormal(&new_normal, &new_vertex_pos, vertices->at(v2_new), vertices->at(v3_new));
          // Check against v2 normal
          if (Float3::dot(&new_normal, normals->at(v2_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
            return EdgeRemovalDisallowed;
          }
          // Check against v3 normal
          if (Float3::dot(&new_normal, normals->at(v3_new)) < MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD) {
            return EdgeRemovalDisallowed;
          }
        }
        cur_edge = cur_edge->nextAnticlockwiseAround(v1);
      }
      // Otherwise, if we got to here then all edge removals are legal
    }

    // Move V1 to the correct position
    if (settings_.edge_constraint == EdgeRemovalConstrainNonManifold) {
      vertices->at(v1)->set(&new_vertex_pos);
      // Next make the color the average of the two 
      if (colors != NULL) {
        colors->at(v1)->add(colors->at(v1), 
          colors->at(v2));
        colors->at(v1)->scale(0.5f);
      }
    }

    // Smash all old vertices together so that errors are very obvious
    vertices->at(v2)->set(vertices->at(v2)->m[0], vertices->at(v2)->m[1], 
      REMOVED_VERTEX_POSITION_Z);  

    // recall: e1a is the anti-clockwise edge off v1
    //         e1b is the clockwise edge off v1
    //         e2a is the clockwise edge off v2
    //         e2b is the anti-clockwise edge off v2
    Edge* edges_to_remove[2];
    edges_to_remove[0] = edge_to_remove->e2a;
    edges_to_remove[1] = edge_to_remove->e2b;
    Edge* replacement_clockwise;
    Edge* replacement_anti_clockwise;
    uint32_t vertex_to_revolve_around;

    // Step around e2a's vertex that isn't shared with this edge and change 
    // all references of e2a pointer to e1a
    replacement_clockwise = edge_to_remove->e1a;
    if (edges_to_remove[0]->v1 == v2) {
      vertex_to_revolve_around = edges_to_remove[0]->v2;
      replacement_anti_clockwise = edges_to_remove[0]->e2b;
    } else {
      vertex_to_revolve_around = edges_to_remove[0]->v1;
      replacement_anti_clockwise = edges_to_remove[0]->e1a;
    }
    patchVertex(edges_to_remove[0],
      replacement_anti_clockwise,
      replacement_clockwise,
      vertex_to_revolve_around);

    if (num_edges_on_v2 > 3) {
      // Step around v2 and remove e2a
      edges_to_remove[0] = edge_to_remove->e2a;
      replacement_anti_clockwise = edge_to_remove;
      vertex_to_revolve_around = v2;
      if (edges_to_remove[0]->v1 == v2) {
        replacement_clockwise = edges_to_remove[0]->e1b;
      } else {
        replacement_clockwise = edges_to_remove[0]->e2a;
      }
      patchVertex(edges_to_remove[0],
        replacement_anti_clockwise,
        replacement_clockwise,
        vertex_to_revolve_around);
    }

    edges_to_remove[0]->face_a = false;
    edges_to_remove[0]->face_b = false;
    heap->remove(edges_to_remove[0]->heap_index);    

    // Step around e2b's vertex that isn't shared with this edge and change 
    // all references of e2b pointer to e1b
    replacement_anti_clockwise = edge_to_remove->e1b;
    if (edges_to_remove[1]->v1 == v2) {
      vertex_to_revolve_around = edges_to_remove[1]->v2;
      replacement_clockwise = edges_to_remove[1]->e2a;
    } else {
      vertex_to_revolve_around = edges_to_remove[1]->v1;
      replacement_clockwise = edges_to_remove[1]->e1b;
    }
    patchVertex(edges_to_remove[1],
      replacement_anti_clockwise,
      replacement_clockwise,
      vertex_to_revolve_around);

    if (num_edges_on_v2 > 3) {
      // Step around v2 and remove e2b
      edges_to_remove[1] = edge_to_remove->e2b;
      replacement_clockwise = edge_to_remove;
      vertex_to_revolve_around = v2;
      if (edges_to_remove[1]->v1 == v2) {
        replacement_anti_clockwise = edges_to_remove[1]->e1a;
      } else {
        replacement_anti_clockwise = edges_to_remove[1]->e2b;
      }
      patchVertex(edges_to_remove[1],
        replacement_anti_clockwise,
        replacement_clockwise,
        vertex_to_revolve_around);
    }

    edges_to_remove[1]->face_a = false;
    edges_to_remove[1]->face_b = false;
    heap->remove(edges_to_remove[1]->heap_index);

    if (num_edges_on_v2 > 3) {
      vertex_to_revolve_around = v2;
      replacement_clockwise = edge_to_remove->e1b;
      replacement_anti_clockwise = edge_to_remove->e1a;
      patchVertex(edge_to_remove,
        replacement_anti_clockwise,
        replacement_clockwise,
        vertex_to_revolve_around);

      vertex_to_revolve_around = v1;
      replacement_clockwise = edge_to_remove->e2a;
      replacement_anti_clockwise = edge_to_remove->e2b;
      patchVertex(edge_to_remove,
        replacement_anti_clockwise,
        replacement_clockwise,
        vertex_to_revolve_around);
    } else {
      vertex_to_revolve_around = v1;
      replacement_clockwise = edge_to_remove->e1b;
      replacement_anti_clockwise = edge_to_remove->e1a;
      patchVertex(edge_to_remove,
        replacement_anti_clockwise,
        replacement_clockwise,
        vertex_to_revolve_around);        
    }

    // Now invalidate the actual edge
    edge_to_remove->face_a = false;
    edge_to_remove->face_b = false;
    vertex_edges_[v2] = NULL;

    if (num_edges_on_v2 > 3) {
      // Now step around the new vertex updating all attached edges (since the
      // vertex moved)
      Edge* start_edge = edge_to_remove->e1b;
      Edge* cur_edge = start_edge;
      do {
        if (cur_edge->v1 == v2) {
          cur_edge->v1 = v1;
        } else if (cur_edge->v2 == v2) {
          cur_edge->v2 = v1;
        }
        cur_edge = cur_edge->nextAnticlockwiseAround(v1);
      } while (cur_edge != start_edge);
    }

    // Update the edge costs on the newly changed vertices
    updateEdgeCosts(edge_to_remove->e1a, v1, vertices, normals, heap);

    // We also need to keep track of valid vertex normals (so we can test for
    // non-manifold geometry)
    if (settings_.edge_constraint == EdgeRemovalConstrainNonManifold) {
      // Update on V1
      updateVertexNormal(normals, vertices, edge_to_remove->e1a, v1);
      // Update on all vertices attached to v1
      Edge* start_edge = edge_to_remove->e1a;
      Edge* cur_edge = start_edge;
      do {
        if (cur_edge->v1 == v1) {
          updateVertexNormal(normals, vertices, cur_edge, cur_edge->v2);
        } else {
          updateVertexNormal(normals, vertices, cur_edge, cur_edge->v1);
        }
      } while (cur_edge != start_edge);
    }

    return EdgeRemovalOK;
  }
  
  void MeshSimplification::patchVertex(Edge* edge_to_remove, 
                                       Edge* edge_to_replace_anti_clockwise, 
                                       Edge* edge_to_replace_clockwise,
                                       uint32_t vertex_to_patch) {
    if (vertex_edges_[vertex_to_patch] == edge_to_remove) {
      vertex_edges_[vertex_to_patch] = edge_to_replace_anti_clockwise;
    }

    if (edge_to_remove->v1 == vertex_to_patch) {  
      // Fix up the connection for e1b --> e1a
      if (edge_to_remove->e1b->v1 == vertex_to_patch) {
          edge_to_remove->e1b->e1a = edge_to_replace_anti_clockwise;
      } else {
          edge_to_remove->e1b->e2b = edge_to_replace_anti_clockwise;
      }
      
      // Fix up the connection for e1a --> e1b
      // exists
      if (edge_to_remove->e1a->v1 == vertex_to_patch) {
          edge_to_remove->e1a->e1b = edge_to_replace_clockwise;
      } else {
          edge_to_remove->e1a->e2a = edge_to_replace_clockwise;
      }
      
    } else {
      // Fix up the connection for e2a --> e2b
      // exists
      if (edge_to_remove->e2a->v1 == vertex_to_patch) {
          edge_to_remove->e2a->e1a = edge_to_replace_anti_clockwise;
      } else {
          edge_to_remove->e2a->e2b = edge_to_replace_anti_clockwise;
      }
      
      // Fix up the connection for e2b --> e2a
      if (edge_to_remove->e2b->v1 == vertex_to_patch) {
          edge_to_remove->e2b->e1b = edge_to_replace_clockwise;
      } else {
        edge_to_remove->e2b->e2a = edge_to_replace_clockwise;
      }
    }
  }
  
  // Update the edges in the current triangle
  void MeshSimplification::updateEdgeCostTri(Edge* edge, 
    uint32_t vertex_to_patch, Vector<Float3>* vertices, Vector<Float3>* normals,
    MinHeapEdges* heap) {
     edge->calcCost(vertices, settings_.edge_cost_func);
      heap->fixHeap(edge->heap_index);

    // Update edges around the anti-clockwise face
    if (edge->v1 == vertex_to_patch && edge->face_a) {
      edge->e1a->calcCost(vertices, settings_.edge_cost_func);
      heap->fixHeap(edge->e1a->heap_index);

      edge->e2a->calcCost(vertices, settings_.edge_cost_func);
      heap->fixHeap(edge->e2a->heap_index);
    } else if (edge->v1 != vertex_to_patch && edge->face_b) {
      edge->e1b->calcCost(vertices, settings_.edge_cost_func);
      heap->fixHeap(edge->e1b->heap_index);
      
      edge->e2b->calcCost(vertices, settings_.edge_cost_func);
      heap->fixHeap(edge->e2b->heap_index);
    }
  }
  
  void MeshSimplification::updateEdgeCosts(Edge* start_edge, 
                                           uint32_t vertex_to_patch,
                                           Vector<Float3>* vertices,
                                           Vector<Float3>* normals,
                                           MinHeapEdges* heap) {
    if (settings_.edge_constraint == EdgeRemovalConstrainNonManifold) {
      // If the triangle area is the cost, then we need to update ALL edges of
      // the affected triangles
      Edge* cur_edge = start_edge;
      do {
        updateEdgeCostTri(start_edge, vertex_to_patch, vertices, normals, heap);
        cur_edge = cur_edge->nextAnticlockwiseAround(vertex_to_patch);
      } while (cur_edge != start_edge);
    } else {
      // Otherwise, just modify the cost of the edges attached to the modified
      // vertex if the cost is just edge length
      start_edge->calcCost(vertices, settings_.edge_cost_func);
      heap->fixHeap(start_edge->heap_index);
      Edge* cur_edge = start_edge->nextAnticlockwiseAround(vertex_to_patch);
      while (cur_edge != start_edge) {
        cur_edge->calcCost(vertices, settings_.edge_cost_func);
        heap->fixHeap(cur_edge->heap_index);
        cur_edge = cur_edge->nextAnticlockwiseAround(vertex_to_patch);
      }
    }
  }

  bool MeshSimplification::validateWEStructure(data_str::Vector<Edge>* we) {
    uint32_t we_size = we->size();
    for (uint32_t i = 0; i < we_size; i++) {
      Edge* cur_edge = we->at(i);
      if (!cur_edge->face_a && !cur_edge->face_b) {
        continue;  // Face has been removed, skip it
      }

      if (vertex_edges_[cur_edge->v1] == NULL || 
          vertex_edges_[cur_edge->v2] == NULL) {
        printf("ERROR: an edge is pointing towards a removed vertex\n");
        return false;
      }

      uint32_t num_iterations = 0;

      uint32_t v1 = cur_edge->v1;
      uint32_t v2 = cur_edge->v2;

      // Check for duplicates around v1
      Edge* scan_edge = vertex_edges_[v1];
      Edge* start_edge = scan_edge;
      do {
        if (scan_edge != cur_edge) {  // skip over the current edge
          if ((scan_edge->v1 == v1 && scan_edge->v2 == v2) ||
              (scan_edge->v2 == v1 && scan_edge->v1 == v2)) {
            printf("ERROR: Duplicate edge found [%d, %d]\n", v1, v2);
            return false;
          }
        }
        scan_edge = scan_edge->nextAnticlockwiseAround(v1);
        num_iterations++;

        if (num_iterations > 10000) {
          printf("ERROR: infinite loop around a vertex found\n");
          return false;
        }
      } while (scan_edge != start_edge);

      num_iterations = 0;

      // Check for duplicates around v2
      scan_edge = vertex_edges_[v2];
      start_edge = scan_edge;
      do {
        if (scan_edge != cur_edge) {  // skip over the current edge
          if ((scan_edge->v1 == v1 && scan_edge->v2 == v2) ||
              (scan_edge->v2 == v1 && scan_edge->v1 == v2)) {
            printf("ERROR: Duplicate edge found [%d, %d]\n", v1, v2);
            return false;
          }
        }
        scan_edge = scan_edge->nextAnticlockwiseAround(v2);
        num_iterations++;

        if (num_iterations > 10000) {
          printf("ERROR: infinite loop around a vertex found\n");
          return false;
        }
      } while (scan_edge != start_edge);
    }
    return true;
  }
  
}  // namespace mesh_simplification
