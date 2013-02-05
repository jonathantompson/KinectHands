//
//  mesh_simplification.h
//  Created by Jonathan Tompson on 6/21/12.
//
//  Takes as input a vertex, normal, color (optional) and index array.
//  It will modify these buffers, creating a simplified version of the 
//  mesh.
//
//  LIMITATIONS: The input mesh must be locally manifold.  This is most meshes
//  that occur in practice.  If it is not, it will still return a simplified
//  mesh, but the returned mesh may contain holes.
//
//  Top level algorithm:
//   - Create a winged-edge mesh representation of the input geometry.
//   - Construct a min-heap of input edges sorted by their removal cost.
//   - For N edges, remove the smallest cost edge and patch the data structure
//   - Finally re-triangulate the mesh after simplification.
//
//  TO DO 1: Edge removals that will cause degenerate triangles (other than e1a
//  and e1b) are disallowed, in reality we can actually perform the removal and
//  then remove the edges.  Fix this.
//
//  TO DO 2: Edge removals may cause non-manifold meshes, fix this --> From
//  "Surface Simplification Using Quadric Error Metrics" (Garland, Heckbert):
//
//  Preventing Mesh Inversion: Pair contractions do not necessarily
//  preserve the orientation of the faces in the area of the contraction.
//  For instance, it is possible to contract an edge and cause some
//  neighboring faces to fold over on each other. It is usually best to try
//  to avoid this type of mesh inversion. We use essentially the same
//  scheme as others have before ([7] for example). When considering
//  a possible contraction, we compare the normal of each neighboring
//  face before and after the contraction. If the normal flips, that contraction
//  can be either heavily penalized or disallowed.

#ifndef MESH_SIMPLIFICATION_MESH_SIMPLIFICATION_HEADER
#define MESH_SIMPLIFICATION_MESH_SIMPLIFICATION_HEADER

#define MESH_SIMPLIFICATION_CONTOUR_PENALTY std::numeric_limits<float>::infinity()
#define REMOVED_VERTEX_POSITION_Z std::numeric_limits<float>::infinity()
#define EDGE_REINSERTION_PENALTY 10000

#define MAX_NUM_EDGE_REINSERTIONS 3
#define MAX_EDGE_REMOVAL_NORMAL_CHANGE_DOT_PROD 0.1f

#include "mesh_simplification/edge.h"
#include "data_str/vector.h"
#include "math/math_types.h"

// If defined: On every iteration, validate the heap property --> SLOW:
// #define DEBUG_MODE_VALIDATE_HEAP

// If defined: On every iteration, validate the entire WE structure --> SLOW
// #define DEBUG_MODE_VALIDATE_WE_STRUCTURE

namespace mesh_simplification {
  class MinHeapEdges;

  typedef enum {
    SimpleNormalApproximation,  // average normals around vert weighted by area
    RobustNormalApproximation,  // average normals weighted by angle at the vert
  } NormalApproximationMethod;

  typedef enum {
    MidpointVertexMerge,  //  calculate average position and color
    StaticVertexMerge,  // No vertex modification
  } VertexMergeOperation;

  typedef enum {
    EdgeRemovalError,
    EdgeRemovalDisallowed,
    EdgeRemovalOK,
  } EdgeRemovalResult;

  enum {
    NO_EDGE_FOUND = 0,
    FORWARD_EDGE_FOUND = 1,
    BACKWARD_EDGE_FOUND = 2
  };

  struct MeshSettings {
  public:
    MeshSettings();    
    NormalApproximationMethod normal_method;
    VertexMergeOperation vertex_merge_method;
    EdgeCostFunction edge_cost_func;
    EdgeRemovalConstraint edge_constraint;

    MeshSettings& operator=(const MeshSettings& other);
  };

  class MeshSimplification {
  public:
    MeshSimplification(const MeshSettings& settings);
    MeshSimplification() { }

    // Top level external function.  See description in header for details.
    void simplifyMesh(uint32_t edge_reduction,
      data_str::Vector<math::Float3>* vertices, 
      data_str::Vector<uint32_t>* indices, 
      data_str::Vector<math::Float3>* normals_return,
      data_str::Vector<math::Float3>* colors = NULL);

    data_str::Vector<Edge>* getWEStructure() { return &we_; }
    void calcNormalsPerVertex(data_str::Vector<math::Float3>* normals_return,
                              data_str::Vector<math::Float3>* vertices, 
                              data_str::Vector<uint32_t>* indices);

  private:
    // TOP LEVEL INTERNAL FUNCTIONS
    void facesToWingedEdge(data_str::Vector<Edge>* we_return,
      data_str::Vector<math::Float3>* normals,
      data_str::Vector<math::Float3>* vertices, 
      data_str::Vector<uint32_t>* indices);
    void cullEdges(data_str::Vector<Edge>* we, 
      data_str::Vector<math::Float3>* normals, 
      data_str::Vector<math::Float3>* vertices,
      data_str::Vector<math::Float3>* colors, uint32_t edge_reduction);
    void wingedEdgeToFaces(data_str::Vector<Edge>* we,
      data_str::Vector<math::Float3>* normals,
      data_str::Vector<math::Float3>* vertices,
      data_str::Vector<uint32_t>* indices);    

    // HELPER FUNCTIONS
    void insertEdges(data_str::Vector<Edge>* we, 
      data_str::Vector<math::Float3>* normals,
      data_str::Vector<math::Float3>* vertices,  
      data_str::Vector<uint32_t>* indices, uint32_t* v123); 
    void calcNormalUnNormalized(math::Float3* normal, math::Float3* pt0, 
      math::Float3* pt1, math::Float3* pt2);
    float calcNormalAndAngle(math::Float3* normal, math::Float3* pt0, 
      math::Float3* pt1, math::Float3* pt2);
    void calcNormal(math::Float3* normal, math::Float3* pt0, 
      math::Float3* pt1, math::Float3* pt2);
    float calcEdgeAngle(math::Float3* vertex1, math::Float3* vertex2, 
      math::Float3* normal, math::Float3* reference_vector);
    void stichEdge(Edge* edge, uint32_t ind_v1,
      data_str::Vector<math::Float3>* vertices,
      data_str::Vector<math::Float3>* normals);
    void stichForwardEdge(Edge* edge, uint32_t ind_v1,
      data_str::Vector<math::Float3>* vertices,
      data_str::Vector<math::Float3>* normals);   
    void findInsertionPoint(uint32_t vindex, float angle, 
      Edge** cur_edge, Edge** prev_edge);
    void patchVertex(Edge* edge_to_remove, Edge* edge_to_replace_anti_clockwise,
      Edge* edge_to_replace_clockwise, uint32_t vertex_to_patch);
    void updateEdgeCosts(Edge* start_edge, uint32_t vertex_to_patch,
      data_str::Vector<math::Float3>* vertices,
      data_str::Vector<math::Float3>* normals, MinHeapEdges* heap);
    EdgeRemovalResult removeEdge(Edge* edge_to_remove, 
      data_str::Vector<math::Float3>* normals,
      data_str::Vector<math::Float3>* vertices,
      data_str::Vector<math::Float3>* colors, MinHeapEdges* heap);
    float calcAngleSafe(math::Float3* pt0, math::Float3* pt1, 
      math::Float3* pt2);
    void updateEdgeCostTri(Edge* edge, uint32_t vertex_to_patch, 
      data_str::Vector<math::Float3>* vertices, 
      data_str::Vector<math::Float3>* normals, MinHeapEdges* heap);
    void updateVertexNormal(data_str::Vector<math::Float3>* normals,
      data_str::Vector<math::Float3>* vertices, Edge* edge, 
      uint32_t vertex_to_patch);

    bool validateWEStructure(data_str::Vector<Edge>* we);

    // Some temporary structures for building the data
    data_str::Vector<Edge> we_;
    data_str::Vector<Edge*> vertex_edges_;
    data_str::Vector<math::Float3> vertex_reference_vectors_;
    math::Float3 tmp1_;
    math::Float3 tmp2_;
    MeshSettings settings_;
  };
};  // namespace mesh_simplification

#endif  // MESH_SIMPLIFICATION_MESH_SIMPLIFICATION_HEADER
