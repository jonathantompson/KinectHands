//
//  hand_model_geometry.h
//
//  Created by Jonathan Tompson on 10/26/12.
//
//  Base container class for all the hand geometry

#ifndef HAND_MODEL_HAND_MODEL_GEOMETRY_HEADER
#define HAND_MODEL_HAND_MODEL_GEOMETRY_HEADER

#include "kinect_interface/hand_net/hand_model.h"
#include "Eigen"

#define NSPH_PER_FING 6
// #define USE_SIMPLE_GEOMETRY  // Otherwise use dense mesh
#if defined(__APPLE__)
  #define HAND_MODEL_PATH string("./../../../../../../../../../models/")
#else
  #define HAND_MODEL_PATH string("./models/")
#endif

#define LOAD_HAND_MESH_JFILE  // Much faster and more compact format!
//#define LHAND_MODEL_FILE "hand.dae"
//#define LHAND_MODEL_JFILE "hand.jbin"
//#define LHAND_MODEL_FILE "hand_short_wrist.dae"
//#define LHAND_MODEL_JFILE "hand_short_wrist.jbin"
#define LHAND_MODEL_FILE "hand_medium_wrist.dae"
#define LHAND_MODEL_JFILE "hand_medium_wrist.jbin"
//#define LHAND_MODEL_FILE "hand_vertex_colors.dae"
//#define LHAND_MODEL_JFILE "hand_vertex_colors.jbin"

//#define RHAND_MODEL_FILE "hand_right.dae"
//#define RHAND_MODEL_JFILE "hand_right.jbin"
//#define RHAND_MODEL_FILE "hand_short_wrist_right.dae"
//#define RHAND_MODEL_JFILE "hand_short_wrist_right.jbin"
#define RHAND_MODEL_FILE "hand_medium_wrist_right.dae"
#define RHAND_MODEL_JFILE "hand_medium_wrist_right.jbin"
//#define RHAND_MODEL_FILE "hand_vertex_colors_right.dae"
//#define RHAND_MODEL_JFILE "hand_vertex_colors_right.jbin"

namespace renderer { class Geometry; }

namespace hand_model {

  typedef enum {
    F1_KNU3_A = 0,
    F1_KNU3_B = 1,
    F1_KNU2_A = 2,
    F1_KNU2_B = 3,
    F1_KNU1_A = 4,
    F1_KNU1_B = 5,
    F2_KNU3_A = 6,
    F2_KNU3_B = 7,
    F2_KNU2_A = 8,
    F2_KNU2_B = 9,
    F2_KNU1_A = 10,
    F2_KNU1_B = 11,
    F3_KNU3_A = 12,
    F3_KNU3_B = 13,
    F3_KNU2_A = 14,
    F3_KNU2_B = 15,
    F3_KNU1_A = 16,
    F3_KNU1_B = 17,
    F4_KNU3_A = 18,
    F4_KNU3_B = 19,
    F4_KNU2_A = 20,
    F4_KNU2_B = 21,
    F4_KNU1_A = 22,
    F4_KNU1_B = 23,
    TH_KNU3_A = 24,
    TH_KNU3_B = 25,
    TH_KNU2_A = 26,
    TH_KNU2_B = 27,
    TH_KNU1_A = 28,
    TH_KNU1_B = 29,
    NUM_BOUNDING_SPHERES = 30
  } HandSphereIndices;

  class HandGeometry {
  public:
    // Constructor / Destructor
    HandGeometry(kinect_interface::hand_net::HandType hand_type) : hand_type_(hand_type) { }
    virtual ~HandGeometry() { }

    // Call before rendering hand depth maps:
    virtual void updateMatrices(const float* coeff) { }
    virtual void updateHeirachyMatrices() { }
    virtual void fixBoundingSphereMatrices() { }
    virtual inline renderer::Geometry* scene_graph() { return NULL; }

    virtual void renderStackReset() { }
    virtual renderer::Geometry* renderStackPop() { return NULL; }
    virtual bool renderStackEmpty() { return true; }

  protected:
    kinect_interface::hand_net::HandType hand_type_;

    // Non-copyable, non-assignable.
    HandGeometry(HandGeometry&);
    HandGeometry& operator=(const HandGeometry&);
  };
};  // namespace hand_model

#endif  // HAND_MODEL_HAND_MODEL_RENDERER_GEOMETRY_HEADER
