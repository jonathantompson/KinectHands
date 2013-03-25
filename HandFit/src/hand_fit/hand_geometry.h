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
//#define LHAND_MODEL_FILE "hand_palm_parent.dae"
//#define LHAND_MODEL_JFILE "hand_palm_parent.jbin"
//#define LHAND_MODEL_FILE "hand_palm_parent_short_wrist.dae"
//#define LHAND_MODEL_JFILE "hand_palm_parent_short_wrist.jbin"
#define LHAND_MODEL_FILE "hand_palm_parent_medium_wrist.dae"
#define LHAND_MODEL_JFILE "hand_palm_parent_medium_wrist.jbin"
//#define LHAND_MODEL_FILE "hand_palm_parent_vertex_colors.dae"
//#define LHAND_MODEL_JFILE "hand_palm_parent_vertex_colors.jbin"

//#define RHAND_MODEL_FILE "hand_right.dae"
//#define RHAND_MODEL_JFILE "hand_right.jbin"
//#define RHAND_MODEL_FILE "hand_palm_parent_right.dae"
//#define RHAND_MODEL_JFILE "hand_palm_parent_right.jbin"
//#define RHAND_MODEL_FILE "hand_palm_parent_short_wrist_right.dae"
//#define RHAND_MODEL_JFILE "hand_palm_parent_short_wrist_right.jbin"
#define RHAND_MODEL_FILE "hand_palm_parent_medium_wrist_right.dae"
#define RHAND_MODEL_JFILE "hand_palm_parent_medium_wrist_right.jbin"
//#define RHAND_MODEL_FILE "hand_palm_parent_vertex_colors_right.dae"
//#define RHAND_MODEL_JFILE "hand_palm_parent_vertex_colors_right.jbin"

#define NUM_SPHERES kinect_interface::hand_net::HandSphereIndices::NUM_BOUNDING_SPHERES

namespace renderer { class Geometry; }

namespace hand_fit {

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
};  // namespace hand_fit

#endif  // HAND_MODEL_HAND_MODEL_RENDERER_GEOMETRY_HEADER
