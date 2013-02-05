//
//  hand_model_geometry.h
//
//  Created by Jonathan Tompson on 10/26/12.
//
//  Base container class for all the hand geometry

#ifndef HAND_MODEL_HAND_MODEL_GEOMETRY_HEADER
#define HAND_MODEL_HAND_MODEL_GEOMETRY_HEADER

#include "hand_model/hand_model.h"
#include "Eigen"

#define NSPH_PER_FING 6

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

  class HandModelGeometry {
  public:
    // Constructor / Destructor
    HandModelGeometry(HandType hand_type) : hand_type_(hand_type) { }
    virtual ~HandModelGeometry() { }

    // Call before rendering hand depth maps:
    virtual void updateMatrices(const Eigen::MatrixXf& coeff) { }
    virtual void updateHeirachyMatrices() { }
    virtual void fixBoundingSphereMatrices() { }
    virtual inline renderer::Geometry* scene_graph() { return NULL; }

    virtual void renderStackReset() { }
    virtual renderer::Geometry* renderStackPop() { return NULL; }
    virtual bool renderStackEmpty() { return true; }

  protected:
    HandType hand_type_;

    // Non-copyable, non-assignable.
    HandModelGeometry(HandModelGeometry&);
    HandModelGeometry& operator=(const HandModelGeometry&);
  };
};  // namespace hand_model

#endif  // HAND_MODEL_HAND_MODEL_RENDERER_GEOMETRY_HEADER
