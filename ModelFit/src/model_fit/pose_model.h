//
//  pose_shape.h
//
//  Created by Jonathan Tompson on 10/26/12.
//
//  Base container class for all the hand geometry

#ifndef MODEL_FIT_POSE_MODEL_HEADER
#define MODEL_FIT_POSE_MODEL_HEADER

namespace renderer { class Geometry; }

namespace model_fit {

  class PoseModel {
  public:
    // Constructor / Destructor
    PoseModel();
    virtual ~PoseModel() { }

    // Call before rendering hand depth maps:
    virtual void updateMatrices(const float* coeff) = 0;
    virtual void updateHeirachyMatrices() = 0;
    virtual void fixBoundingSphereMatrices() = 0;
    virtual inline renderer::Geometry* scene_graph() = 0;

    virtual void renderStackReset() = 0;
    virtual renderer::Geometry* renderStackPop() = 0;
    virtual bool renderStackEmpty() = 0;

  protected:
    // Non-copyable, non-assignable.
    PoseModel(PoseModel&);
    PoseModel& operator=(const PoseModel&);
  };
};  // namespace hand_fit

#endif  // MODEL_FIT_POSE_MODEL_HEADER
