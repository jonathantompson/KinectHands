//
//  robot_hand_model.h
//
//  Created by Jonathan Tompson on 5/22/13.
//
//  A dense modeled robot hand mesh.  Wrapper makes setting bone positions
//  from HandModelCoeff easier
//

#pragma once

#include "kinect_interface/hand_net/hand_model_coeff.h"
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector.h"
#include "jtil/data_str/vector_managed.h"
#include "jtil/data_str/pair.h"

namespace jtil { namespace renderer { class GeometryInstance; } }
namespace jtil { namespace renderer { class Geometry; } }
namespace jtil { namespace renderer { namespace objects { class BSphere; } } }

namespace kinect_interface {
namespace hand_net {

  class RobotHandModel {
  public:
    // Constructor / Destructor
    RobotHandModel(kinect_interface::hand_net::HandType hand_type);
    ~RobotHandModel();

    // Call before rendering hand depth maps:
    void updateMatrices(const float* coeff);
    void updateHeirachyMatrices();

    // setRendererAttachement - called when ModelFit wants to detach the model
    // from the global renderer
    void setRenderVisiblity(const bool visible);
    const bool getRenderVisiblity();

  private:
    // Note all geometry is attached to the global renderer's scene graph and
    // therefore we transfer ownership of the memory to it.
    jtil::renderer::GeometryInstance* model_;  // The geometry - Not owned here
    bool visible_;

    // BFS sorted array of nodes. Memory not owned here!
    // We need to keep around doubles for BFGS
    jtil::data_str::Vector<jtil::renderer::GeometryInstance*> nodes_;
    jtil::data_str::Vector<uint32_t> nodes_parents_;

    void loadRobotHandGeometry(kinect_interface::hand_net::HandType type);

    // References to bone indices for quick access (not owned here)
    uint32_t index_mesh_node_;
    uint32_t index_bone_wrist_;
    uint32_t index_bone_palm_;
    uint32_t index_bone_thumb_[3];
    uint32_t index_bone_finger1_[4];
    uint32_t index_bone_finger2_[4];
    uint32_t index_bone_finger3_[4];
    uint32_t index_bone_finger4_[4];

    void euler2RotMatGM(jtil::math::Float4x4& a, const float x_angle, 
      const float y_angle, const float z_angle);
    void rotateMatZAxisGM(jtil::math::Float4x4& ret, const float angle);
    void rotateMatYAxisGM(jtil::math::Float4x4& ret, const float angle);
    void rotateMatXAxisGM(jtil::math::Float4x4& ret, const float angle);
    void euler2RotMatGM(jtil::math::Double4x4& a, const double x_angle, 
      const double y_angle, const double z_angle);
    void rotateMatZAxisGM(jtil::math::Double4x4& ret, const double angle);
    void rotateMatYAxisGM(jtil::math::Double4x4& ret, const double angle);
    void rotateMatXAxisGM(jtil::math::Double4x4& ret, const double angle);

    // Non-copyable, non-assignable.
    RobotHandModel(RobotHandModel&);
    RobotHandModel& operator=(const RobotHandModel&);
  };
};  // namespace hand_net
};  // namespace kinect_interface
