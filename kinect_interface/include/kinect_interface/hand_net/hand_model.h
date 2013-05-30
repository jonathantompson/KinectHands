//
//  hand_model.h
//
//  Created by Jonathan Tompson on 5/22/13.
//
//  A dense modeled hand mesh (linear blend skinning) --> this version is for
//  my deferred rendering engine, and doesn't include any of the PSO related 
//  stuff from HandGeometryMesh.  It's a simple wrapper in order to update
//  from a coeff input.
//

#ifndef KINECT_INTERFACE_HAND_NET_HAND_MODEL_HEADER
#define KINECT_INTERFACE_HAND_NET_HAND_MODEL_HEADER

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

  class HandModel {
  public:
    // Constructor / Destructor
    HandModel(kinect_interface::hand_net::HandType hand_type);
    ~HandModel();

    // Call before rendering hand depth maps:
    void updateMatrices(const float* coeff);
    void updateHeirachyMatrices();
    void fixBoundingSphereMatrices();

    // setRendererAttachement - called when ModelFit wants to detach the model
    // from the global renderer
    void setRenderVisiblity(const bool visible);
    const bool getRenderVisiblity();

    static const bool* angle_coeffs() { return angle_coeffs_; }
    static const float* coeff_min_limit() { return coeff_min_limit_; }
    static const float* coeff_max_limit() { return coeff_max_limit_; }
    static const float* coeff_penalty_scale() { return coeff_penalty_scale_; }
    static const uint32_t max_bsphere_groups() { return 6; }
    static const uint32_t num_bspheres_per_group() { return 6; }

    void calcBoundingSphereUVDPos(float* uvd, const uint32_t b_sphere_index, 
      const jtil::math::Float4x4& pv_mat);

  private:
    // Note all geometry is attached to the global renderer's scene graph and
    // therefore we transfer ownership of the memory to it.
    jtil::renderer::GeometryInstance* model_;  // The renderable geometry - Not owned here
    jtil::data_str::VectorManaged<jtil::renderer::objects::BSphere*> bspheres_;
    bool visible_;

    void loadHandGeometry(kinect_interface::hand_net::HandType type);

    // References to bones for quick access (not owned here)
    jtil::renderer::GeometryInstance* mesh_node_;
    jtil::renderer::GeometryInstance* bone_wrist_;
    jtil::renderer::GeometryInstance* bone_palm_;
    jtil::renderer::GeometryInstance* bone_thumb_[3];
    jtil::renderer::GeometryInstance* bone_finger1_[4];
    jtil::renderer::GeometryInstance* bone_finger2_[4];
    jtil::renderer::GeometryInstance* bone_finger3_[4];
    jtil::renderer::GeometryInstance* bone_finger4_[4];

    // Copy of the Renderer's stack interface methods, I'd rather duplicate
    // them and keep the renderer seperate.
    jtil::data_str::Vector<jtil::renderer::GeometryInstance*> render_stack_;

    void euler2RotMatGM(jtil::math::Float4x4& a, const float x_angle, 
      const float y_angle, const float z_angle);
    void rotateMatZAxisGM(jtil::math::Float4x4& ret, const float angle);
    void rotateMatYAxisGM(jtil::math::Float4x4& ret, const float angle);
    void rotateMatXAxisGM(jtil::math::Float4x4& ret, const float angle);

    static const float coeff_min_limit_[HAND_NUM_COEFF];
    static const float coeff_max_limit_[HAND_NUM_COEFF];
    static const float coeff_penalty_scale_[HAND_NUM_COEFF];
    static const bool angle_coeffs_[HAND_NUM_COEFF];

    virtual void renderStackReset();
    virtual jtil::renderer::GeometryInstance* renderStackPop();
    virtual bool renderStackEmpty();

    void addBoneBSphere(const uint32_t ibone, 
      jtil::renderer::GeometryInstance* bone);

    // Non-copyable, non-assignable.
    HandModel(HandModel&);
    HandModel& operator=(const HandModel&);
  };
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_HAND_MODEL_HEADER
