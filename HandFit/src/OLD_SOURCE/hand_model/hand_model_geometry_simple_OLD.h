//
//  hand_model_geometry_simple.h
//
//  Created by Jonathan Tompson on 10/26/12.
//
//  The ball and cylinder hand model

#ifndef HAND_MODEL_HAND_MODEL_GEOMETRY_SIMPLE_HEADER
#define HAND_MODEL_HAND_MODEL_GEOMETRY_SIMPLE_HEADER

#include "renderer/open_gl_common.h"  // GLfloat
#include "hand_model/hand_model.h"
#include "hand_model/hand_model_geometry.h"
#include "math/math_types.h"
#include "Eigen"

namespace renderer { class Geometry; }
namespace renderer { class Renderer; }

namespace hand_model {

  class HandModelGeometrySimple : public HandModelGeometry {
  public:
    // Constructor / Destructor
    HandModelGeometrySimple(HandType hand_type, renderer::Renderer* g_renderer);
    virtual ~HandModelGeometrySimple();

    // Call before rendering hand depth maps:
    virtual void updateMatrices(const Eigen::MatrixXf& coeff);
    virtual void updateHeirachyMatrices();
    virtual inline renderer::Geometry* scene_graph() { return scene_graph_; }

    virtual void renderStackReset();
    virtual renderer::Geometry* renderStackPop();
    virtual bool renderStackEmpty();

  private:
    // Note all geometry is attached to the global renderer's scene graph and
    // therefore we transfer ownership of the memory to it.
    renderer::Geometry* scene_graph_;  // The renderable geometry
    renderer::Geometry* palm_cylinder_;
    renderer::Geometry* palm_sphere1_;
    renderer::Geometry* palm_sphere2_;
    renderer::Geometry* wrist_base_;
    renderer::Geometry* wrist_cylinder_;
    renderer::Geometry* wrist_sphere1_;
    renderer::Geometry* wrist_sphere2_;
    renderer::Geometry* finger_k1_base_[4];
    renderer::Geometry* finger_k1_cylinder_[4];
    renderer::Geometry* finger_k1_sphere_[4];
    renderer::Geometry* finger_k2_base_[4];
    renderer::Geometry* finger_k2_cylinder_[4];
    renderer::Geometry* finger_k2_sphere_[4];
    renderer::Geometry* finger_k3_base_[4];
    renderer::Geometry* finger_k3_cylinder_[4];
    renderer::Geometry* finger_k3_sphere_[4];
    renderer::Geometry* thumb_k1_base_;
    renderer::Geometry* thumb_k1_cylinder_;
    renderer::Geometry* thumb_k1_sphere1_;
    renderer::Geometry* thumb_k1_sphere2_;
    renderer::Geometry* thumb_k2_base_;
    renderer::Geometry* thumb_k2_cylinder_;
    renderer::Geometry* thumb_k2_sphere_;
    renderer::Geometry* thumb_k3_base_;
    renderer::Geometry* thumb_k3_cylinder_;
    renderer::Geometry* thumb_k3_sphere_;

    // Temp matricies are not static to be thread safe.
    math::Float4x4 mat_tmp1;
    math::Float4x4 mat_tmp2;
    math::Float4x4 mat_tmp3;
    math::Float4x4 mat_rotateXAxis90;

    static const int nslices;  // even number
    static const int nstacks;  // odd number
    static const int knuckle_nslices;  // even number
    static const int knuckle_nstacks;  // odd number
    static const float finger_width_scale;

    static const float hand_palm_width;
    static const float hand_palm_thickness;
    static const float hand_palm_length;
    static const float wrist_width;
    static const float wrist_thickness;

    static const float finger_offsets[4];
    static const float finger_k1_lengths[4];
    static const float finger_k2_lengths[4];
    static const float finger_k3_lengths[4];
    static const float finger_k1_widths[4];
    static const float finger_k2_widths[4];
    static const float finger_k3_widths[4];

    static const float thumb_k1_length;
    static const float thumb_k2_length;
    static const float thumb_k3_length;
    static const float thumb_k1_width;
    static const float thumb_k2_width;
    static const float thumb_k3_width;

    void createHandGeometry(renderer::Renderer* g_renderer);

    // Copy of the Renderer's stack interface methods, I'd rather duplicate
    // them and keep the renderer seperate.
    data_str::Vector<renderer::Geometry*> render_stack_;

    // Non-copyable, non-assignable.
    HandModelGeometrySimple(HandModelGeometrySimple&);
    HandModelGeometrySimple& operator=(const HandModelGeometrySimple&);
  };
};  // namespace hand_model

#endif  // HAND_MODEL_HAND_MODEL_GEOMETRY_SIMPLE_HEADER
