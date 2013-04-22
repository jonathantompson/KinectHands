#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include "model_fit/model_renderer.h"
#include "model_fit/calibrate_geometry.h"
#include "renderer/open_gl_common.h"
#include "renderer/renderer.h"
#include "renderer/colors.h"
#include "renderer/geometry/geometry_manager.h"
#include "renderer/geometry/geometry.h"
#include "renderer/geometry/geometry_vertices.h"
#include "renderer/geometry/geometry_colored_mesh.h"
#include "renderer/geometry/bone_info.h"
#include "jtil/data_str/pair.h"
#include "kinect_interface/hand_net/hand_net.h"  // for HandCoeffConvnet
#include "kinect_interface/hand_net/hand_image_generator.h"  // for HN_HAND_SIZE

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }

#ifndef HAND_FIT
#error "HAND_FIT is not defined!  You need to declare it in the preprocessor"
#endif

using namespace jtil::math;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using namespace renderer;
using namespace kinect_interface::hand_net;

namespace model_fit {
  float CalibrateGeometry::pso_radius_c_[CAL_GEOM_NUM_COEFF];
  float CalibrateGeometry::cur_scale_;
  
  CalibrateGeometry::CalibrateGeometry() {
    Float3 tennis_ball_yellow(0.776470f, 0.929412f, 0.172549f);
    Float3 wood_brown(0.50588f, 0.48235f, 0.11372f);
    sphere_a_ = GeometryColoredMesh::makeSphere(SPHERE_NSTACKS, SPHERE_NSLICES,
      SPHERE_BASE_RADIUS, tennis_ball_yellow);
    sphere_b_ = GeometryColoredMesh::makeSphere(SPHERE_NSTACKS, SPHERE_NSLICES,
      SPHERE_BASE_RADIUS, tennis_ball_yellow);
    sphere_c_ = GeometryColoredMesh::makeSphere(SPHERE_NSTACKS, SPHERE_NSLICES,
      SPHERE_BASE_RADIUS, tennis_ball_yellow);
    cylinder_a_ = GeometryColoredMesh::makeCylinder(CYL_NSLICES,
      CYL_BASE_HEIGHT, CYL_BASE_RADIUS, CYL_BASE_RADIUS, wood_brown);
    cylinder_b_ = GeometryColoredMesh::makeCylinder(CYL_NSLICES,
      CYL_BASE_HEIGHT, CYL_BASE_RADIUS, CYL_BASE_RADIUS, wood_brown);

    scene_graph_ = new Geometry();
    scene_graph_->addChild(sphere_a_);
    scene_graph_->addChild(sphere_b_);
    scene_graph_->addChild(sphere_c_);
    scene_graph_->addChild(cylinder_a_);
    scene_graph_->addChild(cylinder_b_);

    Float3 scale_vec(SPHERE_RADIUS, SPHERE_RADIUS, SPHERE_RADIUS);
    Float4x4::scaleMat(*sphere_a_->mat(), scale_vec);
    Float4x4::scaleMat(*sphere_b_->mat(), scale_vec);
    Float4x4::scaleMat(*sphere_c_->mat(), scale_vec);

    sphere_a_->mat()->leftMultTranslation(0, (SPHERE_A_OFST + SPHERE_RADIUS), 0);
    sphere_b_->mat()->leftMultTranslation(-(SPHERE_B_OFST + SPHERE_RADIUS), 0, 0);
    sphere_c_->mat()->leftMultTranslation((SPHERE_C_OFST + SPHERE_RADIUS), 0, 0);

    scale_vec.set(CYL_RADIUS, SPHERE_A_OFST + SPHERE_RADIUS, CYL_RADIUS);
    Float4x4::scaleMat(*cylinder_b_->mat(), scale_vec);
    cylinder_b_->mat()->leftMultRotateZAxis((float)M_PI_2);
    scale_vec.set(CYL_RADIUS, SPHERE_B_OFST + SPHERE_RADIUS, CYL_RADIUS);
    Float4x4::scaleMat(*cylinder_a_->mat(), scale_vec);

    for (uint32_t i = 0; i < 3; i++) {
      pso_radius_c_[i] = 25.0f;
    }
    for (uint32_t i = 3; i < 6; i++) {
      pso_radius_c_[i] = 0.5f;
    }

    // Adding the geometry to the gobal geometry manager's scene graph will
    // transfer ownership of the memory.
    GeometryManager::scene_graph_root()->addChild(scene_graph_);
    renderer_attachment_ = true;

  }

  CalibrateGeometry::~CalibrateGeometry() {
    // Note, ownership of all geometry is transfered to the renderer class
  }

  void CalibrateGeometry::updateMatrices(const float* coeff) {
    Float4x4* mat;

    // Set the root matrix:
    mat = scene_graph_->mat();
    euler2RotMatGM(*mat, coeff[CALIB_ORIENT_X], coeff[CALIB_ORIENT_Y],
      coeff[CALIB_ORIENT_Z]);
    mat->leftMultTranslation(coeff[CALIB_POS_X],
                             coeff[CALIB_POS_Y],
                             coeff[CALIB_POS_Z]);
    mat->rightMultScale(coeff[CALIB_SCALE], coeff[CALIB_SCALE], coeff[CALIB_SCALE]);
  }
  
  void CalibrateGeometry::renderStackReset() {
    render_stack_.resize(0);  // empty the stack (without deallocating)
    // Seed the render stack with the root node
    render_stack_.pushBack(scene_graph_);
  }

  Geometry* CalibrateGeometry::renderStackPop() {
    Geometry* ret = NULL;
    if (render_stack_.size() > 0) {
      render_stack_.popBackUnsafe(ret);  // Remove the last element

      // Now add the children to the geometry stack
      for (uint32_t i = 0; i < ret->numChildren(); i ++) {
        render_stack_.pushBack(ret->getChild(i));
      }
    }
    return ret;
  }

  bool CalibrateGeometry::renderStackEmpty() {
    return render_stack_.size() == 0;
  }

  void CalibrateGeometry::updateHeirachyMatrices() {
    renderStackReset();
    while (!renderStackEmpty()) {
      Geometry* cur_geom = renderStackPop();
      // Update the render matrix based on our parents position
      if (cur_geom->parent() != NULL) {
        Float4x4::multSIMD(*cur_geom->mat_hierarchy(),
          *cur_geom->parent()->mat_hierarchy(), *cur_geom->mat());

      } else {
        cur_geom->mat_hierarchy()->set(*cur_geom->mat());
      }
    }
  }

  void CalibrateGeometry::fixBoundingSphereMatrices() {
  }

  void CalibrateGeometry::setRendererAttachement(const bool renderer_attachment) {
    if (renderer_attachment_ != renderer_attachment) {
      renderer_attachment_ = renderer_attachment;
      Geometry* g = scene_graph_;
      if (!renderer_attachment_) {
        g->parent()->removeChild(g);
      } else {
        GeometryManager::g_geom_manager()->scene_graph_root()->addChild(g);
      }
    }
  }

  const bool CalibrateGeometry::getRendererAttachement() {
    return renderer_attachment_;
  }

  // These next few methods are to avoid the cos and sin double functions in 
  // the Mat4x4 template
  void CalibrateGeometry::euler2RotMatGM(Float4x4& a, const float x_angle, 
    const float y_angle, const float z_angle) {
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
    float c1 = cosf(x_angle);
    float s1 = sinf(x_angle);
    float c2 = cosf(y_angle);
    float s2 = sinf(y_angle);
    float c3 = cosf(z_angle);
    float s3 = sinf(z_angle);
#ifdef COLUMN_MAJOR
    a.m[0] = c1*c2;
    a.m[4] = -c1*s2*c3 + s1*s3;
    a.m[8] = c1*s2*s3 + s1*c3;
    a.m[12] = 0;
    a.m[1] = s2;
    a.m[5] = c2*c3;
    a.m[9] = -c2*s3;
    a.m[13] = 0;
    a.m[2] = -s1*c2;
    a.m[6] = s1*s2*c3 + c1*s3;
    a.m[10] = -s1*s2*s3 + c1*c3;
    a.m[14] = 0;
    a.m[3] = 0;
    a.m[7] = 0;
    a.m[11] = 0;
    a.m[15] = 1;
#endif
#ifdef ROW_MAJOR
    a.m[0] = c1*c2;
    a.m[1] = -c1*s2*c3 + s1*s3;
    a.m[2] = c1*s2*s3 + s1*c3;
    a.m[3] = 0;
    a.m[4] = s2;
    a.m[5] = c2*c3;
    a.m[6] = -c2*s3;
    a.m[7] = 0;
    a.m[8] = -s1*c2;
    a.m[9] = s1*s2*c3 + c1*s3;
    a.m[10] = -s1*s2*s3 + c1*c3;
    a.m[11] = 0;
    a.m[12] = 0;
    a.m[13] = 0;
    a.m[14] = 0;
    a.m[15] = 1;
#endif
  }


  // modulu - similar to matlab's mod()
  // result is always possitive. not similar to fmod()
  // Mod(-3,4)= 1   fmod(-3,4)= -3
#if defined(__APPLE__) || defined(_WIN32)
  float inline __fastcall Mod(float x, float y) {
    if (0 == y) {
      return x;
    }
    
    return x - y * floor(x / y);
  }
#else
  float inline Mod(float x, float y) {
    if (0 == y) {
      return x;
    }

    return x - y * floor(x / y);
  }
#endif

  // wrap [rad] angle to [0...2PI)
  inline void WrapTwo2PI(float& angle) {
    angle = Mod(angle, static_cast<float>(2.0 * M_PI));
  }

  // wrap [rad] angle to [-PI...PI)
  inline void WrapTwoPI(float& angle) {
    angle = Mod(angle + static_cast<float>(M_PI), 
      static_cast<float>(2.0 * M_PI)) - static_cast<float>(M_PI);
  }

  FloatQuat tmp_quat_;
  void CalibrateGeometry::renormalizeCoeffs(float* coeff) {
    // Set all angles 0 --> 2pi
    for (uint32_t i = HAND_ORIENT_X; i < CAL_GEOM_NUM_COEFF; i++) {
      WrapTwoPI(coeff[i]);
    }
  }

  void CalibrateGeometry::setCurrentStaticHandProperties(
    const float* coeff) {
    cur_scale_ = coeff[CALIB_SCALE];
  }

  // coeff_min_limit_ is the minimum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float CalibrateGeometry::coeff_min_limit_[CAL_GEOM_NUM_COEFF] = {
    -std::numeric_limits<float>::infinity(),    // HAND_POS_X
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    -3.14159f,  // HAND_ORIENT_X
    -3.14159f,  // HAND_ORIENT_Y
    -3.14159f,  // HAND_ORIENT_Z
  };
  
  // coeff_max_limit_ is the maximum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float CalibrateGeometry::coeff_max_limit_[CAL_GEOM_NUM_COEFF] = {
    std::numeric_limits<float>::infinity(),    // HAND_POS_X
    std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    3.14159f,  // HAND_ORIENT_X
    3.14159f,  // HAND_ORIENT_Y
    3.14159f,  // HAND_ORIENT_Z
  };
  
  // coeff_penalty_scale_ is the exponential scale to use when penalizing coeffs
  // outside the min and max values.
  const float CalibrateGeometry::coeff_penalty_scale_[CAL_GEOM_NUM_COEFF] = {
    0,    // HAND_POS_X
    0,    // HAND_POS_Y
    0,    // HAND_POS_Z
    0,  // HAND_ORIENT_X
    0,  // HAND_ORIENT_Y
    0,  // HAND_ORIENT_Z
  };

// angle_coeffs are boolean values indicating if the coefficient represents
  // a pure angle (0 --> 2pi)
  const bool CalibrateGeometry::angle_coeffs_[CAL_GEOM_NUM_COEFF] = {
    // Hand 1
    false,  // HAND_POS_X
    false,  // HAND_POS_Y
    false,  // HAND_POS_Z
    true,  // HAND_ORIENT_X
    true,  // HAND_ORIENT_Y
    true,  // HAND_ORIENT_Z
  };

}  // namespace hand_fit
