//
//  calibrate geometry.h
//
//  Created by Jonathan Tompson on 10/30/12.
//
//  3 Spheres connected by a stick
//

#ifndef MODEL_FIT_CALIBRATE_GEOMETRY_HEADER
#define MODEL_FIT_CALIBRATE_GEOMETRY_HEADER

#include "renderer/open_gl_common.h"  // GLfloat
#include "kinect_interface/hand_net/hand_model_coeff.h"
#include "model_fit/pose_model.h"
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector.h"
#include "jtil/data_str/pair.h"

#define CAL_GEOM_NUM_COEFF 7
#define SPHERE_RADIUS (65.5f / 2.0f)
#define SPHERE_A_OFST 97.0  // From center of cross to outside of ball (actually 95)
#define SPHERE_B_OFST 92.0
#define SPHERE_C_OFST 91.5
#define SPHERE_NSTACKS 15
#define SPHERE_NSLICES 15
#define SPHERE_BASE_RADIUS 1.0f
#define CYL_RADIUS 10.0f // It's actually 8
#define CYL_NSLICES 15
#define CYL_BASE_RADIUS 1.0f
#define CYL_BASE_HEIGHT 2.0f

#define NUM_CAL_SPHERES 0
namespace jtil { namespace math { class BFGS; } }

namespace renderer { class Geometry; }
namespace renderer { class Renderer; }

namespace model_fit {
  class ModelRenderer;

  typedef enum {
    CALIB_POS_X        = 0, 
    CALIB_POS_Y        = 1,
    CALIB_POS_Z        = 2,
    CALIB_ORIENT_X     = 3,
    CALIB_ORIENT_Y     = 4,
    CALIB_ORIENT_Z     = 5,
    CALIB_SCALE        = 6,
    NUM_PARAMETERS     = 7,
  } CalibrateCoeff;

  class CalibrateGeometry : public PoseModel {
  public:
    // Constructor / Destructor
    CalibrateGeometry();
    virtual ~CalibrateGeometry();

    // Call before rendering hand depth maps:
    virtual void updateMatrices(const float* coeff);
    virtual void updateHeirachyMatrices();
    virtual void fixBoundingSphereMatrices();
    virtual inline renderer::Geometry* scene_graph() { return scene_graph_; }
    
    jtil::data_str::Vector<renderer::BoundingSphere*>& bspheres() { return bspheres_; }

    // From the fitted coeffs, find an "average" affine transformation that
    // describes i_query_cam in i_base_cam's coordinate system.
    void calcAveCameraView(jtil::math::Float4x4& ret, 
      const uint32_t i_base_cam, const uint32_t i_query_cam, 
      const float*** coeffs, const uint32_t num_frames);

    virtual void renderStackReset();
    virtual renderer::Geometry* renderStackPop();
    virtual bool renderStackEmpty();

    // setRendererAttachement - called when ModelFit wants to detach the model
    // from the global renderer
    virtual void setRendererAttachement(const bool renderer_attachment);
    virtual const bool getRendererAttachement();

    virtual const bool* angle_coeffs() { return angle_coeffs_; }
    virtual const float* pso_radius_c() { return pso_radius_c_; }
    virtual const float* coeff_min_limit() { return coeff_min_limit_; }
    virtual const float* coeff_max_limit() { return coeff_max_limit_; }
    virtual const float* coeff_penalty_scale() { return coeff_penalty_scale_; }
    virtual const uint32_t max_bsphere_groups() { return 6; }

    static void renormalizeCoeffs(float* coeff);

    // Since there are a few coeffs that we cannot get from the PSO (since
    // the PSO doesn't optimize them), we must keep track of some coeff parameters
    // that are going to be constant accross an optimization run.
    static void setCurrentStaticHandProperties(const float* coeff);

  private:
    static float cur_scale_;
    renderer::Geometry* scene_graph_;  // The renderable geometry - Not owned here
    renderer::Geometry* sphere_a_;
    renderer::Geometry* sphere_b_;
    renderer::Geometry* sphere_c_;
    renderer::Geometry* cylinder_a_;
    renderer::Geometry* cylinder_b_;
    jtil::data_str::Vector<renderer::BoundingSphere*> bspheres_;  // Attached to scene graph!
    bool renderer_attachment_;  // whether or not the model is attached to the 
                                // global renderer's scene graph

    // Temp matricies are not static to be thread safe.
    jtil::math::Float4x4 mat_tmp1;

    // Copy of the Renderer's stack interface methods, I'd rather duplicate
    // them and keep the renderer seperate.
    jtil::data_str::Vector<renderer::Geometry*> render_stack_;

    static void euler2RotMatGM(jtil::math::Float4x4& a, const float x_angle, 
      const float y_angle, const float z_angle);

    static const float coeff_min_limit_[CAL_GEOM_NUM_COEFF];
    static const float coeff_max_limit_[CAL_GEOM_NUM_COEFF];
    static const float coeff_penalty_scale_[CAL_GEOM_NUM_COEFF];
    static const bool angle_coeffs_[CAL_GEOM_NUM_COEFF];
    static float pso_radius_c_[CAL_GEOM_NUM_COEFF];

    // Satic data used for calculating average coordinate frame
    static jtil::math::BFGS* solver_;
    static jtil::math::Float3* vq_[3];
    static jtil::math::Float3* vb_[3];
    static jtil::math::Float3 vmodel_[3];
    static void coeff2Mat(jtil::math::Float4x4& mat, const float* coeff);

    // Non-copyable, non-assignable.
    CalibrateGeometry(CalibrateGeometry&);
    CalibrateGeometry& operator=(const CalibrateGeometry&);
  };
};  // namespace hand_fit

#endif  // MODEL_FIT_CALIBRATE_GEOMETRY_HEADER
