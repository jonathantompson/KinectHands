//
//  model_fit.h
//
//  Created by Jonathan Tompson on 8/17/12.
//
//  Code to perform PSO to fit a known graphical model to input depth data.  
//  It does this by rendering using openGL.
//
//  Obviously, the region of convergence is finite so starting values matter.
//  pso_radius_c sets the radius in each dimension that the PSO will sample
//  from.  This should be as tight as possible.
//
//  For better results, you should set the coeff_min_limit, coeff_max_limit and
//  coeff_penalty scale parameters to provide a prior on your coeff limits.
//  This can drasticly reduce the space of possible poses.
//
//  Finally, you must set the angle_coeffs boolean array.  This simply marks
//  those coefficients that represent angles so that the optimizer handles them
//  properly.
//

#ifndef MODEL_FIT_MODEL_FIT_HEADER
#define MODEL_FIT_MODEL_FIT_HEADER

#include "renderer/open_gl_common.h"  // GLfloat
#include "kinect_interface/depth_images_io.h"  // src_dim
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector.h"
#include "jtil/data_str/vector_managed.h"
#include "math/common_fitting.h"  // For CoeffUpdateFuncPtr

// Particle-Swarm optimizer settings
#define PSO_MAX_ITERATIONS 300  // Default 501
#define PSO_DELTA_C_TERMINATION 1e-3f
#define PSO_SWARM_SIZE 64  // Default 128 or 64
#define PSO_REPEATS 1  // Default 2
 
// #define PREV_FRAME_DIST_PENALTY  // Distance from last frame penalty
#define PREV_FRAME_DIST_PENALTY_SCALE  0.001f
#define PREV_FRAME_DIST_PENALTY_THRESHOLD  0.3f  // % of total range
#define MAX_DEPTH_IN_RESIDUE 30.0f  // default 40 (from paper) but 30 works better
#define DATA_TERM_LAMBDA 0.2f  // default 0.0025f  (higher values = depth difference is more important)
#define INTERPENETRATION_CONSTANT 0.1f

#define NTILES_X 8  // Don't change these --> If you do you'll have to change 
#define NTILES_Y 8  //                        the downsample chain
#define NTILES (NTILES_X * NTILES_Y)

namespace jtil { namespace math { class LPRPSOFittingModelFit; } }
 
namespace model_fit {
  class ModelRenderer;

  class ModelFit {
  public:
    friend class jtil::math::LPRPSOFittingModelFit;
    // Constructor / Destructor
    ModelFit(uint32_t num_models, uint32_t coeff_size_per_model,
      float* pso_radius_c, jtil::math::CoeffUpdateFuncPtr renormalize_coeff_func);
    ~ModelFit();

    void fitModel(int16_t* depth, uint8_t* label, float** coeffs,
      float** prev_coeffs[2]);
    float queryObjectiveFunction(int16_t* depth, uint8_t* label, 
      float** coeffs, float** prev_coeffs[2]);

    inline void resetFuncEvalCount() { func_eval_count_ = 0; }
    inline uint64_t func_eval_count() { return func_eval_count_; }
 

    // NOTE: THESE SHOULD BE SET BEFORE FITTING 
    static const float* coeff_min_limit;
    static const float* coeff_max_limit;
    static const float* coeff_penalty_scale;

    // NOTE: THIS MUST BE SET BEFORE FITTING 
    static const bool* angle_coeffs;

    static float calcDistPenalty(const float* coeff0, const float* coeff1);

  private:
    uint32_t coeff_dim_; 
    uint32_t num_models_;
    static ModelFit* cur_fit_;
    static uint64_t func_eval_count_;
    ModelRenderer* model_renderer_;

    float* coeff_;
    float* prev_coeff_;

    jtil::math::LPRPSOFittingModelFit* lprpso_;
    float* coeff_tmp_;
    float* cur_obj_func_coeff_;
    float* pso_radius_c_;  // Not owned here
    int16_t kinect_depth_masked_[src_dim];
    static const float finger_crossover_penalty_threshold;

    // static functions for non-linear fitting
    static void evalFunc(Eigen::MatrixXf& f_val, const Eigen::MatrixXf& coeff, 
      const Eigen::MatrixXf& x);
    static void preturbCoeffs(Eigen::MatrixXf& coeff);
    static float calculateResidual(const Eigen::MatrixXf& y, 
      const Eigen::MatrixXf& f_x, const Eigen::MatrixXf& coeff);
    static void calculateResidualTiled(jtil::data_str::Vector<float>& residues, 
      jtil::data_str::VectorManaged<Eigen::MatrixXf>& coeffs);
    static void approxJacobian(Eigen::MatrixXf& jacob, 
      const Eigen::MatrixXf& coeff);
    static void approxJacobianTiled(Eigen::MatrixXf& jacob, 
      const Eigen::MatrixXf& coeff);
    static float calcPenalty(const Eigen::MatrixXf& coeff);
    // Main objective function used for fitting.
    static float objectiveFunc(const Eigen::MatrixXf& coeff);
    static void objectiveFuncTiled(jtil::data_str::Vector<float>& residues, 
      jtil::data_str::VectorManaged<Eigen::MatrixXf>& coeffs);

    void manualFit();
    float evaluateFuncAndCalculateResidual();
    void prepareKinectData(int16_t* depth, uint8_t* label);

    // Non-copyable, non-assignable.
    ModelFit(ModelFit&);
    ModelFit& operator=(const ModelFit&);
  };
};  // namespace model_fit

#endif  // MODEL_FIT_MODEL_FIT_HEADER
