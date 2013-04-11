//
//  hand_model_fitting.h
//
//  Created by Jonathan Tompson on 8/17/12.
//
//  Code to perform Levenberg-Marquardt non-linear optimization to fit a very
//  simple hand model to input depth data.  It does this by rendering the
//  hand model using openGL then, estimating the jacobian using finite 
//  differences, and using this in the internal loop of LM.
//
//  Obviously, the region of convergence is finite so starting values matter.
//  Secondly, none of the functions here are thread safe.

#ifndef HAND_MODEL_HAND_MODEL_FIT_HEADER
#define HAND_MODEL_HAND_MODEL_FIT_HEADER

#include "renderer/open_gl_common.h"  // GLfloat
#include "kinect_interface/hand_net/hand_model.h"
#include "kinect_interface/depth_images_io.h"  // src_dim
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector.h"
#include "jtil/data_str/vector_managed.h"
#include "Eigen"

// Nelder-Mead optimizer settings
#define NM_NUM_RESARTS 3
#define NM_MAX_ITERATIONS 400
#define NM_SIMPLEX_DIAMETER_TERMINATION 1e-3f;

// Particle-Swarm optimizer settings
#define PSO_MAX_ITERATIONS 300  // Default 501
#define PSO_DELTA_C_TERMINATION 1e-3f
#define PSO_SWARM_SIZE 64  // Default 128 or 64
#define PSO_RAD_FINGERS 0.40f  // Search radius in frac of min - max coeff - Def 0.4
#define PSO_RAD_WRIST 0.40f
#define PSO_RAD_THUMB 0.40f 
#define PSO_RAD_EULER 0.40f
#define PSO_RAD_POSITION 25  // Absolute value in mm
#define PSO_REPEATS 1  // Default 2

// Manual Fit
#define MANUAL_FIT_SAMPLE_RADIUS 0.5f  // fraction of entire range
#define MANUAL_FIT_POS_RADIUS 50.0f
#define MANUAL_FIT_SAMPLE_SIZE 64  // Make a multiple of ntiles
#define MANUAL_FIT_NUM_REPEATS 4
#define MANUAL_FIT_RADIUS_DECAY 0.25f  // rad_k_p1 = rad_k * decay

// BFGS Descent method fit
#define DESCENT_NUM_REPEATS 10
#define DESCENT_PENALTY_SIZE 0.1f
#define DESCENT_DELTA_X_2_NORM_TERM 1e-5f
#define DESCENT_DELTA_F_TERM 1e-5f
#define DESCENT_PENALTY_SIZE 0.1f
#define DESCENT_PENALTY_SIZE 0.1f 
 
#define DEPTH_ONLY_RESIDUE_FUNC  // Faster but less accurate
#define MAX_DEPTH_IN_RESIDUE 30.0f  // default 40 (from paper) but 30 works better
#define DATA_TERM_LAMBDA 0.2f  // default 0.0025f  (higher values = depth difference is more important)
#define INTERPENETRATION_ALLOWENCE 2.0f  // Let them intepenetrate a little bit
#define LINEAR_INTERPENETRATION_PENALTY  // otherwise quadratic
#ifdef LINEAR_INTERPENETRATION_PENALTY
  #define INTERPENETRATION_CONSTANT 0.32f
#else
  #define INTERPENETRATION_CONSTANT 0.1f
#endif

namespace jtil {
  namespace math {
    class NMFitting;
    class BFGSFitting;
    class LPRPSOFittingHands; 
  }
}
 
namespace hand_fit {
  
  class HandRenderer;

  struct Coeff { 
    float c[HAND_NUM_COEFF*2];
    Coeff& operator=(const Coeff &rhs);
    void copyFromEigen(const Eigen::MatrixXf &c);
    void copyToEigen(Eigen::MatrixXf &c);
  };

  class HandFit {
  public:
    friend class jtil::math::LPRPSOFittingHands;
    // Constructor / Destructor
    HandFit(HandRenderer* hand_renderer, uint32_t num_hands);
    ~HandFit();

    // Fit this hand to the input depth data:
    // hands[0] = left_hand    --> or if num_hands = 1, then right_hand
    // hands[1] = right_hand   --> or if num_hands = 1, then NULL
    void fitModel(int16_t* depth, uint8_t* label, 
      kinect_interface::hand_net::HandModel* hands[2]);
    float queryObjectiveFunction(int16_t* depth, uint8_t* label, 
      kinect_interface::hand_net::HandModel* hands[2]);

    inline void resetFuncEvalCount() { func_eval_count_ = 0; }
    inline uint64_t func_eval_count() { return func_eval_count_; }
  
    // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    float calcPenalty();

    static const float coeff_min_limit[HAND_NUM_COEFF];
    static const float coeff_max_limit[HAND_NUM_COEFF];

  private:
    uint32_t coeff_dim_;  // Either HAND_NUM_COEFF or 2*HAND_NUM_COEFF
    uint32_t nhands_;
    static HandFit* cur_fit_;
    static kinect_interface::hand_net::HandModel* cur_hand_[2];
    static uint64_t func_eval_count_;

    HandRenderer* hand_renderer_;
    Eigen::MatrixXf coeff_;

    jtil::math::NMFitting* nm_fitting_;
    jtil::math::LPRPSOFittingHands* lprpso_fitting_;
    jtil::math::BFGSFitting* bfgs_fitting_;
    Eigen::MatrixXf coeff_tmp_;
    Eigen::MatrixXf cur_obj_func_coeff_;
    Eigen::MatrixXf nm_coeff_step_size_;
    Eigen::MatrixXf pso_radius_c_;
    int16_t kinect_depth_masked_[src_dim];
    static const float delta_coeff_[HAND_NUM_COEFF];
    static const float preturb_coeff_[HAND_NUM_COEFF];
    static const bool angle_coeffs[HAND_NUM_COEFF*2];
    static const float coeff_penalty_scale_[HAND_NUM_COEFF];
    static const float finger_crossover_penalty_threshold;
    static const uint32_t manual_fit_order_[HAND_NUM_COEFF];
    static jtil::data_str::Vector<Coeff> min_pts_;
    static bool include_min_pts_constraints_;
    static jtil::data_str::VectorManaged<Eigen::MatrixXf> tiled_coeffs_;
    static jtil::data_str::Vector<float> tiled_residues_;
    kinect_interface::hand_net::HandModel* cur_hand_to_fit_;

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
    HandFit(HandFit&);
    HandFit& operator=(const HandFit&);
  };
};  // unnamed namespace

#endif  // HAND_MODEL_HAND_MODEL_FIT_HEADER
