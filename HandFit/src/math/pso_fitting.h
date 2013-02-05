//
//  pso_fitting.h
//
//  Created by Jonathan Tompson on 8/23/12.
//
//  A black box implementation of the Particle-swarm non-linear optimization
//  algorithm.  Does not require Hessian or Jacobian evaluations.
//
//  This is an implementation of the algorithm discribed in "An Off-The-Shelf
//  PSO" (with details filled out from the wikipedia article)
//
//  This class and its supporting functions takes in a set of data points y, 
//  evaluated at points x and will perform a non-linear fit for the user 
//  supplied function f(x,c) by modifying the function coefficients c.  The 
//  user supplied residual function is used to evaluate the "quality" of the 
//  fit (otherwise the L2 norm of the error is used if no residual function is
//  supplied).
//

#ifndef MATH_PSO_FITTING_HEADER
#define MATH_PSO_FITTING_HEADER

#include <random>
#include "math/math_types.h"
#include "math/common_fitting.h"
#include "Eigen"

#if defined(WIN32) || defined(_WIN32)
  #define constexpr 
#endif

// #define PSO_VERBOSE_SOLVER  // Print out per-iteration information

namespace math {

  // Differential-Evolution optimization
  class PSOFitting {
  public:
    // Set population size (num_agents) to -1 to let the optimizer choose a
    // population size for you.
    PSOFitting(uint32_t num_coeffs, int swarm_size_ = -1);
    ~PSOFitting();

    // fitModel = Top level function:
    // start_c --> The starting inital guess and the center of the intial 
    //           search radius.  Should be chosen very carefully to ensure 
    //           proper convergence.
    // radius_c --> The initial search radius.  Also, should be chosen
    //              very carefully to ensure proper convergence.
    // angle_coeff --> Since the mid-point of two angles needs to be interpreted
    //                 differently, the user can supply an array of boolians
    //                 indicating that the i-th coefficient is an angle.
    // obj_func --> Function to minimize, takes c and calculates a single
    //              float f(x, c).
    // coeff_norm_func --> After coefficient update, some coefficients may need
    //                     normalization (ie, rotation quaternion coeffs).  Set
    //                     to NULL if not needed.
    Eigen::MatrixXf& fitModel(const Eigen::MatrixXf& start_c,  // <1, num_coeffs>
                              const Eigen::MatrixXf& radius_c,  // <1, num_coeffs>
                              const bool* angle_coeff,  // [num_coeffs] or NULL
                              ObjectiveFuncPtr obj_func,
                              CoeffUpdateFuncPtr coeff_update_func);  // can be NULL

    // Termination and Optimization settings:
    float delta_coeff_termination;  // The spread of the agent coefficients
    uint64_t max_iterations;
    float phi_p;
    float phi_g;
    
    // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
    uint32_t num_coeffs_;
    uint32_t swarm_size_;
    Eigen::MatrixXf c_lo_;  // lower bound of search space
    Eigen::MatrixXf c_hi_;  // upper bound of search space
    Eigen::MatrixXf cur_c_min_;  // Current lower bound of the swarm's postions
    Eigen::MatrixXf cur_c_max_;  // Current upper bound of the swarm's postions
    Eigen::MatrixXf vel_max_;
    Eigen::MatrixXf delta_c_;
    float best_residue_global_;
    Eigen::MatrixXf best_pos_global_;
    float kappa_;  // Formula from paper

    SwarmNode* swarm_;
    const bool* angle_coeff_;

    static MERSINE_TWISTER_ENG eng;
    static UNIFORM_REAL_DISTRIBUTION dist_real;
    

    ObjectiveFuncPtr obj_func_;
    CoeffUpdateFuncPtr coeff_update_func_;
    CoeffPreturbFuncPtr coeff_preturb_func_;
    
    float interpolateCoeff(const float a, const float interp_val, 
      const float b, const float c, bool angle);
  };

};  // namespace math

#endif  // MATH_PSO_FITTING_HEADER
