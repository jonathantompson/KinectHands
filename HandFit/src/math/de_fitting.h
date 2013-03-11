//
//  de_fitting.h
//
//  Created by Jonathan Tompson on 8/23/12.
//
//  A black box implementation of the Differential-Evolution non-linear optimization
//  algorithm.  Does not require Hessian or Jacobian evaluations.
//
//  Based on the DE algorithm described by "Good Parameters for Differential
//  Evolution" by Magnus Erik Hvass Pedersen:
//  http://www.hvass-labs.org/people/magnus/publications/pedersen10good-de.pdf
//  This is called: "DE/rand/1/bin"
//
//  This class and its supporting functions takes in a set of data points y, 
//  evaluated at points x and will perform a non-linear fit for the user 
//  supplied function f(x,c) by modifying the function coefficients c.  The 
//  user supplied residual function is used to evaluate the "quality" of the 
//  fit (otherwise the L2 norm of the error is used if no residual function is
//  supplied).
//

#ifndef MATH_DE_FITTING_HEADER
#define MATH_DE_FITTING_HEADER

#include <random>
#include "jtil/math/math_types.h"
#include "math/common_fitting.h"
#include "Eigen"

#if defined(WIN32) || defined(_WIN32)
  #define constexpr 
#endif

// #define DE_VERBOSE_SOLVER  // Print out per-iteration information
#define DE_PRINT_TO_SCREEN_TIME_INTERVAL_SEC 0.5  // Otherwise if not verbose, how often to print out

namespace jtil {
namespace math {

  // Differential-Evolution optimization
  class DEFitting {
  public:
    // Set population size (num_agents) to -1 to let the optimizer choose a
    // population size for you.
    DEFitting(uint32_t num_coeffs, int num_agents = -1);
    ~DEFitting();

    // fitModel = Top level function:
    // min_c --> The minimum coefficient in the search radius.  Should be chosen
    //           very carefully to ensure proper convergence.
    // max_c --> The minimum coefficient in the search radius.  Should be chosen
    //           very carefully to ensure proper convergence.
    // angle_coeff --> Since the mid-point of two angles needs to be interpreted
    //                 differently, the user can supply an array of boolians
    //                 indicating that the i-th coefficient is an angle.
    // obj_func --> Function to minimize, takes x and c and calculates a single
    //              float f(x, c).
    // coeff_norm_func --> After coefficient update, some coefficients may need
    //                     normalization (ie, rotation quaternion coeffs).  Set
    //                     to NULL if not needed.
    Eigen::MatrixXf& fitModel(const Eigen::MatrixXf& min_c,  // <1, num_coeffs>
                              const Eigen::MatrixXf& max_c,  // <1, num_coeffs>
                              const bool* angle_coeff,  // [num_coeffs] or NULL
                              ObjectiveFuncPtr obj_func,
                              CoeffUpdateFuncPtr coeff_update_func);  // can be NULL

    // Termination and Optimization settings:
    float delta_residue_termination;  // TYPICALLY NOT USED FOR DE SOLVERS!
    float delta_coeff_termination;  // The spread of the agent coefficients
    uint64_t max_iterations;
    float CR;  // Cross-over probability
    float F;  // Differential weight
    
    // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
    uint32_t num_coeffs_;
    uint32_t num_agents_;  // Otherwise known as the "NP" parameter in the literature
    Eigen::MatrixXf c_new_;
    Eigen::MatrixXf c_best_;
    Eigen::MatrixXf cur_c_min_;
    Eigen::MatrixXf cur_c_max_;
    Eigen::MatrixXf delta_c_;
    OptNode* agents_;
    OptNode* new_agents_;
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
};  // namespace jtil

#endif  // MATH_DE_FITTING_HEADER
