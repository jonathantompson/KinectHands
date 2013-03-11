//
//  nm_fitting.h
//
//  Created by Jonathan Tompson on 8/23/12.
//
//  A black box implementation of the Nelder-Mead non-linear optimization
//  algorithm.  Does not require Hessian or Jacobian evaluations.
//
//  This class and its supporting functions takes in an initial coefficient 
//  set c_0 and minimizes the objective function f(c).
//
//  This Version is based on the modified NM in the book "Derivative Free
//  Optimization" by Conn, Scheinberg, Vicente (page 151)
//

#ifndef MATH_NM_FITTING_HEADER
#define MATH_NM_FITTING_HEADER

#include <random>
#include "jtil/math/math_types.h"
#include "math/common_fitting.h"
#include "Eigen"

#if defined(WIN32) || defined(_WIN32)
  #define constexpr 
#endif

// #define NM_VERBOSE_SOLVER  // Print out per-iteration information
#define NM_PRINT_TO_SCREEN_TIME_INTERVAL_SEC 0.5  // Otherwise if not verbose, how often to print out

namespace jtil {
namespace math {

  // Nelder-Mead optimization
  class NMFitting {
  public:
    NMFitting(uint32_t num_coeffs);
    ~NMFitting();

    // fitModel = Top level function:
    // start_c --> Starting value for fitting coefficient c
    // coeff_step_size --> A reasonable step size for each of the N coeffs, 
    //                     used in creation of the initial simplex.
    // angle_coeff --> Since the mid-point of two angles needs to be interpreted
    //                 differently, the user can supply an array of boolians
    //                 indicating that the i-th coefficient is an angle.
    // obj_func --> Function to minimize, takes c and calculates a single
    //              float f(x, c).
    // coeff_norm_func --> After coefficient update, some coefficients may need
    //                     normalization (ie, rotation quaternion coeffs).  Set
    //                     to NULL if not needed.
    Eigen::MatrixXf& fitModel(const Eigen::MatrixXf& start_c,  // <1, num_coeffs>
                              const Eigen::MatrixXf& coeff_step_size,  // <1, num_coeffs>
                              const bool* angle_coeff,  // [num_coeffs] or NULL
                              ObjectiveFuncPtr obj_func,
                              CoeffUpdateFuncPtr coeff_update_func);  // can be NULL

    // Termination and Optimization settings:
    double simplex_diameter_termination;
    double simplex_von_constant;
    uint64_t max_iterations;
    
    // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
    uint32_t num_coeffs_;
    double num_coeffs_factorial_;
    Eigen::MatrixXf c_center_;
    Eigen::MatrixXf c_reflect_;
    Eigen::MatrixXf c_expansion_;
    Eigen::MatrixXf c_contract_inside_;
    Eigen::MatrixXf c_contract_outside_;
    Eigen::MatrixXf c_contract_;
    Eigen::MatrixXf c_tmp_;
    Eigen::MatrixXd Y_matrix_;  // For calculating normalized volume
    OptNode* simplex_vert_;
    OptNode** ordered_vert_;
    OptNode* rotated_simplex_;
    const bool* angle_coeff_;
    
    static const float delta_r_;   // = 1.0 (reflection coefficient)
    static const float delta_e_;   // = 2.0 (expansion coefficient)
    static const float delta_oc_;  // = 0.5 (outside contraction coefficient)
    static const float delta_ic_;  // = -0.5 (inside contraction coefficient)
    static const float delta_s_;   // = 0.5 (shrink coefficient)

    static const double gamma_e_r_;  // = 1.0 (diameter modifiction coeff for reflection)
    static const double gamma_e_e_;  // = 2.0 (diameter modifiction coeff for expansions)

    ObjectiveFuncPtr obj_func_;
    CoeffUpdateFuncPtr coeff_update_func_;
    CoeffPreturbFuncPtr coeff_preturb_func_;

    static MERSINE_TWISTER_ENG eng;
    static NORMAL_REAL_DISTRIBUTION dist;
    
    void InsertionSortSimplexPts();
    void interpolateCoeff(Eigen::MatrixXf& ret, const Eigen::MatrixXf& a,
      const float interp_val, const Eigen::MatrixXf& b,
      const Eigen::MatrixXf& c);

    double calcApproxSimplexDiameter();  // O(n) --> But not accurate!

    double calcSimplexDiameter();  // O(n^2)
    double calcNormalizedVolume();
    double calcVolume();

    // Using the input coefficient as a replacement for the worst coefficient,
    // calculate the normalized volume and the diameter of the new simplex
    void calcVonAndDiamWithReplacement(const Eigen::MatrixXf& v_n, 
      double& volume_normalized, double& diameter);

    // Forcing functions are described on page 134 of the text
    inline double forcingFunction(double x) { return x * x; }
  };

};  // namespace math
};  // namespace jtil

#endif  // MATH_NM_FITTING_HEADER
