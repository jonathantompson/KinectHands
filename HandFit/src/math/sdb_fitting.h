//
//  sdb_fitting.h
//
//  Created by Jonathan Tompson on 11/20/12.
//
//  A black box implementation of the steepest-descent with linear-search
//  backtracking non-linear optimization algorithm.  Requires Jacobian function
//  (which you can estimate using central differencing)
//

#ifndef MATH_SDB_FITTING_HEADER
#define MATH_SDB_FITTING_HEADER

#include <random>
#include "jtil/math/math_types.h"
#include "math/common_fitting.h"
#include "Eigen"

#if defined(WIN32) || defined(_WIN32)
  #define constexpr 
#endif

#define SDB_VERBOSE_SOLVER  // Print out per-iteration information

namespace jtil {
namespace math {

  // Steepest Descent with backtracking optimization
  class SDBFitting {
  public:
    SDBFitting(uint32_t num_coeffs);
    ~SDBFitting();

    // fitModel = Top level function:
    // start_c --> Starting value for fitting coefficient c
    // angle_coeff --> Since the mid-point of two angles needs to be interpreted
    //                 differently, the user can supply an array of boolians
    //                 indicating that the i-th coefficient is an angle.
    // obj_func --> Function to minimize, takes c and calculates a single
    //              float f(x, c).
    // jac_func --> Jacobian function, takes c and x and calculates n-vector
    //              of partial derivatives in f.
    // coeff_norm_func --> After coefficient update, some coefficients may need
    //                     normalization (ie, rotation quaternion coeffs).  Set
    //                     to NULL if not needed.
    Eigen::MatrixXf& fitModel(const Eigen::MatrixXf& start_c,  // <1, num_coeffs>
                              const bool* angle_coeff,  // [num_coeffs] or NULL
                              ObjectiveFuncPtr obj_func,
                              JacobianFuncPtr jac_func,
                              CoeffUpdateFuncPtr coeff_update_func);  // can be NULL

    // Termination and Optimization settings:
    float eta_s;  // Default: 1e-4: Armijo sufficient descent parameter
    float gamma;  // Default: 0.5:  Backtracking search contraction parameter
    uint64_t max_iterations;
    float jac_2norm_term;  // Default 1e-4;
    float delta_x_2norm_term;  // Default 1e-5;
    float delta_f_term;  // Default 1e-5;
    
    // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
    uint32_t num_coeffs_;
    Eigen::MatrixXf cur_x_;
    Eigen::MatrixXf new_x_;
    Eigen::MatrixXf delta_x_;
    Eigen::MatrixXf cur_J_;  // Jacobian
    Eigen::MatrixXf cur_J_transpose_;  // Jacobian transpose
    Eigen::MatrixXf cur_p_;  // Search direction
    float cur_f_;
    float new_f_;
    const bool* angle_coeff_;  // Now owned here

    ObjectiveFuncPtr obj_func_;
    CoeffUpdateFuncPtr coeff_update_func_;
    CoeffPreturbFuncPtr coeff_preturb_func_;
    JacobianFuncPtr jac_func_;

    // ret = a + interp_val * (b - c)
    void interpolateCoeff(Eigen::MatrixXf& ret, const Eigen::MatrixXf& a,
      const float interp_val, const Eigen::MatrixXf& b, 
      const Eigen::MatrixXf& c);

    // ret = a + interp_val * b
    void interpolateCoeff(Eigen::MatrixXf& ret, const Eigen::MatrixXf& a,
      const float interp_val, const Eigen::MatrixXf& b);
  };

};  // namespace math
};  // namespace jtil

#endif  // SDB_VERBOSE_SOLVER
