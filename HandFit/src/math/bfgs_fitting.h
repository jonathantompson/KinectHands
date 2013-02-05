//
//  bfgs_fitting.h
//
//  Created by Jonathan Tompson on 11/20/12.
//
//  A black box implementation of the BFGS with linear-search
//  backtracking non-linear optimization algorithm.  Requires Jacobian function
//  (which you can estimate using central differencing).
//
//  http://en.wikipedia.org/wiki/BFGS_method
//

#ifndef MATH_BFGS_FITTING_HEADER
#define MATH_BFGS_FITTING_HEADER

#include <random>
#include "math/math_types.h"
#include "math/common_fitting.h"
#include "Eigen"

#if defined(WIN32) || defined(_WIN32)
  #define constexpr 
#endif

// #define BFGS_VERBOSE_SOLVER  // Print out per-iteration information

namespace math {

  // BFGS with backtracking optimization
  class BFGSFitting {
  public:
    BFGSFitting(uint32_t num_coeffs);
    ~BFGSFitting();

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
    float alpha_k_;
    Eigen::MatrixXf x_k_;
    Eigen::MatrixXf x_k_p1_;
    Eigen::MatrixXf B_inv_k_;  // Approximate Inverse Hessian
    Eigen::MatrixXf B_inv_k_p1_;
    Eigen::MatrixXf J_k_;  // Jacobian
    Eigen::MatrixXf J_k_p1_;
    Eigen::MatrixXf y_k_;  // J_k_p1_ - J_k_
    Eigen::MatrixXf p_k_;  // Search direction
    Eigen::MatrixXf s_k_;  // alpha_k_ * p_k_
    float f_k_;
    float f_k_p1_;
    const bool* angle_coeff_;  // Now owned here

    // Temp vectors
    Eigen::MatrixXf J_k_tran_;
    Eigen::MatrixXf J_k_tran_p_k_;
    Eigen::MatrixXf s_k_tran_;
    Eigen::MatrixXf y_k_tran_;
    Eigen::MatrixXf s_k_tran_y_k_;
    Eigen::MatrixXf s_k_y_k_tran_;
    Eigen::MatrixXf s_k_s_k_tran_;
    Eigen::MatrixXf y_k_s_k_tran_;
    Eigen::MatrixXf y_k_tran_B_inv_k_;
    Eigen::MatrixXf tmp_scalar1_;

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

#endif  // MATH_BFGS_FITTING_HEADER
