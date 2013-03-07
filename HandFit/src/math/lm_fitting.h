//
//  lm_fitting.h
//
//  Created by Jonathan Tompson on 8/23/12.
//
//  A black box implementation of the Levenberg-Marquardt non-linear
//  optimization algorithm.  Requires a function pointer to Jacobian matrix
//  calculator.  This can be estimated using the mid-point method if the
//  function is non-differentiable.
//
//  This class and its supporting functions takes in a set of data points y,
//  evaluated at points x and will perform a non-linear fit for the user 
//  supplied function f(x,c) by modifying the function coefficients c.  The 
//  user supplied residual function is used to evaluate the "quality" of the 
//  fit (otherwise the L2 norm of the error is used if no residual function is
//  supplied).
//

#ifndef MATH_LM_FITTING_HEADER
#define MATH_LM_FITTING_HEADER

#include "jtil/math/math_types.h"
#include "math/common_fitting.h"
#include "Eigen"

// #define LM_VERBOSE_SOLVER  // Print out per-iteration information

namespace jtil {
namespace math {

  // Levenberg-Marquardt optimization
  class LMFitting {
  public:
    LMFitting(uint32_t num_coeffs, uint32_t data_length);
    ~LMFitting();

    // fitModel = Top level function:
    // y --> Data to fit
    // x --> X values for the data points y(x)
    // start_c --> Starting value for fitting coefficient c
    // eval_func --> Function pointer to function to fit: takes in a set of 
    //               coefficients and a set of x evaluation points and returns
    //               f(x,c).
    // jacobian_func --> Function pointer to jacobian calculation: should return
    //                   J_ji = df(x_j,c) / dc_i.   That is for each element x_j
    //                   it needs to calculate the parital deriviative wrt c_i.
    // residue_func --> Function to calculate residue (error) per iteration.
    //                  set to NULL to let LMFitting use the L2 norm of error as 
    //                  residue.
    // coeff_norm_func --> After coefficient update, some coefficients may need
    //                     normalization (ie, rotation quaternion coeffs).  Set
    //                     to NULL if not needed.
    // coeff_preturb_func --> num_restarts != 0, then you need to define a
    //                        function to randomly preturb the coefficients
    //                        before restarting the optimization.  Otherwise set
    //                        to NULL;
    Eigen::MatrixXf& fitModel(const Eigen::MatrixXf& y,  // <data_length, 1>
                              const Eigen::MatrixXf& x,  // <data_length, 1>
                              const Eigen::MatrixXf& start_c,  // <1, num_coeffs>
                              EvalFuncPtr eval_func,
                              LSJacobianFuncPtr jacobian_func,
                              ResidueFuncPtr residue_func,  // can be NULL
                              CoeffUpdateFuncPtr coeff_update_func,  // can be NULL
                              CoeffPreturbFuncPtr coeff_preturb_func);  // can be NULL

    // Termination and Optimization settings:
    float delta_c_termination;  // L2_norm of delta C (coefficients) to stop optimization at
    float delta_residue_termination;  // delta residue to stop optimization at
    float lambda_start;  // Starting lambda value
    float lambda_min;  // minimum lambda value
    float lambda_max;  // maximum lambda value
    uint32_t num_restarts;  // Convergence may improve if the coefficients are
                            // preturbed and LM optimization is repeated.
    uint32_t max_iterations;  // Maximum number of LM iterations

    // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
    uint32_t num_coeffs_;
    uint32_t data_length_;
    Eigen::MatrixXf jacobian_k_;
    Eigen::MatrixXf jacobian_k_tran_;
    Eigen::MatrixXf delta_c_k_p1_;
    Eigen::MatrixXf delta_c_k_p2_;
    Eigen::MatrixXf normal_mat_;
    Eigen::MatrixXf cur_normal_mat_;
    Eigen::MatrixXf coeff_k_;
    Eigen::MatrixXf best_coeff_;
    Eigen::MatrixXf coeff_tmp_;
    Eigen::MatrixXf coeff_k_p1_;
    Eigen::MatrixXf coeff_k_p2_;
    Eigen::MatrixXf f_k_;
    Eigen::MatrixXf f_k_p1_;
    Eigen::MatrixXf f_k_p2_;
    Eigen::MatrixXf f_k_partial_;
    Eigen::MatrixXf delta_y_;
    Eigen::MatrixXf delta_y_prime_;
    Eigen::MatrixXf delta_y_p12_;
    Eigen::MatrixXf residual_;

    EvalFuncPtr eval_func_;
    LSJacobianFuncPtr jacobian_func_;
    ResidueFuncPtr residue_func_;
    CoeffUpdateFuncPtr coeff_update_func_;
    CoeffPreturbFuncPtr coeff_preturb_func_;
  };

};  // namespace math
};  // namespace jtil

#endif  // MATH_LM_FITTING_HEADER
