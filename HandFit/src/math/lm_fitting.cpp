//
//  lm_fitting.cpp
//
//  Created by Jonathan Tompson on 8/23/12.
//
//  A black box implementation of the Levenberg-Marquardt non-linear
//  optimization algorithm.  Requires a function pointer to Jacobian matrix
//  calculator.  This can be estimated using the mid-point method if the
//  function is non-differentiable.
//

#include <stdexcept>
#include <iostream>
#include "math/lm_fitting.h"
#include "Eigen"

using Eigen::MatrixXf;
using std::cout;
using std::endl;
using std::runtime_error;

namespace math {

  LMFitting::LMFitting(uint32_t num_coeffs, uint32_t data_length) {
    jacobian_k_.resize(data_length, num_coeffs);
    jacobian_k_tran_.resize(num_coeffs, data_length);
    delta_c_k_p1_.resize(1, num_coeffs);
    delta_c_k_p2_.resize(1, num_coeffs);
    normal_mat_.resize(num_coeffs, num_coeffs);
    cur_normal_mat_.resize(num_coeffs, num_coeffs);
    coeff_k_.resize(1, num_coeffs);
    coeff_k_p1_.resize(1, num_coeffs);
    coeff_k_p2_.resize(1, num_coeffs);
    f_k_.resize(data_length, 1);
    f_k_p1_.resize(data_length, 1);
    f_k_p2_.resize(data_length, 1);
    delta_y_.resize(data_length, 1);
    delta_y_p12_.resize(data_length, 1);
    delta_y_prime_.resize(num_coeffs, 1);
    residual_.resize(data_length, 1);

    num_coeffs_ = num_coeffs;
    data_length_ = data_length;

    // Some default parameters
    delta_c_termination = 1e-10f;
    delta_residue_termination = 1e-10f;
    lambda_start = 1e-6f;
    lambda_min = 1e-12f;
    lambda_max = 1e12f;
    num_restarts = 1;
    max_iterations = 1000;
  }

  LMFitting::~LMFitting() {
    // Empty
  }


  MatrixXf& LMFitting::fitModel(const MatrixXf& y,
                                const MatrixXf& x,
                                const MatrixXf& start_c,
                                EvalFuncPtr eval_func,
                                LSJacobianFuncPtr jacobian_func,
                                ResidueFuncPtr residue_func,
                                CoeffUpdateFuncPtr coeff_update_func,
                                CoeffPreturbFuncPtr coeff_preturb_func) {
    eval_func_ = eval_func;
    jacobian_func_ = jacobian_func;
    residue_func_ = residue_func;
    coeff_update_func_ = coeff_update_func;
    coeff_preturb_func_ = coeff_preturb_func;

    coeff_k_ = start_c;
    best_coeff_ = coeff_k_;
    float r_k;
    float best_r_k;

    for (uint32_t repeats = 0; repeats <= num_restarts; repeats++) {
      if (coeff_update_func_ != NULL) {  // Just in case make sure coeff_k_ is valid
        coeff_update_func_(coeff_k_);
      }
      uint32_t iteration_num = 1;
      float norm_delta_c_k = std::numeric_limits<float>::infinity();
      float delta_r_k = std::numeric_limits<float>::infinity();
      float lambda_k = lambda_start;

      // Calculate the starting residual and Jacobian
      eval_func_(f_k_, coeff_k_, x);
      jacobian_func_(jacobian_k_, coeff_k_, x);
      jacobian_k_tran_ = jacobian_k_.transpose();
      normal_mat_ = jacobian_k_tran_ * jacobian_k_;
      delta_y_ = y - f_k_;
      delta_y_prime_ = jacobian_k_tran_ * delta_y_;  // RHS 'b' of 'Ax = b'
      if (residue_func != NULL) {
        r_k = residue_func_(y, f_k_, coeff_k_);
      } else {
        MatrixXf r_k_mat = delta_y_.transpose() * delta_y_;   // the dot product
        r_k = r_k_mat(1); 
      }

      if (repeats == 0) {
        best_r_k = r_k;
        best_coeff_ = coeff_k_;
      }

      // Main optimization loop
      do {
#ifdef LM_VERBOSE_SOLVER
        if (iteration_num != 1) {
          // Print the results of the last iteration
          cout << "Iteration " << iteration_num - 1 << endl;
          cout << "  --> 2-norm of delta_c is " << norm_delta_c_k << endl;
          cout << "  --> lambda_k is " << lambda_k << endl;
          cout << "  --> r_k is " << r_k << endl;
          cout << "  --> delta_r_k is " << delta_r_k << endl;
        }
#endif

        // Solve the 1st linear system (for the current lambda)
        cur_normal_mat_ = normal_mat_;
        for (uint32_t i = 0; i < num_coeffs_; i++) {
          // Add the Levenberg Marquardt dapening factor (down the diagonals)
          cur_normal_mat_(i, i) = cur_normal_mat_(i, i) + (lambda_k * cur_normal_mat_(i, i));
        }
        delta_c_k_p1_ = cur_normal_mat_.householderQr().solve(delta_y_prime_);
        if(!delta_y_prime_.isApprox(cur_normal_mat_ * delta_c_k_p1_)) {
          // Matrix might not be positive definite 
          // --> Either way, cannot perform cholesky factorization.
          lambda_k = lambda_k * 2;
          iteration_num++;
          continue;  // Quit this iteration and start again
        }

        // Compute the new coefficients and it's residual
        coeff_k_p1_ = coeff_k_ + delta_c_k_p1_.transpose();
        if (coeff_update_func_ != NULL) {
          coeff_update_func_(coeff_k_p1_);
        }
        eval_func_(f_k_p1_, coeff_k_p1_, x);
        float r_k_p1;
        if (residue_func != NULL) {
          r_k_p1 = residue_func_(y, f_k_p1_, coeff_k_p1_);
        } else {
          delta_y_p12_ = y - f_k_p1_;
          MatrixXf r_k_p1_mat = delta_y_p12_.transpose() * delta_y_p12_;
          r_k_p1 = r_k_p1_mat(1);
        }

        // Solve the 2nd linear system (for decreased current lambda)
        cur_normal_mat_ = normal_mat_;
        for (uint32_t i = 0; i < num_coeffs_; i++) {
          // Add the Levenberg Marquardt dapening factor (down the diagonals)
          cur_normal_mat_(i, i) = cur_normal_mat_(i, i) + (0.5f * lambda_k * cur_normal_mat_(i, i));
        }
        delta_c_k_p2_ = cur_normal_mat_.fullPivHouseholderQr().solve(delta_y_prime_);
        if(!delta_y_prime_.isApprox(cur_normal_mat_ * delta_c_k_p2_)) {
          // Matrix might not be positive definite 
          // --> Either way, cannot perform cholesky factorization.
          lambda_k = lambda_k * 2;
          iteration_num++;
          continue;  // Quit this iteration and start again
        }

        // Compute the new coefficients and it's residual
        coeff_k_p2_ = coeff_k_ + delta_c_k_p2_.transpose();
        if (coeff_update_func_ != NULL) {
          coeff_update_func_(coeff_k_p2_);
        }
        eval_func_(f_k_p2_, coeff_k_p2_, x);
        float r_k_p2;
        if (residue_func != NULL) {
          r_k_p2 = residue_func_(y, f_k_p2_, coeff_k_p2_);
        } else {
          delta_y_p12_ = y - f_k_p2_;
          MatrixXf r_k_p2_mat = delta_y_p12_.transpose() * delta_y_p12_;
          r_k_p2 = r_k_p2_mat(1);
        }

        // NOW WE HAVE 3 OPTIONS WITH 3 RESIDUALS:
        // 1. We couldn't make progress (r_k get's worse, so we need a higher
        //    dampening factor).
        // 2. The current dampening factor is perfect
        // 3. We made progress and we can reduce the dampening factor (for faster
        //    convergence on the next iteration).
        int residual_case;
        if (r_k <= r_k_p1) {
          if (r_k <= r_k_p2) {
            residual_case = 0;
          } else {
            residual_case = 2;
          }
        } else {
          if (r_k_p1 < r_k_p2) {
            residual_case = 1;
          } else {
            residual_case = 2;
          }
        }

        switch (residual_case) {
        case 0:
          // Stepping increased the residual, double the dampening factor and
          // try again without updating coefficients
          lambda_k = lambda_k * 2;
          break;
        case 1:
          lambda_k = lambda_k;
          delta_r_k = r_k - r_k_p1;
          r_k = r_k_p1;
          coeff_k_ = coeff_k_p1_;
          f_k_ = f_k_p1_;
          norm_delta_c_k = delta_c_k_p1_.lpNorm<2>();
          break;
        case 2:
          lambda_k = lambda_k * 0.5f;
          delta_r_k = r_k - r_k_p2;
          r_k = r_k_p2;
          coeff_k_ = coeff_k_p2_;
          f_k_ = f_k_p2_;
          norm_delta_c_k = delta_c_k_p2_.lpNorm<2>();
          break;
        }

        if (residual_case != 0) {  // We made some progress --> Update the jacobian
          jacobian_func_(jacobian_k_, coeff_k_, x);
          jacobian_k_tran_ = jacobian_k_.transpose();
          normal_mat_ = jacobian_k_tran_ * jacobian_k_;
          delta_y_ = y - f_k_;
          delta_y_prime_ = jacobian_k_tran_ * delta_y_;  // RHS 'b' of 'Ax = b'
        }

        if (lambda_k < lambda_min) {
          lambda_k = lambda_min;
        }

        iteration_num++;
      } while (norm_delta_c_k > delta_c_termination &&
        delta_r_k > delta_residue_termination && 
        iteration_num < max_iterations && 
        lambda_k < lambda_max);

      if (r_k < best_r_k) {
        best_coeff_ = coeff_k_;
        best_r_k = r_k;
      }
        
      if (coeff_preturb_func_ != NULL) {
        coeff_k_ = best_coeff_;
        coeff_preturb_func_(coeff_k_);  // Preturb before restarting
      }
    }  // for (num_restarts)

    return best_coeff_;
  }

};  // namespace math
