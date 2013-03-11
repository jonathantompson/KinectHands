#include <random>
#include <stdexcept>
#include <iostream>
#include "math/bfgs_fitting.h"
#include "Eigen"

using Eigen::MatrixXf;
using std::cout;
using std::endl;
using std::runtime_error;

namespace jtil {
namespace math {
  BFGSFitting::BFGSFitting(uint32_t num_coeffs) {
    num_coeffs_ = num_coeffs;

    x_k_.resize(num_coeffs_, 1);
    x_k_p1_.resize(num_coeffs_, 1);
    B_inv_k_.resize(num_coeffs_, num_coeffs_);
    B_inv_k_p1_.resize(num_coeffs_, num_coeffs_);
    J_k_.resize(num_coeffs_, 1);
    J_k_p1_.resize(num_coeffs_, 1);
    y_k_.resize(num_coeffs_, 1);
    p_k_.resize(num_coeffs_, 1);
    s_k_.resize(num_coeffs_, 1);

    J_k_tran_.resize(1, num_coeffs_);
    J_k_tran_p_k_.resize(1, 1);
    s_k_tran_.resize(1, num_coeffs_);
    y_k_tran_.resize(1, num_coeffs_);
    s_k_tran_y_k_.resize(1, 1);
    s_k_y_k_tran_.resize(num_coeffs_, num_coeffs_);
    s_k_s_k_tran_.resize(num_coeffs_, num_coeffs_);
    y_k_s_k_tran_.resize(num_coeffs_, num_coeffs_);
    y_k_tran_B_inv_k_.resize(1, num_coeffs_);

    // Some default parameters
    max_iterations = 1000;
    eta_s = 1e-4f;
    jac_2norm_term = 1e-4f;
    gamma = 0.5f;
    delta_x_2norm_term = 1e-5f;
    delta_f_term = 1e-5f;
  }

  BFGSFitting::~BFGSFitting() {

  }

  MatrixXf& BFGSFitting::fitModel(const MatrixXf& start_c,
                                const bool* angle_coeff,
                                ObjectiveFuncPtr obj_func,
                                JacobianFuncPtr jac_func,
                                CoeffUpdateFuncPtr coeff_update_func) {
    obj_func_ = obj_func;
    jac_func_ = jac_func;
    coeff_update_func_ = coeff_update_func;
    angle_coeff_ = angle_coeff;

    cout << "Starting BFGS with backtracking optimization..." << endl;

    if (start_c.rows() == 1) {
      x_k_ = start_c.transpose();  // I want a row vector here
    } else {
      x_k_ = start_c;
    }
    f_k_ = obj_func(x_k_);
    B_inv_k_.setZero();
    for (uint32_t i = 0; i < num_coeffs_; i++) {
      B_inv_k_(i,i) = 1.0f;
    }
    jac_func_(J_k_, x_k_);

    uint32_t no_iterations = 0;
    while (no_iterations < max_iterations) {
#ifdef BFGS_VERBOSE_SOLVER
      cout << "******************************************************" << endl;
      cout << "Iteration: " << no_iterations << endl;
      //cout << "   cur_x_^T = ";
      //for (uint32_t i = 0; i < num_coeffs_; i++) { cout << cur_x_(i) << " "; }
      //cout << endl;
      cout << "   cur_f_ = " << f_k_ << endl;
#endif

      J_k_tran_ = J_k_.transpose();
      float jac_2norm_ = J_k_.norm();
#ifdef BFGS_VERBOSE_SOLVER
      //cout << "   cur_J_^T = ";
      //for (uint32_t i = 0; i < num_coeffs_; i++) { cout << cur_J_(i) << " "; }
      //cout << endl;
      cout << "   ||J||_2 = " << jac_2norm_ << endl;
#endif
      if (jac_2norm_ < jac_2norm_term) {
#ifdef BFGS_VERBOSE_SOLVER
        cout << "   jac_2norm_ < jac_2norm_term" << endl;
#endif
        break;
      }

      p_k_ = -B_inv_k_ * J_k_;  // Search direction satisfies H_k_ * p_k_ = -J_k_

      // Otherwise perform a backtracking line search:
      alpha_k_ = 1;
      uint32_t num_alpha_iterations = 0;
      interpolateCoeff(x_k_p1_, x_k_, alpha_k_, p_k_);
      f_k_p1_ = obj_func_(x_k_p1_);

      J_k_tran_p_k_ = J_k_tran_ * p_k_;  // avoid redundant calc
      while (f_k_p1_ > (f_k_ + eta_s * alpha_k_ * J_k_tran_p_k_(0)) && 
        num_alpha_iterations < 100) {
        // Armijo sufficient decrease condition not met: contract the step
        alpha_k_ *= gamma;
        interpolateCoeff(x_k_p1_, x_k_, alpha_k_, p_k_);
        f_k_p1_ = obj_func_(x_k_p1_);
        num_alpha_iterations++;
      }

      if (num_alpha_iterations > 100) {
#ifdef BFGS_VERBOSE_SOLVER
        cout << "   num_alpha_iterations > 100" << endl;
#endif
        break;
      }

#ifdef BFGS_VERBOSE_SOLVER
      cout << "   alpha = " << alpha_k_ << endl;
#endif

      // Take the step:
      s_k_ = x_k_p1_ - x_k_;  // s_k_= alpha_k_ * p_k_ = x_k_ + alpha_k_ * p_k_ - x_k = x_k_p1_ - x_k_
      s_k_tran_ = s_k_.transpose();
      float delta_x_2norm = s_k_.norm();
      float delta_f = f_k_ - f_k_p1_;  // Guaranteed positive

#ifdef BFGS_VERBOSE_SOLVER
      cout << "   ||DeltaX||_2 = " << delta_x_2norm << endl;
      cout << "   |delta_f| = " << delta_f << endl;
#endif
      if (delta_x_2norm < delta_x_2norm_term) {
#ifdef BFGS_VERBOSE_SOLVER
        cout << "   delta_x_2norm < delta_x_2norm_term" << endl;
#endif
        break;
      }
      if (delta_f < delta_f_term) {
#ifdef BFGS_VERBOSE_SOLVER
        cout << "   delta_f < delta_f_term" << endl;
#endif
        break;
      }

      // Update the new Jacobian
      jac_func_(J_k_p1_, x_k_p1_);
      y_k_ = J_k_p1_ - J_k_;
      y_k_tran_ = y_k_.transpose();

      // Update the Hessian inverse
      // http://en.wikipedia.org/wiki/BFGS_method
      s_k_tran_y_k_ = s_k_tran_ * y_k_;
      s_k_y_k_tran_ = s_k_ * y_k_tran_;
      s_k_s_k_tran_ = s_k_ * s_k_tran_;
      y_k_s_k_tran_ = y_k_ * s_k_tran_;
      y_k_tran_B_inv_k_ = y_k_tran_ * B_inv_k_;

      //B_inv_k_p1_ = B_inv_k_ 
      //  + ((s_k_tran_y_k_ + y_k_tran_B_inv_k_ * y_k_) * s_k_s_k_tran_) / (s_k_tran_y_k_(0) * s_k_tran_y_k_(0))
      //  - (B_inv_k_ * y_k_s_k_tran_ + s_k_y_k_tran_ * B_inv_k_) / (s_k_tran_y_k_(0));

      tmp_scalar1_ = (s_k_tran_y_k_ + y_k_tran_B_inv_k_ * y_k_) / (s_k_tran_y_k_(0)*s_k_tran_y_k_(0));
      B_inv_k_p1_ = B_inv_k_ + (tmp_scalar1_(0) * s_k_s_k_tran_)
        - (B_inv_k_ * y_k_s_k_tran_ + s_k_y_k_tran_ * B_inv_k_) / (s_k_tran_y_k_(0));

      
      // Get ready for the next iteration
      x_k_ = x_k_p1_;
      f_k_ = f_k_p1_;
      J_k_ = J_k_p1_;
      B_inv_k_ = B_inv_k_p1_;

      no_iterations++;
    }
    std::cout << "BFGS with backtracking finished with f = " << f_k_;
    std::cout << "(" << no_iterations << " iterations)" << endl;

    if (start_c.rows() == 1) {
      x_k_.transposeInPlace();
    }
    return x_k_;
  }
  
  // interpolateCoeff performs the following:
  // ret = a + interp_val * (b - c)
  void BFGSFitting::interpolateCoeff(Eigen::MatrixXf& ret, const Eigen::MatrixXf& a,
    const float interp_val, const Eigen::MatrixXf& b, const Eigen::MatrixXf& c) {
    for (uint32_t i = 0; i < num_coeffs_; i++) {
      if (angle_coeff_ != NULL && angle_coeff_[i]) {
        float real_a = cos(a(i));
        float imag_a = sin(a(i));
        float real_b = cos(b(i));
        float imag_b = sin(b(i));
        float real_c = cos(c(i));
        float imag_c = sin(c(i));
        float real_interp = real_a + interp_val * (real_b - real_c);
        float imag_interp = imag_a + interp_val * (imag_b - imag_c);
        float interp_angle = atan2(imag_interp, real_interp);
        ret(i) = interp_angle;
      } else {
        ret(i) = a(i) + interp_val * (b(i) - c(i));
      }
    }
    if (coeff_update_func_) {
      coeff_update_func_(ret);
    }
  }

  // interpolateCoeff performs the following:
  // ret = a + interp_val * b
  void BFGSFitting::interpolateCoeff(Eigen::MatrixXf& ret, const Eigen::MatrixXf& a,
    const float interp_val, const Eigen::MatrixXf& b) {
    for (uint32_t i = 0; i < num_coeffs_; i++) {
      if (angle_coeff_ != NULL && angle_coeff_[i]) {
        float real_a = cos(a(i));
        float imag_a = sin(a(i));
        float real_b = cos(b(i));
        float imag_b = sin(b(i));
        float real_interp = real_a + interp_val * real_b;
        float imag_interp = imag_a + interp_val * imag_b;
        float interp_angle = atan2(imag_interp, real_interp);
        ret(i) = interp_angle;
      } else {
        ret(i) = a(i) + interp_val * b(i);
      }
    }
    if (coeff_update_func_) {
      coeff_update_func_(ret);
    }
  }

}  // namespace math
}  // namespace jtil
