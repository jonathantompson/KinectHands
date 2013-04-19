#include <random>
#include <stdexcept>
#include <iostream>
#include "math/sdb_fitting.h"
#include "Eigen"

using Eigen::MatrixXf;
using std::cout;
using std::endl;
using std::runtime_error;

namespace jtil {
namespace math {
  SDBFitting::SDBFitting(uint32_t num_coeffs) {
    num_coeffs_ = num_coeffs;
    cur_x_.resize(num_coeffs_, 1);
    new_x_.resize(num_coeffs_, 1);
    delta_x_.resize(num_coeffs_, 1);
    cur_J_.resize(num_coeffs_, 1);
    cur_J_transpose_.resize(1, num_coeffs_);
    cur_p_.resize(num_coeffs_, 1);

    // Some default parameters
    max_iterations = 1000;
    eta_s = 1e-4f;
    jac_2norm_term = 1e-4f;
    gamma = 0.5f;
    delta_x_2norm_term = 1e-5f;
    delta_f_term = 1e-5f;
  }

  SDBFitting::~SDBFitting() {

  }

  MatrixXf& SDBFitting::fitModel(const MatrixXf& start_c,
                                const bool* angle_coeff,
                                ObjectiveFuncPtr obj_func,
                                JacobianFuncPtr jac_func,
                                CoeffUpdateFuncPtr coeff_update_func) {
    obj_func_ = obj_func;
    jac_func_ = jac_func;
    coeff_update_func_ = coeff_update_func;
    angle_coeff_ = angle_coeff;

    cout << "Starting Steepest-Descent with backtracking optimization..." << endl;

    if (start_c.rows() == 1) {
      cur_x_ = start_c.transpose();  // I want a row vector here
    } else {
      cur_x_ = start_c;
    }
    cur_f_ = obj_func(cur_x_);

    uint32_t no_iterations = 0;
    while (no_iterations < max_iterations) {
#ifdef SDB_VERBOSE_SOLVER
      cout << "******************************************************" << endl;
      cout << "Iteration: " << no_iterations << endl;
      //cout << "   cur_x_^T = ";
      //for (uint32_t i = 0; i < num_coeffs_; i++) { cout << cur_x_(i) << " "; }
      //cout << endl;
      cout << "   cur_f_ = " << cur_f_ << endl;
#endif

      jac_func_(cur_J_, cur_x_);
      float jac_2norm_ = cur_J_.norm();
#ifdef SDB_VERBOSE_SOLVER
      //cout << "   cur_J_^T = ";
      //for (uint32_t i = 0; i < num_coeffs_; i++) { cout << cur_J_(i) << " "; }
      //cout << endl;
      cout << "   ||J||_2 = " << jac_2norm_ << endl;
#endif
      if (jac_2norm_ < jac_2norm_term) {
#ifdef SDB_VERBOSE_SOLVER
        cout << "   jac_2norm_ < jac_2norm_term" << endl;
#endif
        break;
      }

      cur_p_ = -cur_J_;  // search direction is the steepest descent direction
      cur_J_transpose_ = cur_J_.transpose();

      // Otherwise perform a backtracking line search:
      float alpha = 1;
      uint32_t num_alpha_iterations = 0;
      interpolateCoeff(new_x_, cur_x_, alpha, cur_p_);
      new_f_ = obj_func_(new_x_);

      MatrixXf cur_J_tran_p = cur_J_transpose_ * cur_p_;  // avoid redundant calc
      while (new_f_ > (cur_f_ + eta_s * alpha * cur_J_tran_p(0)) && 
        num_alpha_iterations < 100) {
        // Armijo sufficient decrease condition not met:  contract the step
        alpha *= gamma;
        interpolateCoeff(new_x_, cur_x_, alpha, cur_p_);
        new_f_ = obj_func_(new_x_);
        num_alpha_iterations++;
      }

      if (num_alpha_iterations > 100) {
#ifdef SDB_VERBOSE_SOLVER
        cout << "   num_alpha_iterations > 100" << endl;
#endif
        break;
      }

#ifdef SDB_VERBOSE_SOLVER
      cout << "   alpha = " << alpha << endl;
#endif

      // Take the step:
      delta_x_ = cur_x_ - new_x_;
      float delta_x_2norm = delta_x_.norm();
      float delta_f = cur_f_ - new_f_;  // Guaranteed positive

#ifdef SDB_VERBOSE_SOLVER
      cout << "   ||DeltaX||_2 = " << delta_x_2norm << endl;
      cout << "   |delta_f| = " << delta_f << endl;
#endif
      cur_x_ = new_x_;
      cur_f_ = new_f_;

      if (delta_x_2norm < delta_x_2norm_term) {
#ifdef SDB_VERBOSE_SOLVER
        cout << "   delta_x_2norm < delta_x_2norm_term" << endl;
#endif
        break;
      }
      if (delta_f < delta_f_term) {
#ifdef SDB_VERBOSE_SOLVER
        cout << "   delta_f < delta_f_term" << endl;
#endif
        break;
      }
      no_iterations++;
    }
    std::cout << "Steepest Descent with backtracking finished with f = " << cur_f_ << endl;

    if (start_c.rows() == 1) {
      cur_x_.transposeInPlace();
    }
    return cur_x_;
  }
  
  // interpolateCoeff performs the following:
  // ret = a + interp_val * (b - c)
  void SDBFitting::interpolateCoeff(Eigen::MatrixXf& ret, const Eigen::MatrixXf& a,
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
  void SDBFitting::interpolateCoeff(Eigen::MatrixXf& ret, const Eigen::MatrixXf& a,
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