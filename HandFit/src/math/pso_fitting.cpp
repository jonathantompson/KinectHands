#include <random>
#include <stdexcept>
#include <iostream>
#include "math/pso_fitting.h"
#include "Eigen"

using Eigen::MatrixXf;
using std::cout;
using std::endl;
using std::runtime_error;

namespace jtil {
namespace math {

  MERSINE_TWISTER_ENG PSOFitting::eng;
  UNIFORM_REAL_DISTRIBUTION PSOFitting::dist_real(0, 1);

  PSOFitting::PSOFitting(uint32_t num_coeffs, int swarm_size) {
    num_coeffs_ = num_coeffs;
    if (swarm_size > 0) {
      swarm_size_ = static_cast<uint32_t>(swarm_size);
    } else {
      swarm_size_ = 30;  // Recommended by "An Off-The-Shelf PSO"
    }

    best_pos_global_.resize(1, num_coeffs_);
    c_lo_.resize(1, num_coeffs_);
    c_hi_.resize(1, num_coeffs_);
    cur_c_min_.resize(1, num_coeffs_);
    cur_c_max_.resize(1, num_coeffs_);
    delta_c_.resize(1, num_coeffs_);
    vel_max_.resize(1, num_coeffs_);

    // Allocate space for the N+1 probe points
    swarm_ = new SwarmNode[swarm_size_];
    for (uint32_t i = 0; i < swarm_size_; i++) {
      swarm_[i].resize(num_coeffs_);
    }

    // Some default parameters
    max_iterations = 1000;
    delta_coeff_termination = 1e-8f;
    phi_p = 2.8f;  // Recommended by "An Off-The-Shelf PSO"
    phi_g = 1.3f;  // Recommended by "An Off-The-Shelf PSO"
  }

  PSOFitting::~PSOFitting() {
    delete[] swarm_;
  }

  // Lots of good info here: http://www.scholarpedia.org/article/Nelder-Mead_algorithm
  // and some more here: http://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method
  MatrixXf& PSOFitting::fitModel(const Eigen::MatrixXf& start_c,
                                 const Eigen::MatrixXf& radius_c,
                                 const bool* angle_coeff,
                                 ObjectiveFuncPtr obj_func,
                                 CoeffUpdateFuncPtr coeff_update_func) {
    eng.seed();
    obj_func_ = obj_func;
    coeff_update_func_ = coeff_update_func;
    angle_coeff_ = angle_coeff;

    cout << "Starting PSO optimization..." << endl;

    // Initialize all random swarm particles to Uniform(c_lo_, c_hi_)
    radius_c.cwiseAbs();
    c_lo_ = start_c - radius_c;
    c_hi_ = start_c + radius_c;
    vel_max_ = radius_c;  // According to paper (forgot title), 0.5x search space
                          // is best for multi-modal distributions.
    for (uint32_t j = 0; j < num_coeffs_; j++) {
      UNIFORM_REAL_DISTRIBUTION c_dist(c_lo_(j), c_hi_(j));
      for (uint32_t i = 0; i < swarm_size_; i++) {
        float uniform_rand_num = c_dist(eng);  // [c_lo_, c_hi_)
        swarm_[i].pos(j) = uniform_rand_num;
      }
    }

    // evaluate the agent's function values, calculate residue and set the best
    // position and residue for the particle x_i as it's starting position
    best_residue_global_ = std::numeric_limits<float>::infinity();
    for (uint32_t i = 0; i < swarm_size_; i++) {
      if (coeff_update_func_) {
        coeff_update_func_(swarm_[i].pos);
      }
      swarm_[i].residue = obj_func_(swarm_[i].pos);
      swarm_[i].best_residue = swarm_[i].residue;
      swarm_[i].best_pos = swarm_[i].pos;
      if (swarm_[i].residue < best_residue_global_) {
        best_residue_global_ = swarm_[i].residue;
        best_pos_global_ = swarm_[i].pos;
      }
    }

    // Initialize random velocity to Uniform(-2*radius_c, 2*radius_c)
    for (uint32_t j = 0; j < num_coeffs_; j++) {
      UNIFORM_REAL_DISTRIBUTION c_dist(-2*radius_c(j),
        2*radius_c(j));
      for (uint32_t i = 0; i < swarm_size_; i++) {
        float uniform_rand_num = c_dist(eng);  // [-2*radius_c, 2*radius_c)
        swarm_[i].vel(j) = uniform_rand_num;
      }
    }

    float phi = phi_p + phi_g;
    if (phi <= 4) {
      throw std::runtime_error("ERROR: kappa_ = phi_p + phi_g <= 4!");
    }
    kappa_ = 2.0f / fabsf(2.0f - phi - sqrtf(phi * phi - 4 * phi));

#ifdef PSO_VERBOSE_SOLVER
    cout << "Iteration 0:" << endl;
    cout << "  --> min residue of population = " << best_residue_global_ << endl;
    cout << "  --> Agent residues: <";
    for (uint32_t i = 0; i < swarm_size_; i++) {
      cout << swarm_[i].residue;
      if (i != swarm_size_ - 1) { cout << ", "; }
    }
    cout << ">" << endl;
#endif

    uint64_t num_iterations = 0;
    float delta_coeff = std::numeric_limits<float>::infinity();
    do {
      // For each particle, i in the swarm:
      for (uint32_t i = 0; i < swarm_size_; i++) {
        SwarmNode* cur_node = &swarm_[i];
        // For each dimension d:
        for (uint32_t d = 0; d < num_coeffs_; d++) {
          float r_p = dist_real(eng);  // [0,1)
          float r_g = dist_real(eng);  // [0,1)
          // Update the velocity
          cur_node->vel(d) = kappa_ * (cur_node->vel(d) + 
            (phi_p * r_p * (cur_node->best_pos(d) - cur_node->pos(d))) + 
            (phi_g * r_g * (best_pos_global_(d) - cur_node->pos(d))));
          // Limit the velocity as discussed in the paper "An Off-The-Shelf PSO"
          if (cur_node->vel(d) > vel_max_(d)) {
            cur_node->vel(d) = vel_max_(d);
          }
          if (cur_node->vel(d) < -vel_max_(d)) {
            cur_node->vel(d) = -vel_max_(d);
          }
        }  // for each dimension

        // Update the particle's postion
        cur_node->pos = cur_node->pos + cur_node->vel;

        if (coeff_update_func) {
          coeff_update_func(cur_node->pos);
        }

        // Evaluate the function at the new position
        cur_node->residue = obj_func_(cur_node->pos);

        if (cur_node->residue < cur_node->best_residue) {
          cur_node->best_residue = cur_node->residue;
          cur_node->best_pos = cur_node->pos;
        }

        if (cur_node->residue < best_residue_global_) {
          best_residue_global_ = cur_node->residue;
          best_pos_global_ = cur_node->pos;
        }
      }  // for each agent

      num_iterations ++;

      // Calculate the spread in coefficients
      for (uint32_t j = 0; j < num_coeffs_; j++) {
        cur_c_min_(j) = std::numeric_limits<float>::infinity();
        cur_c_max_(j) = -std::numeric_limits<float>::infinity();
        for (uint32_t i = 0; i < swarm_size_; i++) {
          if (cur_c_min_(j) > swarm_[i].pos(j)) {
            cur_c_min_(j) = swarm_[i].pos(j);
          }
          if (cur_c_max_(j) < swarm_[i].pos(j)) {
            cur_c_max_(j) = swarm_[i].pos(j);
          }
        }
      }
      delta_c_ = cur_c_max_ - cur_c_min_;
      delta_coeff = delta_c_.norm();

#ifdef PSO_VERBOSE_SOLVER
      cout << "Iteration " << num_iterations << ":" << endl;
      cout << "  --> min residue of population = " << best_residue_global_ << endl;
      cout << "  --> delta_coeff = " << delta_coeff << endl;
#endif
    } while (num_iterations <= max_iterations && 
             delta_coeff >= delta_coeff_termination);
    
    cout << endl << "Finished PSO optimization with ";
    cout << "residue " << best_residue_global_ << endl;

    return best_pos_global_;
  }
  
  // interpolateCoeff performs the following:
  // ret = a + interp_val * (b - c)
  float PSOFitting::interpolateCoeff(const float a, const float interp_val, 
    const float b, const float c, bool angle) {
    float interp;
    if (angle) {
      float real_a = cos(a);
      float imag_a = sin(a);
      float real_b = cos(b);
      float imag_b = sin(b);
      float real_c = cos(c);
      float imag_c = sin(c);
      float real_interp = real_a + interp_val * (real_b - real_c);
      float imag_interp = imag_a + interp_val * (imag_b - imag_c);
      interp = atan2(imag_interp, real_interp);
    } else {
      interp = a + interp_val * (b - c);
    }
    return interp;
  }

};  // namespace math
};  // namespace jtil
