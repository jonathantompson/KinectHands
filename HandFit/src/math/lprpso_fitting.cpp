#include <random>
#include <stdexcept>
#include <iostream>
#include "math/lprpso_fitting.h"
#include "Eigen"

using Eigen::MatrixXf;
using std::cout;
using std::endl;
using std::runtime_error;

namespace jtil {
namespace math {

  MERSINE_TWISTER_ENG LPRPSOFitting::eng;
  UNIFORM_REAL_DISTRIBUTION LPRPSOFitting::dist_real(0, 1);

  LPRPSOFitting::LPRPSOFitting(uint32_t num_coeffs, int swarm_size) {
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

    // Allocate space for the N probe points
    swarm_ = new SwarmNode[swarm_size_];
    ordered_swarm_ = new SwarmNode*[swarm_size_];
    for (uint32_t i = 0; i < swarm_size_; i++) {
      swarm_[i].resize(num_coeffs_);
      ordered_swarm_[i] = &swarm_[i];
    }

    // Some default parameters
    max_iterations = 1000;
    delta_coeff_termination = 1e-8f;
    // Values recommended by "Extended PSO with Partial Randomization..."
    // C = 3.0;  
    // w = 0.729;
    Pr = (1.0f / static_cast<float>(num_coeffs)) * expf(-2.0f);
    // Values consistent with other papers
    C = 3.0f;  
    w = 0.7f;
    
  }

  LPRPSOFitting::~LPRPSOFitting() {
    delete[] swarm_;
    delete[] ordered_swarm_;
  }

  // Lots of good info here: http://www.scholarpedia.org/article/Nelder-Mead_algorithm
  // and some more here: http://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method
  MatrixXf& LPRPSOFitting::fitModel(const Eigen::MatrixXf& start_c,
                                    const Eigen::MatrixXf& radius_c,
                                    const bool* angle_coeff,
                                    ObjectiveFuncPtr obj_func,
                                    CoeffUpdateFuncPtr coeff_update_func) {
    eng.seed();
    obj_func_ = obj_func;
    coeff_update_func_ = coeff_update_func;
    angle_coeff_ = angle_coeff;

    cout << "Starting LPRPSO optimization..." << endl;

    // Initialize all random swarm particles to Uniform(c_lo_, c_hi_)
    radius_c.cwiseAbs();
    c_lo_ = start_c - radius_c;
    c_hi_ = start_c + radius_c;
    vel_max_ = radius_c;  // According to paper (forgot title), 0.5x search space
                          // is best for multi-modal distributions.
    swarm_[0].pos = start_c;  // Make the 0th particle the same as the start
    for (uint32_t j = 0; j < num_coeffs_; j++) {
      UNIFORM_REAL_DISTRIBUTION c_dist(c_lo_(j), c_hi_(j));
      for (uint32_t i = 1; i < swarm_size_; i++) {
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

    // Initialize random velocity to Uniform(radius_c, radius_c)
    for (uint32_t j = 0; j < num_coeffs_; j++) {
      UNIFORM_REAL_DISTRIBUTION c_dist(radius_c(j), radius_c(j));
      for (uint32_t i = 0; i < swarm_size_; i++) {
        float uniform_rand_num = c_dist(eng);  // [-2*radius_c, 2*radius_c)
        swarm_[i].vel(j) = uniform_rand_num;
      }
    }

#ifdef LPRPSO_VERBOSE_SOLVER
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
    cout << "Iteration 0:" << endl;
    cout << "  --> min residue of population = " << best_residue_global_ << endl;
    cout << "  --> Agent residues: <";
    for (uint32_t i = 0; i < swarm_size_; i++) {
      cout << swarm_[i].residue;
      if (i != swarm_size_ - 1) { cout << ", "; }
    }
    cout << ">" << endl;
    cout << "  --> delta_c_ = "; 
    for (uint32_t i = 0; i < num_coeffs_; i++) { cout << delta_c_(i) << " "; }
    float bounding_area = delta_c_(0);
    for (uint32_t i = 1; i < num_coeffs_; i++) {
      bounding_area *= delta_c_(i);
    }
    cout << endl << "  --> bounding_area = " << bounding_area << endl;
#endif

    uint64_t num_iterations = 0;
    float preturb_rad = 1.0f;
    // Exponential decay, want preturb_rad = 1e-4 on max_iteration
    const float nth_rad = 1e-4f;
    const float preturb_rad_decay =
      powf(10.0f, log10f(nth_rad) / static_cast<float>(max_iterations));
    float delta_coeff = std::numeric_limits<float>::infinity();
    do {
      // As per the PrPSO part of the paper, randomly preturb particles on
      // a restart.
      if (num_iterations > 0) {
        // For each particle, i in the swarm:
        for (uint32_t i = 0; i < swarm_size_; i++) {
          SwarmNode* cur_node = &swarm_[i];
          // For each dimension d:
          for (uint32_t d = 0; d < num_coeffs_; d++) {
            float rand = dist_real(eng);  // [0,1)
            // With some probability, preterb the position of the d-th coeff
            // using Uniform(c_lo_, c_hi_)
            if (rand <= Pr) {
              // cur_node->pos(d) = (c_hi_(d) - c_lo_(d)) * dist_real(eng) + c_lo_(d);  // [c_lo_, c_hi_]
              cur_node->pos(d) += preturb_rad * (c_hi_(d) - c_lo_(d)) * (2.0f * dist_real(eng) - 1.0f);
            }
          }  // for each dimension
          if (coeff_update_func) {
            coeff_update_func(cur_node->pos);
          }

          // Now re-evaluate the residual at the new position
          cur_node->residue = obj_func_(cur_node->pos);
          if (cur_node->residue < cur_node->best_residue) {
            cur_node->best_residue = cur_node->residue;
            cur_node->best_pos = cur_node->pos;
          }
          if (cur_node->residue < best_residue_global_) {
            best_residue_global_ = cur_node->residue;
            best_pos_global_ = cur_node->pos;
          }
        }
      }

      // Particles need to be ranked by their residue:
      InsertionSortSwarmPts();  // O(N^2) worst case, O(N) likely best case

      // For each particle, i in the swarm:
      for (uint32_t i = 0; i < swarm_size_; i++) {
        SwarmNode* cur_node = ordered_swarm_[i];
        float rank = static_cast<float>(i + 1);  // 0 --> Best, swarm_size_-1 --> Worst
        float tmp = static_cast<float>(swarm_size_ - 1);
        float tmp2 = rank - static_cast<float>(swarm_size_);
        float c_p = (C / (tmp * tmp)) * (tmp2 * tmp2);
        float c_g = C - c_p;
        // For each dimension d:
        for (uint32_t d = 0; d < num_coeffs_; d++) {
          float r_p = dist_real(eng);  // [0,1)
          float r_g = dist_real(eng);  // [0,1)
          // Update the velocity
          cur_node->vel(d) = w * cur_node->vel(d) + 
            (c_p * r_p * (cur_node->best_pos(d) - cur_node->pos(d))) + 
            (c_g * r_g * (best_pos_global_(d) - cur_node->pos(d)));
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
      preturb_rad *= preturb_rad_decay;

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

#ifdef LPRPSO_VERBOSE_SOLVER
      cout << "Iteration " << num_iterations << ":" << endl;
      cout << "  --> min residue of population = " << best_residue_global_ << endl;
      cout << "  --> delta_coeff = " << delta_coeff << endl;
      cout << "  --> delta_c_ = "; 
      for (uint32_t i = 0; i < num_coeffs_; i++) { cout << delta_c_(i) << " "; }
      float bounding_area = delta_c_(0);
      for (uint32_t i = 1; i < num_coeffs_; i++) {
        bounding_area *= delta_c_(i);
      }
      cout << endl << "  --> bounding_area = " << bounding_area << endl;
#endif
    } while (num_iterations <= max_iterations && 
             delta_coeff >= delta_coeff_termination);
    
    cout << "Finished PSO optimization with residue ";
    cout << best_residue_global_ << endl;

    return best_pos_global_;
  }
  
  // interpolateCoeff performs the following:
  // ret = a + interp_val * (b - c)
  float LPRPSOFitting::interpolateCoeff(const float a, const float interp_val, 
    const float b, const float c, bool angle) {
    float interp;
    if (angle) {
      float real_a = cosf(a);
      float imag_a = sinf(a);
      float real_b = cosf(b);
      float imag_b = sinf(b);
      float real_c = cosf(c);
      float imag_c = sinf(c);
      float real_interp = real_a + interp_val * (real_b - real_c);
      float imag_interp = imag_a + interp_val * (imag_b - imag_c);
      interp = atan2f(imag_interp, real_interp);
    } else {
      interp = a + interp_val * (b - c);
    }
    return interp;
  }

  // http://en.wikipedia.org/wiki/Insertion_sort
  void LPRPSOFitting::InsertionSortSwarmPts() {
    for (uint32_t i = 1; i < swarm_size_; i++) {
      SwarmNode* item = ordered_swarm_[i];
      uint32_t i_hole = i;
      while (i_hole > 0 && ordered_swarm_[i_hole - 1]->residue > item->residue) {
        // move hole to next smaller index
        ordered_swarm_[i_hole] = ordered_swarm_[i_hole - 1];
        i_hole--;
      }
      // put item in the hole
      ordered_swarm_[i_hole] = item;
    }
#if (defined(DEBUG) || defined(_DEBUG)) && defined(LPRPSO_VERBOSE_SOLVER)
    cout << endl << "ordered_swarm = ";
    for (uint32_t i = 0; i < swarm_size_; i++) {
      cout << ordered_swarm_[i]->residue;
      if (i != swarm_size_ - 1) {
        cout << ", ";
      }
    }
    cout << endl << endl;
#endif
  }

}  // namespace math
}  // namespace jtil
