#include <random>
#include <stdexcept>
#include <iostream>
#include "math/common_fitting.h"
#include "math/de_fitting.h"
#include "Eigen"

using Eigen::MatrixXf;
using std::cout;
using std::endl;
using std::runtime_error;

namespace jtil {
namespace math {

  MERSINE_TWISTER_ENG DEFitting::eng;
  UNIFORM_REAL_DISTRIBUTION DEFitting::dist_real(0, 1);

  DEFitting::DEFitting(uint32_t num_coeffs, int num_agents) {
    c_new_.resize(1, num_coeffs);
    c_best_.resize(1, num_coeffs);
    cur_c_min_.resize(1, num_coeffs);
    cur_c_max_.resize(1, num_coeffs);

    num_coeffs_ = num_coeffs;
    if (num_agents > 0) {
      num_agents_ = static_cast<uint32_t>(num_agents);
    } else {
      num_agents_ = num_coeffs * 3 / 2;
      if (num_agents_ < 10) {
        num_agents_ = 10;
      }
    }
    CR = 0.9f;
    F = 0.5f;
    
    // Allocate space for the N+1 probe points
    agents_ = new OptNode[num_agents_];
    new_agents_ = new OptNode[num_agents_];
    for (uint32_t i = 0; i < num_agents_; i++) {
      agents_[i].coeff.resize(1, num_coeffs);
      agents_[i].residue = std::numeric_limits<float>::infinity();
      new_agents_[i].coeff.resize(1, num_coeffs);
      new_agents_[i].residue = std::numeric_limits<float>::infinity();
    }

    // Some default parameters
    max_iterations = 10000;
    delta_residue_termination = 0;
    delta_coeff_termination = 1e-8f;
  }

  DEFitting::~DEFitting() {
    delete[] agents_;
    delete[] new_agents_;
  }

  // Lots of good info here: http://www.scholarpedia.org/article/Nelder-Mead_algorithm
  // and some more here: http://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method
  MatrixXf& DEFitting::fitModel(const Eigen::MatrixXf& min_c,
                                const Eigen::MatrixXf& max_c,
                                const bool* angle_coeff,
                                ObjectiveFuncPtr obj_func,
                                CoeffUpdateFuncPtr coeff_update_func) {
    eng.seed();
    obj_func_ = obj_func;
    coeff_update_func_ = coeff_update_func;
    angle_coeff_ = angle_coeff;

    cout << "Starting Differential-Evolution optimization..." << endl;

    // Initialize all random agents to Uniform(min_c, max_c)
    for (uint32_t j = 0; j < num_coeffs_; j++) {
      UNIFORM_REAL_DISTRIBUTION c_dist(min_c(j), max_c(j));
      for (uint32_t i = 0; i < num_agents_; i++) {
        float uniform_rand_num = c_dist(eng);  // [min_c, max_c)
        agents_[i].coeff(j) = uniform_rand_num;
      }
    }

    // evaluate the agent's function values and calculate residue
    float r_best = std::numeric_limits<float>::infinity();
    float r_worst = -std::numeric_limits<float>::infinity();
    for (uint32_t i = 0; i < num_agents_; i++) {
      if (coeff_update_func_) {
        coeff_update_func_(agents_[i].coeff);
      }
      agents_[i].residue = obj_func(agents_[i].coeff);
      if (agents_[i].residue < r_best) {
        r_best = agents_[i].residue;
        c_best_ = agents_[i].coeff;
      }
      if (agents_[i].residue > r_worst) {
        r_worst = agents_[i].residue;
      }
    }

#ifdef DE_VERBOSE_SOLVER
    cout << "Iteration 0:" << endl;
    cout << "  --> min residue of population = " << r_best << endl;
    cout << "  --> Agent residues: <";
    for (uint32_t i = 0; i < num_agents_; i++) {
      cout << agents_[i].residue;
      if (i != num_agents_ - 1) { cout << ", "; }
    }
    cout << ">" << endl;
#endif

    uint64_t num_iterations = 0;
    UNIFORM_INT_DISTRIBUTION dist_coeffs(0, num_coeffs_ - 1);
    UNIFORM_INT_DISTRIBUTION dist_agents(0, num_agents_ - 1);
    float delta_residue = std::numeric_limits<float>::infinity();
    float delta_coeff = std::numeric_limits<float>::infinity();
    do {
      float old_r_worst = r_worst;

      // For each agent, x_i in the population:
      for (int cur_agent = 0; cur_agent < static_cast<int>(num_agents_); cur_agent++) {
        // 1. Pick three distinct agents in the population (not equal to cur_agent)
        int a = cur_agent, b = cur_agent, c = cur_agent;
        while (a == cur_agent) {
          a = dist_agents(eng);  // [0, num_agents_-1]
        }
        while (b == cur_agent || b == a) {
          b = dist_agents(eng);  // [0, num_agents_-1]
        }
        while (c == cur_agent || c == a || c == b) {
          c = dist_agents(eng);  // [0, num_agents_-1]
        }

        // 2. Pick a random index R in 1 to n (n being the dimensionality of
        //    the optimization problem)
        int R = dist_coeffs(eng);  // [0, num_coeffs_ - 1]

        // 3. & 4. Compute the agents potentially new position and bound it to
        //         min_c and max_c.
        for (int i = 0; i < static_cast<int>(num_coeffs_); i++) {
          float r_i = dist_real(eng);  // [0, 1)
          if (r_i < CR || i == R) {
            // c_new = a + F * (b - c)
            c_new_(i) = interpolateCoeff(agents_[a].coeff(i),
                                         F,
                                         agents_[b].coeff(i),
                                         agents_[c].coeff(i),
                                         angle_coeff != 0 && angle_coeff[i]);
            if (c_new_(i) < min_c(i)) {
              c_new_(i) = min_c(i);
            }
            if (c_new_(i) > max_c(i)) {
              c_new_(i) = max_c(i);
            }
          } else {
            c_new_(i) = agents_[cur_agent].coeff(i);
          }
        }

        if (coeff_update_func) {
          coeff_update_func(c_new_);
        }
        
        // 5. Evaluate the function at the new position
        float r_new = obj_func_(c_new_);
        if (r_new < agents_[cur_agent].residue) {
          new_agents_[cur_agent].residue = r_new;
          new_agents_[cur_agent].coeff = c_new_;
        } else {
          new_agents_[cur_agent].residue = agents_[cur_agent].residue;
          new_agents_[cur_agent].coeff = agents_[cur_agent].coeff;
        }
      }  // for each agent

      // Update the agent positions for the next iteration
      r_worst = -std::numeric_limits<float>::infinity();
      for (uint32_t i = 0; i < num_agents_; i++) {
        agents_[i].residue = new_agents_[i].residue;
        agents_[i].coeff = new_agents_[i].coeff;
        if (agents_[i].residue < r_best) {
          r_best = agents_[i].residue;
          c_best_ = agents_[i].coeff;
        }
        if (agents_[i].residue > r_worst) {
          r_worst = agents_[i].residue;
        }
      }

      num_iterations ++;
      delta_residue = old_r_worst - r_worst;
      // Calculate the spread in coefficients
      for (uint32_t j = 0; j < num_coeffs_; j++) {
        cur_c_min_(j) = std::numeric_limits<float>::infinity();
        cur_c_max_(j) = -std::numeric_limits<float>::infinity();
        for (uint32_t i = 0; i < num_agents_; i++) {
          if (cur_c_min_(j) > agents_[i].coeff(j)) {
            cur_c_min_(j) = agents_[i].coeff(j);
          }
          if (cur_c_max_(j) < agents_[i].coeff(j)) {
            cur_c_max_(j) = agents_[i].coeff(j);
          }
        }
      }
      delta_c_ = cur_c_max_ - cur_c_min_;
      delta_coeff = delta_c_.norm();

#ifdef DE_VERBOSE_SOLVER
      cout << "Iteration " << num_iterations << ":" << endl;
      cout << "  --> min residue of population = " << r_best << endl;
      cout << "  --> delta_worst_residue = " << old_r_worst - r_worst << endl;
      cout << "  --> delta_coeff = " << delta_coeff << endl;
#endif
    } while (num_iterations <= max_iterations && 
             delta_residue >= delta_residue_termination && 
             delta_coeff >= delta_coeff_termination);
    
    cout << endl << "Finished Differential-Evolution optimization with ";
    cout << "residue " << r_best << endl;

    return c_best_;
  }
  
  // interpolateCoeff performs the following:
  // ret = a + interp_val * (b - c)
  float DEFitting::interpolateCoeff(const float a, const float interp_val, 
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

}  // namespace math
}  // namespace jtil