//
//  lprpso_fitting_model_fit.h
//
//  Created by Jonathan Tompson on 11/23/12.
//
//  Same implementation as the regular lprpso fitting but takes advantage of
//  tiled rendering, so that function evaluations can be performed in parallel
//

#ifndef MATH_LPRPSO_FITTING_MODEL_FIT_HEADER
#define MATH_LPRPSO_FITTING_MODEL_FIT_HEADER

#include <random>
#include "jtil/math/math_types.h"
#include "math/common_fitting.h"
#include "jtil/data_str/vector.h"
#include "jtil/data_str/vector_managed.h"

#if defined(WIN32) || defined(_WIN32)
  #define constexpr 
#endif

// #define LPRPSO_VERBOSE_SOLVER  // Print out per-iteration information

namespace model_fit { class ModelFit; }

namespace jtil {
namespace math {

  // LPRPSO Optimization using hand model
  class LPRPSOFittingModelFit {
  public:
    // Set population size (num_agents) to -1 to let the optimizer choose a
    // population size for you.
    LPRPSOFittingModelFit(uint32_t num_coeffs, int swarm_size_,
      model_fit::ModelFit* model_fit, CoeffUpdateFuncPtr coeff_update_func);
    ~LPRPSOFittingModelFit();

    // fitModel = Top level function:
    // end_c --> Output value
    // start_c --> The starting inital guess and the center of the intial 
    //           search radius.  Should be chosen very carefully to ensure 
    //           proper convergence.
    // radius_c --> The initial search radius.  Also, should be chosen
    //              very carefully to ensure proper convergence.
    // angle_coeffs --> array of length num_coeffs saying which coeffs are angles
    void fitModel(const float* end_c,
                  const float* start_c,
                  const float* radius_c,
                  const bool* angle_coeffs);

    // Termination and Optimization settings:
    float delta_coeff_termination;  // The spread of the agent coefficients
    uint64_t max_iterations;
    // LPRPSO
    float C;
    float w;
    float Pr;
    // PSO
    float c_p;
    float c_g;
    float kappa;

  private:
    model_fit::ModelFit* model_fit_;  // Not owned here
    const bool* angle_coeffs_;
    uint32_t num_coeffs_;
    uint32_t swarm_size_;
    float* c_lo_;  // lower bound of search space
    float* c_hi_;  // upper bound of search space
    float* cur_c_min_;  // Current lower bound of the swarm's postions
    float* cur_c_max_;  // Current upper bound of the swarm's postions
    float* vel_max_;
    float* delta_c_;
    float best_residue_global_;
    float* best_pos_global_;
    CoeffUpdateFuncPtr coeff_update_func_;

    data_str::VectorManaged<float*> tiled_coeffs;  // size = 8 x 8 (default)
    data_str::Vector<float> tiled_residues;  // size = 8 x 8 (default)

    SwarmNode** swarm_;
    SwarmNode** ordered_swarm_;

    static MERSINE_TWISTER_ENG eng;
    static UNIFORM_REAL_DISTRIBUTION dist_real;
    
    // ret = a + interp_val * (b - c)
    // float interpolateCoeff(const float a, const float interp_val, 
    //   const float b, const float c, const bool angle);

    // ret = a - b  // Chooses the smaller of the angles
    float calcDisplacement(const float a, const float b, const bool angle);
    void InsertionSortSwarmPts();
  };

};  // namespace math
};  // namespace jtil

#endif  // MATH_LPRPSO_FITTING_HANDS_HEADER
