//
//  hand_model_fit.h
//
//  Created by Jonathan Tompson on 8/17/12.
//

#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include "model_fit/model_fit.h"
#include "model_fit/model_renderer.h"
#include "jtil/data_str/vector_managed.h"
#include "jtil/string_util/string_util.h"
#include "jtil/math/pso_parallel.h"
#include "jtil/file_io/file_io.h"
#include "jtil/file_io/csv_handle_write.h"
#include "renderer/gl_state.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"

#define SAFE_DELETE(target) \
  if (target != NULL) { \
    delete target; \
    target = NULL; \
  }

using Eigen::Matrix;
using Eigen::MatrixXf;
using namespace jtil::math;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using jtil::file_io::CSVHandleWrite;
using jtil::data_str::VectorManaged;
using renderer::GLState;

namespace model_fit {
  
  // Static variables
  ModelFit* ModelFit::cur_fit_ = NULL;
  const float* ModelFit::coeff_min_limit = NULL;
  const float* ModelFit::coeff_max_limit = NULL;
  const float* ModelFit::coeff_penalty_scale = NULL;
  const bool* ModelFit::angle_coeffs = NULL;
  uint64_t ModelFit::func_eval_count_ = 0;
 
  ModelFit::ModelFit(uint32_t num_models, uint32_t coeff_size_per_model,
    float* pso_radius_c, CoeffUpdateFuncPtr renormalize_coeff_func) {
    num_models_ = num_models;
    coeff_dim_ = coeff_size_per_model * num_models;
    pso_radius_c_ = pso_radius_c;

    coeff_ = new float[coeff_dim_];
    prev_coeff_ = new float[coeff_dim_];
    coeff_tmp_ = new float[coeff_dim_];

    lprpso_ = new jtil::math::PSOParallel(coeff_dim_, PSO_SWARM_SIZE,
      this, renormalize_coeff_func);
    lprpso_->max_iterations = PSO_MAX_ITERATIONS;
    lprpso_->delta_coeff_termination = PSO_DELTA_C_TERMINATION;

    kinect_depth_masked_ = new int16_t[src_dim];

    model_renderer_ = new ModelRenderer(
  }

  ModelFit::~ModelFit() {
    SAFE_DELETE(lprpso_);
    SAFE_DELETE_ARR(kinect_depth_masked_);
  }

  void ModelFit::prepareKinectData(int16_t* depth, uint8_t* label) {
    memcpy(kinect_depth_masked_, depth, 
      sizeof(kinect_depth_masked_[0]) * src_dim);
#ifdef DEPTH_ONLY_RESIDUE_FUNC
    // Do nothing
#else
    for (uint32_t i = 0; i < src_dim; i++) {
      if (label[i] == 0) {
        kinect_depth_masked_[i] = 0;
      }
    }
#endif
    hand_renderer_->uploadKinectDepth(kinect_depth_masked_);
  }

  void handModel2Eigen(Eigen::MatrixXf& coeff, HandModel* hands[2], 
    const int nhands) {
    memcpy(coeff.block<1, HAND_NUM_COEFF>(0, 0).data(), hands[0]->coeff(),
      HAND_NUM_COEFF * sizeof(hands[0]->coeff()[0]));
    if (nhands > 1) {
      memcpy(coeff.block<1, HAND_NUM_COEFF>(0, HAND_NUM_COEFF).data(), 
        hands[1]->coeff(), HAND_NUM_COEFF * sizeof(hands[1]->coeff()[0]));
    }
  }

  void eigen2HandModel(HandModel* hands[2], Eigen::MatrixXf& coeff, 
    const int nhands) {
    if (nhands == 1) {
      hands[0]->copyCoeffFrom(coeff.data(), HAND_NUM_COEFF);
    } else {
      hands[0]->copyCoeffFrom(coeff.block<1, HAND_NUM_COEFF>(0, 0).data(),
        HAND_NUM_COEFF);
      hands[1]->copyCoeffFrom(coeff.block<1, HAND_NUM_COEFF>(0, HAND_NUM_COEFF).data(),
        HAND_NUM_COEFF);
    }
  }

  void ModelFit::fitModel(int16_t* depth, uint8_t* label, HandModel* hands[2],
    HandModel* prev_hands[2]) {
    min_pts_.resize(0);
    cur_hand_[0] = hands[0];
    cur_hand_[1] = hands[1];
    prev_hand_[0] = prev_hands[0];
    prev_hand_[1] = prev_hands[1];
    handModel2Eigen(coeff_, hands, nhands_);
    if (prev_hand_[0] != NULL) {
      handModel2Eigen(prev_coeff_, prev_hands, nhands_);
    }
    cur_fit_ = this;
    prepareKinectData(depth, label);

    hand_renderer_->preBindUniforms(); 

    // All matrices are empty!  This is because all the evaluation is done
    // on the GPU.  Therefore the optimizer doesn't actually need the values.
    MatrixXf x, y, f;

    // This is a hack: There's something wrong with the first iteration using
    // tiled rendering --> Doing one regular render pass helps.
    float start_func_val = objectiveFunc(coeff_);
    cout << "Starting objective function value = " << start_func_val << endl;
    
    // PrPSO fitting --> Supposedly deals better with multi-modal funcs
    float rad = 1.0f;
    for (uint32_t i = 0; i < PSO_REPEATS; i++) {
      Eigen::MatrixXf cur_pso_radius_c_ = pso_radius_c_ * rad;
      coeff_ = lprpso_fitting_->fitModel(coeff_, cur_pso_radius_c_, angle_coeffs);
      // manualFit();
      rad = rad * 0.5f;
    }

    // Copy back fitted coefficients
    eigen2HandModel(cur_hand_, coeff_, nhands_);
  }

  float ModelFit::queryObjectiveFunction(int16_t* depth, uint8_t* label, 
    HandModel* hands[2], HandModel* prev_hands[2]) {
    cur_hand_[0] = hands[0];
    cur_hand_[1] = hands[1];
    prev_hand_[0] = prev_hands[0];
    prev_hand_[1] = prev_hands[1];
    handModel2Eigen(coeff_, hands, nhands_);
    if (prev_hand_[0] != NULL) {
      handModel2Eigen(prev_coeff_, prev_hands, nhands_);
    }
    cur_fit_ = this;
    prepareKinectData(depth, label);
    MatrixXf x, y, f;
    evalFunc(f, coeff_, x);
    return calculateResidual(y, f, coeff_);
  }

  char* float2CStr(float val) {
    string str = jtil::string_util::Num2Str<float>(val);
    char* c_str = new char[str.length()+1];
    std::copy(str.begin(), str.end(), c_str);
    c_str[str.size()] = '\0';
    return c_str;
  }
  
  void ModelFit::evalFunc(Eigen::MatrixXf& f_val, const Eigen::MatrixXf& coeff, 
    const Eigen::MatrixXf& x) {
    static_cast<void>(x);
    static_cast<void>(f_val);

    cur_fit_->hand_renderer_->drawDepthMap(coeff, cur_hand_, cur_fit_->nhands_);

    func_eval_count_++;
    glFlush();
  }
  
  // modulu - similar to matlab's mod()
  // result is always possitive. not similar to fmod()
  // Mod(-3,4)= 1   fmod(-3,4)= -3
#if defined(__APPLE__) || defined(_WIN32)
  float inline __fastcall Mod(float x, float y) {
    if (0 == y) {
      return x;
    }
    
    return x - y * floor(x / y);
  }
#else
  float inline Mod(float x, float y) {
    if (0 == y) {
      return x;
    }

    return x - y * floor(x / y);
  }
#endif
  
#if defined(WIN32) || defined(_WIN32)
  std::tr1::mt19937 eng;  // a core engine class
  std::tr1::normal_distribution<float> dist;
#else
  std::mt19937 eng;  // a core engine class
  std::normal_distribution<float> dist;
#endif

  void ModelFit::preturbCoeffs(Eigen::MatrixXf& coeff) {
    for (uint32_t i = 0; i < HAND_NUM_COEFF; i++) {
      float norm_rand_num_ = dist(eng);
      coeff(i) = coeff(i) + norm_rand_num_ * preturb_coeff_[i];
    }
    HandModel::renormalizeCoeffs(coeff.data());
  }

  float ModelFit::calculateResidual(const Eigen::MatrixXf& y, 
    const Eigen::MatrixXf& f_x, const Eigen::MatrixXf& coeff) {
    static_cast<void>(coeff);

    // Residue is calculated on the GPU
    float data_term = cur_fit_->hand_renderer_->calculateResidualDataTerm();
    float interpen_term = 
      cur_fit_->hand_renderer_->calcInterpenetrationTerm();

    // Now calculate the penalty terms:
    float penalty_term = calcPenalty(coeff);  // formally scaled by 0.1

    // return penalty_term * (data_term + interpen_term);
    return penalty_term * data_term * interpen_term;
  }

  void ModelFit::calculateResidualTiled(jtil::data_str::Vector<float>& residues, 
    jtil::data_str::VectorManaged<Eigen::MatrixXf>& coeffs) {
    // Note: at this point interpenetration term is already calculated and is
    // sitting in residues[i]
    cur_fit_->hand_renderer_->calculateResidualDataTermTiled(residues);

    // Now calculate the penalty terms:
    for (uint32_t i = 0; i < coeffs.size(); i++) {
      float penalty_term = calcPenalty(coeffs[i]);  // formally scaled by 0.1
      residues[i] *= penalty_term;
    }
  }

  float ModelFit::calcPenalty(const Eigen::MatrixXf& coeff) {
    float penalty = 1.0f;
    for (uint32_t i = 0; i < cur_fit_->coeff_dim_; i++) {
      if (coeff_penalty_scale_[i % HAND_NUM_COEFF] > EPSILON) {
        float cur_coeff_val = coeff(i);

#ifdef LINEAR_PENALTY
        // Linear penalty
        if (cur_coeff_val > coeff_max_limit[i%HAND_NUM_COEFF]) {
          penalty += coeff_penalty_scale_[i%HAND_NUM_COEFF] * 
            fabsf(cur_coeff_val - coeff_max_limit[i%HAND_NUM_COEFF]) / 10.0f;
        }
        if (cur_coeff_val < coeff_min_limit[i%HAND_NUM_COEFF]) { 
          penalty += coeff_penalty_scale_[i%HAND_NUM_COEFF] * 
            fabsf(coeff_min_limit[i%HAND_NUM_COEFF] - cur_coeff_val) / 10.0f;
        }
#else
        // Quadra penalty
        if (cur_coeff_val > coeff_max_limit[i%HAND_NUM_COEFF]) {
          float cur_penalty = coeff_penalty_scale_[i%HAND_NUM_COEFF] * 
            fabsf(cur_coeff_val - coeff_max_limit[i%HAND_NUM_COEFF]) / 2.0f;
          penalty += (cur_penalty * cur_penalty);
        }
        if (cur_coeff_val < coeff_min_limit[i%HAND_NUM_COEFF]) { 
          float cur_penalty = coeff_penalty_scale_[i%HAND_NUM_COEFF] * 
            fabsf(coeff_min_limit[i%HAND_NUM_COEFF] - cur_coeff_val) / 2.0f;
          penalty += (cur_penalty * cur_penalty);
        }
#endif
      }
    }

    // Now add in the penalty due to marked points in the space:
    if (include_min_pts_constraints_) {
      std::cout << "USING MIN_PT_CONSTRAINTS!!  Do you mean to?" << std::endl;
      for (uint32_t i_cur_pt = 0; i_cur_pt < min_pts_.size(); i_cur_pt++) {
        float dist_sq = 0;
        float* cur_min_pt = min_pts_[i_cur_pt].c;
        for (uint32_t j = 0; j < cur_fit_->coeff_dim_; j++) {
          float cur_delta = cur_min_pt[j] - coeff(j);
          if ((j % HAND_NUM_COEFF) < HAND_POS_Z) {
            // Position coeff
            cur_delta /= 10;  // Pretend 1cm is roughly full range
          }
          dist_sq += (cur_delta * cur_delta);
        }
        float cur_penalty = DESCENT_PENALTY_SIZE * (1.0f / sqrtf(dist_sq));
#if defined(WIN32) || defined(_WIN32)
        penalty += min(cur_penalty, 1000000);  // Limit the penalty (don't want singularity)
#else
        penalty += std::min<float>(cur_penalty, 1000000);
#endif
      }
    }

#ifdef PREV_FRAME_DIST_PENALTY
    if (prev_hand_[0] != NULL) {
      penalty += calcDistPenalty(&cur_fit_->prev_coeff_(0), &coeff(0));
      if (cur_fit_->coeff_dim_ > HAND_NUM_COEFF) {
        penalty += calcDistPenalty(&cur_fit_->prev_coeff_(HAND_NUM_COEFF), 
          &coeff(HAND_NUM_COEFF));
      }
    }
#endif

    return penalty;
  }

  float ModelFit::calcDistPenalty(const float* coeff0, const float* coeff1) {
    float dist = 0;  // Prevent NANs
    for (uint32_t i = 0; i < HAND_NUM_COEFF; i++) {
      if (coeff_penalty_scale_[i] > EPSILON) {
        float delta = fabsf(coeff0[i] - coeff1[i]);
        float err = delta - PREV_FRAME_DIST_PENALTY_THRESHOLD * 
          (coeff_max_limit[i] - coeff_min_limit[i]);
        if (err > 0) {
          dist += err * err;
        }
      }
    }
    return PREV_FRAME_DIST_PENALTY_SCALE * dist;
  }

  float ModelFit::evaluateFuncAndCalculateResidual() {
    MatrixXf x;
    MatrixXf y;
    MatrixXf f;
    evalFunc(f, coeff_, x);
    return calculateResidual(y, f, coeff_);
  }

  float ModelFit::objectiveFunc(const MatrixXf& coeff) {
    MatrixXf x;
    MatrixXf y;
    MatrixXf f;
    evalFunc(f, coeff, x);
    return calculateResidual(y, f, coeff);
  }

  void ModelFit::objectiveFuncTiled(jtil::data_str::Vector<float>& residues, 
    jtil::data_str::VectorManaged<Eigen::MatrixXf>& coeffs) {
    if (coeffs.size() > NTILES) {
      throw runtime_error("objectiveFuncTiled() - coeffs.size() > NTILES");
    }
    if (residues.capacity() < coeffs.size()) {
      residues.capacity(coeffs.size());
    }
    residues.resize(0);
    for (uint32_t i = 0; i < coeffs.size(); i++) {
      residues.pushBack(0);
    }
    cur_fit_->hand_renderer_->drawDepthMapTiled(residues, coeffs, cur_hand_, 
      cur_fit_->nhands_);
    func_eval_count_ += NTILES;
    calculateResidualTiled(residues, coeffs);

#if defined(_DEBUG) || defined(DEBUG)
    // Very expensive, but double check that they are correct!
    for (uint32_t i = 0; i < coeffs.size(); i++) {
      float obj_func = objectiveFunc(coeffs[i]);
      if (abs(obj_func - residues[i]) / abs(residues[i]) > 1e-2) {
        cout << "tiled vs non-tiled obj function residue mismatch!" << endl;
        cout << obj_func << " vs " << residues[i] << endl;
        throw runtime_error("ERROR: tiled residue doesn't match single residue!");
      }
    }
#endif
  }

  Coeff& Coeff::operator=(const Coeff &rhs) {
    // Only do assignment if RHS is a different object from this.
    if (this != &rhs) {
      memcpy(this->c, rhs.c, HAND_NUM_COEFF*2*sizeof(this->c[0]));
    }
    return *this;
  }

  void Coeff::copyFromEigen(const Eigen::MatrixXf &c) {
    for (int32_t i = 0; i < c.cols() || i < c.rows(); i++) {
      this->c[i] = c(i);
    }
  }

  void Coeff::copyToEigen(Eigen::MatrixXf &c) {
    for (int32_t i = 0; i < c.cols() || i < c.rows(); i++) {
      c(i) = this->c[i];
    }
  }

}  // namespace hand_fit
