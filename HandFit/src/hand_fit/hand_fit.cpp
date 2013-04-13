//
//  hand_model_fit.h
//
//  Created by Jonathan Tompson on 8/17/12.
//

#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include "hand_fit/hand_fit.h"
#include "hand_fit/hand_renderer.h"
#include "jtil/data_str/vector_managed.h"
#include "jtil/string_util/string_util.h"
#include "math/lm_fitting.h"
#include "math/nm_fitting.h"
#include "math/de_fitting.h"
#include "math/bfgs_fitting.h"
#include "math/pso_fitting.h"
#include "math/lprpso_fitting_hands.h"
#include "jtil/file_io/file_io.h"
#include "jtil/file_io/csv_handle_write.h"
#include "Eigen"
#include "renderer/gl_state.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"

#define SAFE_DELETE(target) \
  if (target != NULL) { \
    delete target; \
    target = NULL; \
  }

using Eigen::Matrix;
using Eigen::MatrixXf;
using jtil::math::Float4x4;
using jtil::math::FloatQuat;
using jtil::math::Float3;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using jtil::math::Float3;
using jtil::file_io::CSVHandleWrite;
using jtil::data_str::VectorManaged;
using renderer::GLState;
using namespace kinect_interface::hand_net;

namespace hand_fit {
  
  // Static variables
  HandFit* HandFit::cur_fit_ = NULL;
  HandModel* HandFit::cur_hand_[2] = {NULL, NULL};
  HandModel* HandFit::prev_hand_[2] = {NULL, NULL};
  uint64_t HandFit::func_eval_count_ = 0;
  jtil::data_str::Vector<Coeff> HandFit::min_pts_;
  bool HandFit::include_min_pts_constraints_ = false;
  jtil::data_str::VectorManaged<Eigen::MatrixXf> HandFit::tiled_coeffs_;
  jtil::data_str::Vector<float> HandFit::tiled_residues_;
  
  const float HandFit::finger_crossover_penalty_threshold = -0.12566f;  // ~ 5 deg
 
  HandFit::HandFit(HandRenderer* hand_renderer, uint32_t num_hands) {
    if (num_hands > 2) {
      throw std::runtime_error("HandFit::HandFit() - num_hands > 2");
    }

    nhands_ = num_hands;
    hand_renderer_ = hand_renderer;

    coeff_dim_ = HAND_NUM_COEFF * nhands_;

    coeff_.resize(1, coeff_dim_);
    prev_coeff_.resize(1, coeff_dim_);
    coeff_tmp_.resize(1, coeff_dim_);
    pso_radius_c_.resize(1, coeff_dim_);
    nm_coeff_step_size_.resize(1, coeff_dim_);
    min_pts_.capacity(DESCENT_NUM_REPEATS);

    nm_fitting_ = new jtil::math::NMFitting(coeff_dim_);
    nm_fitting_->simplex_diameter_termination = NM_SIMPLEX_DIAMETER_TERMINATION;
    nm_fitting_->max_iterations = NM_MAX_ITERATIONS;

    lprpso_fitting_ = new jtil::math::LPRPSOFittingHands(coeff_dim_, PSO_SWARM_SIZE,
      this);
    lprpso_fitting_->max_iterations = PSO_MAX_ITERATIONS;
    lprpso_fitting_->delta_coeff_termination = PSO_DELTA_C_TERMINATION;

    bfgs_fitting_ = new jtil::math::BFGSFitting(coeff_dim_);
    bfgs_fitting_->delta_x_2norm_term = DESCENT_DELTA_X_2_NORM_TERM;
    bfgs_fitting_->delta_f_term = DESCENT_DELTA_F_TERM;

    // Set the PSO static radius
    for (uint32_t i = HAND_POS_X; i <= HAND_POS_Z; i++) {
      pso_radius_c_(i) = PSO_RAD_POSITION;
    }
    for (uint32_t i = HAND_ORIENT_X; i <= HAND_ORIENT_Z; i++) {
      pso_radius_c_(i) = PSO_RAD_EULER;
    }
    for (uint32_t i = WRIST_THETA; i <= WRIST_PHI; i++) {  // Wrist
      pso_radius_c_(i) = (coeff_max_limit[i] - coeff_min_limit[i]) * PSO_RAD_WRIST;
    }
    for (uint32_t i = THUMB_THETA; i <= THUMB_K2_PHI; i++) {  // thumb
      pso_radius_c_(i) = (coeff_max_limit[i] - coeff_min_limit[i]) * PSO_RAD_THUMB;
    }
    for (uint32_t i = 0; i < 4; i++) {  // All fingers
      pso_radius_c_(F0_ROOT_THETA+i*FINGER_NUM_COEFF) = 
        (coeff_max_limit[F0_ROOT_THETA+i*FINGER_NUM_COEFF] - 
        coeff_min_limit[F0_ROOT_THETA+i*FINGER_NUM_COEFF]) * PSO_RAD_FINGERS;
      pso_radius_c_(F0_ROOT_PHI+i*FINGER_NUM_COEFF) = 
        (coeff_max_limit[F0_ROOT_PHI+i*FINGER_NUM_COEFF] - 
        coeff_min_limit[F0_ROOT_PHI+i*FINGER_NUM_COEFF]) * PSO_RAD_FINGERS;
      pso_radius_c_(F0_THETA+i*FINGER_NUM_COEFF) = 
        (coeff_max_limit[F0_THETA+i*FINGER_NUM_COEFF] - 
        coeff_min_limit[F0_THETA+i*FINGER_NUM_COEFF]) * PSO_RAD_FINGERS;
      pso_radius_c_(F0_PHI+i*FINGER_NUM_COEFF) = 
        (coeff_max_limit[F0_PHI+i*FINGER_NUM_COEFF] - 
        coeff_min_limit[F0_PHI+i*FINGER_NUM_COEFF]) * PSO_RAD_FINGERS;
      pso_radius_c_(F0_KNUCKLE_MID+i*FINGER_NUM_COEFF) = 
        (coeff_max_limit[F0_KNUCKLE_MID+i*FINGER_NUM_COEFF] - 
        coeff_min_limit[F0_KNUCKLE_MID+i*FINGER_NUM_COEFF]) * PSO_RAD_FINGERS;
      pso_radius_c_(F0_KNUCKLE_END+i*FINGER_NUM_COEFF) = 
        (coeff_max_limit[F0_KNUCKLE_END+i*FINGER_NUM_COEFF] - 
        coeff_min_limit[F0_KNUCKLE_END+i*FINGER_NUM_COEFF]) * PSO_RAD_FINGERS;
    }
#ifdef FIT_TWIST
    for (uint32_t i = F0_TWIST; i <= THUMB_TWIST; i++) {  // thumb
      pso_radius_c_(i) = (coeff_max_limit[i] - coeff_min_limit[i]) * PSO_RAD_FINGERS;
    }
#endif
    // Copy the rest of the coeffs over to the second hand
    for (uint32_t i = HAND_NUM_COEFF; i < coeff_dim_; i++) {
      pso_radius_c_(i) = pso_radius_c_(i - HAND_NUM_COEFF);
    }

    if (tiled_coeffs_.capacity() < HAND_NUM_COEFF * 4) {
      tiled_coeffs_.capacity(HAND_NUM_COEFF * 4);
    }
    if (tiled_residues_.capacity() < HAND_NUM_COEFF * 4) {
      tiled_residues_.capacity(HAND_NUM_COEFF * 4);
    }
  }

  HandFit::~HandFit() {
    SAFE_DELETE(nm_fitting_);
    SAFE_DELETE(lprpso_fitting_);
    SAFE_DELETE(bfgs_fitting_);
  }

  void HandFit::prepareKinectData(int16_t* depth, uint8_t* label) {
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

  void HandFit::fitModel(int16_t* depth, uint8_t* label, HandModel* hands[2],
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

    //// Use Nelder-Mead
    //Eigen::MatrixXf c_best_ = coeff_;
    //evalFunc(f, c_best_, x);
    //float residual_best = calculateResidual(y, f, c_best_);
    //const float Pr = 2.0f / static_cast<float>(HAND_NUM_COEFF);
    //for (uint32_t cur_restart = 0; cur_restart < NM_NUM_RESARTS; cur_restart++) {
    //  coeff_ = c_best_;

    //  if (cur_restart > 0) {
    //    // Preturb the coefficients slightly with some probability
    //    for (uint32_t i = 0; i < HAND_NUM_COEFF; i++) {
    //      if (fabsf(dist(eng)) <= Pr) {
    //        coeff_(i) += dist(eng) * preturb_coeff_[i];
    //      }
    //    }
    //  }

    //  for (uint32_t i = 0; i < HAND_NUM_COEFF; i++) {
    //    nm_coeff_step_size_(i) = preturb_coeff_[i] * 10.0f;
    //  }
    //  coeff_ = nm_fitting_->fitModel(coeff_, nm_coeff_step_size_,
    //    angle_coeffs, &objectiveFunc, &HandModel::renormalizeCoeffs);

    //  float cur_residual = evaluateFuncAndCalculateResidual();
    //  if (cur_residual < residual_best) {
    //    residual_best = cur_residual;
    //    c_best_ = coeff_;
    //  }
    //}

    // Fit each coefficient heirachically.
    // manualFit();

    // Fit using repeated steepest-descent with backtracking and by adding a
    // penalty constrain on each new iteration
    //MERSINE_TWISTER_ENG eng;
    //UNIFORM_REAL_DISTRIBUTION dist(-1,1);
    //min_pts_.resize(0);
    //include_min_pts_constraints_ = false;
    //Eigen::MatrixXf c_best_ = coeff_;
    //float r_best = objectiveFunc(coeff_);
    //include_min_pts_constraints_ = true;
    //for (uint32_t i = 0; i < DESCENT_NUM_REPEATS; i++) {
    //  if (i > 0) {
    //    coeff_ = c_best_;
    //    // Preturb the coefficients slightly with some probability
    //    for (uint32_t i = 0; i < coeff_dim_; i++) {
    //      if (dist(eng) < 1.0f / static_cast<float>(HAND_NUM_COEFF)) {
    //        if (i <= HAND_POS_Z) {
    //          coeff_(i) += dist(eng) * 10.0f;
    //        } else {
    //          coeff_(i) += dist(eng) * 0.5f;
    //        }
    //      }
    //    }
    //    HandModel::renormalizeCoeffs(coeff_);
    //  }
    //  coeff_ = bfgs_fitting_->fitModel(coeff_, angle_coeffs,
    //    &objectiveFunc, &approxJacobianTiled, &HandModel::renormalizeCoeffs);
    //  min_pts_.resize(min_pts_.size()+1);
    //  min_pts_[min_pts_.size()-1].copyFromEigen(coeff_);

    //  // Now try the current point without the constraints
    //  include_min_pts_constraints_ = false;
    //  float obj_func = objectiveFunc(coeff_);
    //  if (obj_func < r_best) {
    //    r_best = obj_func;
    //    c_best_ = coeff_;
    //  }
    //  include_min_pts_constraints_ = true;
    //}
    //include_min_pts_constraints_ = false;
    //coeff_ = c_best_;
    //cout << "Final residual = " << objectiveFunc(coeff_) << endl;

    // Copy back fitted coefficients
    eigen2HandModel(cur_hand_, coeff_, nhands_);
  }

  float HandFit::queryObjectiveFunction(int16_t* depth, uint8_t* label, 
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
  
  void HandFit::evalFunc(Eigen::MatrixXf& f_val, const Eigen::MatrixXf& coeff, 
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

  void HandFit::preturbCoeffs(Eigen::MatrixXf& coeff) {
    for (uint32_t i = 0; i < HAND_NUM_COEFF; i++) {
      float norm_rand_num_ = dist(eng);
      coeff(i) = coeff(i) + norm_rand_num_ * preturb_coeff_[i];
    }
    HandModel::renormalizeCoeffs(coeff.data());
  }

  float HandFit::calculateResidual(const Eigen::MatrixXf& y, 
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

  void HandFit::calculateResidualTiled(jtil::data_str::Vector<float>& residues, 
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

  void HandFit::approxJacobianTiled(MatrixXf& jacob, const MatrixXf& coeff) {
    throw std::wruntime_error("NEEDS UPDATING");
    //for (uint32_t frame = 0; frame < cur_fit_->coeff_dim_ / HAND_NUM_COEFF; frame++) {
    //  uint32_t c_start = frame * HAND_NUM_COEFF;
    //  uint32_t c_end = (frame + 1) * HAND_NUM_COEFF;

    //  // Fill up the coeff array for tiled rendering
    //  tiled_coeffs_.resize(0);  
    //  for (uint32_t i = c_start; i < c_end; i++) {
    //    cur_fit_->coeff_tmp_ = coeff;
    //    
    //    // Positive step
    //    cur_fit_->coeff_tmp_(i) = coeff(i) + delta_coeff_[i % HAND_NUM_COEFF];
    //    HandModel::renormalizeCoeffs(cur_fit_->coeff_tmp_.data());
    //    tiled_coeffs_.pushBack(cur_fit_->coeff_tmp_);

    //    // negative step
    //    cur_fit_->coeff_tmp_(i) = coeff(i) - delta_coeff_[i % HAND_NUM_COEFF];
    //    HandModel::renormalizeCoeffs(cur_fit_->coeff_tmp_.data());
    //    tiled_coeffs_.pushBack(cur_fit_->coeff_tmp_);
    //  }

    //  // Calculate the objective function in parallel
    //  objectiveFuncTiled(tiled_residues_, tiled_coeffs_);

    //  for (uint32_t i = c_start; i < c_end; i++) {
    //    jacob(i) = (tiled_residues_[2 * (i % HAND_NUM_COEFF)] - 
    //      tiled_residues_[2 * (i % HAND_NUM_COEFF)+1]) / 
    //      (2.0f * delta_coeff_[i % HAND_NUM_COEFF]);
    //  }
    //}
  }

  void HandFit::approxJacobian(MatrixXf& jacob, const MatrixXf& coeff) {
    throw std::wruntime_error("NEEDS UPDATING");
    //cur_fit_->coeff_tmp_ = coeff;
    //for (uint32_t i = 0; i < cur_fit_->coeff_dim_; i++) {
    //  if (i > 0) {
    //    cur_fit_->coeff_tmp_(i - 1) = coeff(i - 1);  // Restore the prev coefficent
    //  }
    //  // Positive step
    //  cur_fit_->coeff_tmp_(i) = coeff(i) + delta_coeff_[i % HAND_NUM_COEFF];
    //  HandModel::renormalizeCoeffs(cur_fit_->coeff_tmp_.data());
    //  float f_p = objectiveFunc(cur_fit_->coeff_tmp_);

    //  // negative step
    //  cur_fit_->coeff_tmp_(i) = coeff(i) - delta_coeff_[i % HAND_NUM_COEFF];
    //  HandModel::renormalizeCoeffs(cur_fit_->coeff_tmp_.data());
    //  float f_n = objectiveFunc(cur_fit_->coeff_tmp_);

    //  jacob(i) = (f_p - f_n) / (2.0f * delta_coeff_[i % HAND_NUM_COEFF]);
    //}
  }

  float HandFit::calcPenalty() {
    return calcPenalty(coeff_);
  }

  float HandFit::calcPenalty(const Eigen::MatrixXf& coeff) {
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

  float HandFit::calcDistPenalty(const float* coeff0, const float* coeff1) {
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

  float HandFit::evaluateFuncAndCalculateResidual() {
    MatrixXf x;
    MatrixXf y;
    MatrixXf f;
    evalFunc(f, coeff_, x);
    return calculateResidual(y, f, coeff_);
  }

  float HandFit::objectiveFunc(const MatrixXf& coeff) {
    MatrixXf x;
    MatrixXf y;
    MatrixXf f;
    evalFunc(f, coeff, x);
    return calculateResidual(y, f, coeff);
  }

  void HandFit::objectiveFuncTiled(jtil::data_str::Vector<float>& residues, 
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

  void HandFit::manualFit() {
    if ((MANUAL_FIT_SAMPLE_SIZE % NTILES) != 0) {
      throw runtime_error(string("manualFit() - ERROR: num samples is not a") +
        string(" multiple of NTILES"));
    }
    if (tiled_coeffs_.capacity() < NTILES) {
      tiled_coeffs_.capacity(NTILES);
    }
    // Try and "touch-up" the coeff values by manually tuning the parameters
    // independantly.
    float sample_radius = MANUAL_FIT_SAMPLE_RADIUS;
    float pos_sample_size = MANUAL_FIT_POS_RADIUS / 
      static_cast<float>((MANUAL_FIT_SAMPLE_SIZE / 2) - 1);
    for (uint32_t cur_repeat = 0; cur_repeat < MANUAL_FIT_NUM_REPEATS; cur_repeat++) {
      Eigen::MatrixXf c_best_ = coeff_;
      float residual_best = objectiveFunc(coeff_);

      for (uint32_t i = 0; i < coeff_dim_; i++) {
        if (i > 0) {
          coeff_ = c_best_;
        }

        uint32_t cur_coeff = manual_fit_order_[i % HAND_NUM_COEFF];
        uint32_t cur_hand_coeff = cur_coeff * (i / HAND_NUM_COEFF);
        float coeff_i_best = coeff_(cur_hand_coeff);
        tiled_coeffs_.resize(0);

        // Sample around the current value --> Accumulate the coeffs into the
        // vector for tiled / parallel evaluation
        for (int sample = -MANUAL_FIT_SAMPLE_SIZE/2; sample < 
          MANUAL_FIT_SAMPLE_SIZE/2; sample++) {

          float cur_coeff_offset;
          if (cur_coeff <= HAND_POS_Z && static_cast<int>(cur_coeff) >= HAND_POS_X) {
            cur_coeff_offset = static_cast<float>(sample) * pos_sample_size;
          } else {
            cur_coeff_offset = static_cast<float>(sample) *
              (coeff_max_limit[cur_coeff] - coeff_min_limit[cur_coeff]) * 
              sample_radius;
          }
          coeff_(cur_hand_coeff) = c_best_(cur_hand_coeff) + cur_coeff_offset;
          HandModel::renormalizeCoeffs(coeff_.data());
          tiled_coeffs_.pushBack(coeff_); 

          if (tiled_coeffs_.size() == NTILES) {
            // Array is full, evaluate the points
            objectiveFuncTiled(tiled_residues_, tiled_coeffs_);
            for (uint32_t j = 0; j < tiled_residues_.size(); j++) {
              if (tiled_residues_[j] < residual_best) {
                residual_best = tiled_residues_[j];
                coeff_i_best = tiled_coeffs_[j](cur_hand_coeff);
              }
            }
            tiled_coeffs_.resize(0);
          }
        }

        c_best_(cur_hand_coeff) = coeff_i_best;
      }

      coeff_ = c_best_;
      sample_radius = sample_radius * MANUAL_FIT_RADIUS_DECAY;
      pos_sample_size = pos_sample_size * MANUAL_FIT_RADIUS_DECAY; 
    }

    float residual = objectiveFunc(coeff_);
    cout << "Manual fit exit with residual " << residual << endl;
  }

  // delta_coeff_ is the offset used for central difference when calculating the
  // jacobian.
  const float HandFit::delta_coeff_[HAND_NUM_COEFF] = {
    1.0f,    // HAND_POS_X
    1.0f,    // HAND_POS_Y
    1.0f,    // HAND_POS_Z
    0.02f,   // HAND_ORIENT_X
    0.02f,   // HAND_ORIENT_Y
    0.02f,   // HAND_ORIENT_Z
    0.01f,   // WRIST_THETA
    0.01f,   // WRIST_PHI
    0.01f,   // THUMB_THETA
    0.01f,   // THUMB_PHI
    0.01f,   // THUMB_K1_THETA
    0.01f,   // THUMB_K1_PHI
    0.01f,   // THUMB_K2_PHI
    0.01f,   // F0_ROOT_THETA
    0.01f,   // F0_ROOT_PHI
    0.01f,   // F0_THETA
    0.01f,   // F0_PHI
    0.01f,   // F0_KNUCKLE_MID
    0.01f,   // F0_KNUCKLE_END
    0.01f,   // F1_ROOT_THETA
    0.01f,   // F1_ROOT_PHI
    0.01f,   // F1_THETA
    0.01f,   // F1_PHI
    0.01f,   // F1_KNUCKLE_MID
    0.01f,   // F1_KNUCKLE_END
    0.01f,   // F2_ROOT_THETA
    0.01f,   // F2_ROOT_PHI
    0.01f,   // F2_THETA
    0.01f,   // F2_PHI
    0.01f,   // F2_KNUCKLE_MID
    0.01f,   // F2_KNUCKLE_END
    0.01f,   // F3_ROOT_THETA
    0.01f,   // F3_ROOT_PHI
    0.01f,   // F3_THETA
    0.01f,   // F3_PHI
    0.01f,   // F3_KNUCKLE_MID
    0.01f,   // F3_KNUCKLE_END
#ifdef FIT_TWIST
    0.01f,  // F0_TWIST
    0.01f,  // F1_TWIST
    0.01f,  // F2_TWIST
    0.01f,  // F3_TWIST
    0.01f,  // THUMB_TWIST
#endif
  };
  
  // preturb_coeff_ is the std of the normally dist. random number added to each
  // coefficient on random restarts.
  // Note: 0.035rad -> Roughly 2 degrees
  const float HandFit::preturb_coeff_[HAND_NUM_COEFF] = {
    1.0f,    // HAND_POS_X
    1.0f,    // HAND_POS_Y
    1.0f,    // HAND_POS_Z
    0.02f,  // HAND_ORIENT_X
    0.02f,  // HAND_ORIENT_Y
    0.02f,  // HAND_ORIENT_Z
    0.01f,  // WRIST_THETA
    0.01f,  // WRIST_PHI
    0.01f,  // THUMB_THETA
    0.01f,  // THUMB_PHI
    0.01f,  // THUMB_K1_THETA
    0.01f,  // THUMB_K1_PHI
    0.01f,  // THUMB_K2_PHI
    0.01f,  // F0_ROOT_THETA
    0.01f,  // F0_ROOT_PHI
    0.01f,  // F0_THETA
    0.01f,  // F0_PHI
    0.01f,  // F0_KNUCKLE_MID
    0.01f,  // F0_KNUCKLE_END
    0.01f,  // F1_ROOT_THETA
    0.01f,  // F1_ROOT_PHI
    0.01f,  // F1_THETA
    0.01f,  // F1_PHI
    0.01f,  // F1_KNUCKLE_MID
    0.01f,  // F1_KNUCKLE_END
    0.01f,  // F2_ROOT_THETA
    0.01f,  // F2_ROOT_PHI
    0.01f,  // F2_THETA
    0.01f,  // F2_PHI
    0.01f,  // F2_KNUCKLE_MID
    0.01f,  // F2_KNUCKLE_END
    0.01f,  // F3_ROOT_THETA
    0.01f,  // F3_ROOT_PHI
    0.01f,  // F3_THETA
    0.01f,  // F3_PHI
    0.01f,  // F3_KNUCKLE_MID
    0.01f,  // F3_KNUCKLE_END
#ifdef FIT_TWIST
    0.01f,  // F0_TWIST
    0.01f,  // F1_TWIST
    0.01f,  // F2_TWIST
    0.01f,  // F3_TWIST
    0.01f,  // THUMB_TWIST
#endif
  };
  
  // angle_coeffs are boolean values indicating if the coefficient represents
  // a pure angle (0 --> 2pi)
  const bool HandFit::angle_coeffs[HAND_NUM_COEFF*2] = {
    // Hand 1
    false,  // HAND_POS_X
    false,  // HAND_POS_Y
    false,  // HAND_POS_Z
    true,  // HAND_ORIENT_X
    true,  // HAND_ORIENT_Y
    true,  // HAND_ORIENT_Z
    true,   // WRIST_THETA
    true,   // WRIST_PHI
    true,   // THUMB_THETA
    true,   // THUMB_PHI
    true,   // THUMB_K1_THETA
    true,   // THUMB_K1_PHI
    true,   // THUMB_K2_PHI
    true,   // F0_ROOT_THETA
    true,   // F0_ROOT_PHI
    true,   // F0_THETA
    true,   // F0_PHI
    true,   // F0_KNUCKLE_MID
    true,   // F0_KNUCKLE_END
    true,   // F1_ROOT_THETA
    true,   // F1_ROOT_PHI
    true,   // F1_THETA
    true,   // F1_PHI
    true,   // F1_KNUCKLE_MID
    true,   // F1_KNUCKLE_END
    true,   // F2_ROOT_THETA
    true,   // F2_ROOT_PHI
    true,   // F2_THETA
    true,   // F2_PHI
    true,   // F2_KNUCKLE_MID
    true,   // F2_KNUCKLE_END
    true,   // F3_ROOT_THETA
    true,   // F3_ROOT_PHI
    true,   // F3_THETA
    true,   // F3_PHI
    true,   // F3_KNUCKLE_MID
    true,   // F3_KNUCKLE_END
#ifdef FIT_TWIST
    true,   // F0_TWIST
    true,   // F1_TWIST
    true,   // F2_TWIST
    true,   // F3_TWIST
    true,   // THUMB_TWIST
#endif
    // Hand 2
    false,  // HAND_POS_X
    false,  // HAND_POS_Y
    false,  // HAND_POS_Z
    true,  // HAND_ORIENT_X
    true,  // HAND_ORIENT_Y
    true,  // HAND_ORIENT_Z
    true,   // WRIST_THETA
    true,   // WRIST_PHI
    true,   // THUMB_THETA
    true,   // THUMB_PHI
    true,   // THUMB_K1_THETA
    true,   // THUMB_K1_PHI
    true,   // THUMB_K2_PHI
    true,   // F0_ROOT_THETA
    true,   // F0_ROOT_PHI
    true,   // F0_THETA
    true,   // F0_PHI
    true,   // F0_KNUCKLE_MID
    true,   // F0_KNUCKLE_END
    true,   // F1_ROOT_THETA
    true,   // F1_ROOT_PHI
    true,   // F1_THETA
    true,   // F1_PHI
    true,   // F1_KNUCKLE_MID
    true,   // F1_KNUCKLE_END
    true,   // F2_ROOT_THETA
    true,   // F2_ROOT_PHI
    true,   // F2_THETA
    true,   // F2_PHI
    true,   // F2_KNUCKLE_MID
    true,   // F2_KNUCKLE_END
    true,   // F3_ROOT_THETA
    true,   // F3_ROOT_PHI
    true,   // F3_THETA
    true,   // F3_PHI
    true,   // F3_KNUCKLE_MID
    true,   // F3_KNUCKLE_END
#ifdef FIT_TWIST
    true,   // F0_TWIST
    true,   // F1_TWIST
    true,   // F2_TWIST
    true,   // F3_TWIST
    true,   // THUMB_TWIST
#endif
  };
  
  // coeff_min_limit is the minimum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float HandFit::coeff_min_limit[HAND_NUM_COEFF] = {
    -std::numeric_limits<float>::infinity(),    // HAND_POS_X
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    -3.14159f,  // HAND_ORIENT_X
    -3.14159f,  // HAND_ORIENT_Y
    -3.14159f,  // HAND_ORIENT_Z
    -0.903f,  // WRIST_THETA
    -1.580f,  // WRIST_PHI
    -0.523f,  // THUMB_THETA
    -0.523f,  // THUMB_PHI
    -0.633f,  // THUMB_K1_THETA
    -1.253f,  // THUMB_K1_PHI
    -1.733f,  // THUMB_K2_PHI
    -0.700f,  // F0_ROOT_THETA
    -0.700f,  // F0_ROOT_PHI
    -0.800f,  // F0_THETA
    -1.443f,  // F0_PHI
    -1.800f,  // F0_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.800f,  // F0_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.700f,  // F1_ROOT_THETA
    -0.700f,  // F1_ROOT_PHI
    -0.800f,  // F1_THETA
    -1.443f,  // F1_PHI
    -1.563f,  // F1_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.563f,  // F1_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.700f,  // F2_ROOT_THETA
    -0.700f,  // F2_ROOT_PHI
    -0.800f,  // F2_THETA
    -1.443f,  // F2_PHI
    -1.800f,  // F2_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.800f,  // F2_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.700f,  // F3_ROOT_THETA
    -0.700f,  // F3_ROOT_PHI
    -0.800f,  // F3_THETA
    -1.443f,  // F3_PHI
    -1.800f,  // F3_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.800f,  // F3_KNUCKLE_END  // Formally -1.363 4/12/2013
#ifdef FIT_TWIST
    -0.300f,  // F0_TWIST
    -0.400f,  // F1_TWIST
    -0.300f,  // F2_TWIST
    -0.300f,  // F3_TWIST
    -0.300f,  // THUMB_TWIST
#endif
  };
  
  // coeff_max_limit is the maximum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float HandFit::coeff_max_limit[HAND_NUM_COEFF] = {
    std::numeric_limits<float>::infinity(),    // HAND_POS_X
    std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    3.14159f,  // HAND_ORIENT_X
    3.14159f,  // HAND_ORIENT_Y
    3.14159f,  // HAND_ORIENT_Z
    0.905f,  // WRIST_THETA
    1.580f,  // WRIST_PHI
    0.550f,  // THUMB_THETA
    0.580f,  // THUMB_PHI
    0.700f,  // THUMB_K1_THETA
    0.750f,  // THUMB_K1_PHI
    0.500f,  // THUMB_K2_PHI
    0.700f,  // F0_ROOT_THETA
    0.700f,  // F0_ROOT_PHI
    0.600f,  // F0_THETA
    0.670f,  // F0_PHI
    0.560f,  // F0_KNUCKLE_MID
    0.560f,  // F0_KNUCKLE_END
    0.700f,  // F1_ROOT_THETA
    0.700f,  // F1_ROOT_PHI
    0.600f,  // F1_THETA
    0.670f,  // F1_PHI
    0.560f,  // F1_KNUCKLE_MID
    0.560f,  // F1_KNUCKLE_END
    0.700f,  // F2_ROOT_THETA
    0.700f,  // F2_ROOT_PHI
    0.600f,  // F2_THETA
    0.670f,  // F2_PHI
    0.560f,  // F2_KNUCKLE_MID
    0.560f,  // F2_KNUCKLE_END
    0.700f,  // F3_ROOT_THETA
    0.700f,  // F3_ROOT_PHI
    0.600f,  // F3_THETA
    0.670f,  // F3_PHI
    0.560f,  // F3_KNUCKLE_MID
    0.560f,  // F3_KNUCKLE_END
#ifdef FIT_TWIST
    0.300f,  // F0_TWIST
    0.300f,  // F1_TWIST
    0.300f,  // F2_TWIST
    0.300f,  // F3_TWIST
    0.300f,  // THUMB_TWIST
#endif
  };

  const uint32_t HandFit::manual_fit_order_[HAND_NUM_COEFF] = {
    HAND_POS_X,
    HAND_POS_Y,
    HAND_POS_Z,
    HAND_ORIENT_X,
    HAND_ORIENT_Y,
    HAND_ORIENT_Z,
    WRIST_THETA,
    WRIST_PHI,
    THUMB_THETA,
    THUMB_PHI,
    THUMB_K1_THETA,
    THUMB_K1_PHI,
    THUMB_K2_PHI,
#ifdef FIT_TWIST
    F0_TWIST,
    F1_TWIST,
    F2_TWIST,
    F3_TWIST,
    THUMB_TWIST,
#endif
    F0_ROOT_THETA,
    F1_ROOT_THETA,
    F2_ROOT_THETA,
    F3_ROOT_THETA,
    F0_ROOT_PHI,
    F1_ROOT_PHI,
    F2_ROOT_PHI,
    F3_ROOT_PHI,
    F0_THETA,
    F1_THETA,
    F2_THETA,
    F3_THETA,
    F0_PHI,
    F1_PHI,
    F2_PHI,
    F3_PHI,
    F0_KNUCKLE_MID,
    F1_KNUCKLE_MID,
    F2_KNUCKLE_MID,
    F3_KNUCKLE_MID,
    F0_KNUCKLE_END,
    F1_KNUCKLE_END,
    F2_KNUCKLE_END,
    F3_KNUCKLE_END,
  };
  
  // coeff_penalty_scale_ is the exponential scale to use when penalizing coeffs
  // outside the min and max values.
  const float HandFit::coeff_penalty_scale_[HAND_NUM_COEFF] = {
    0,    // HAND_POS_X
    0,    // HAND_POS_Y
    0,    // HAND_POS_Z
    0,  // HAND_ORIENT_X
    0,  // HAND_ORIENT_Y
    0,  // HAND_ORIENT_Z
    100,  // WRIST_THETA
    100,  // WRIST_PHI
    100,  // THUMB_THETA
    100,  // THUMB_PHI
    100,  // THUMB_K1_THETA
    100,  // THUMB_K1_PHI
    100,  // THUMB_K2_PHI
    100,  // F0_ROOT_THETA
    100,  // F0_ROOT_PHI
    100,  // F0_THETA
    100,  // F0_PHI
    100,  // F0_KNUCKLE_MID
    100,  // F0_KNUCKLE_END
    100,  // F1_ROOT_THETA
    100,  // F1_ROOT_PHI
    100,  // F1_THETA
    100,  // F1_PHI
    100,  // F1_KNUCKLE_MID
    100,  // F1_KNUCKLE_END
    100,  // F2_ROOT_THETA
    100,  // F2_ROOT_PHI
    100,  // F2_THETA
    100,  // F2_PHI
    100,  // F2_KNUCKLE_MID
    100,  // F2_KNUCKLE_END
    100,  // F3_ROOT_THETA
    100,  // F3_ROOT_PHI
    100,  // F3_THETA
    100,  // F3_PHI
    100,  // F3_KNUCKLE_MID
    100,  // F3_KNUCKLE_END
#ifdef FIT_TWIST
    100,  // F0_TWIST
    100,  // F1_TWIST
    100,  // F2_TWIST
    100,  // F3_TWIST
    100,  // THUMB_TWIST
#endif
  };

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
