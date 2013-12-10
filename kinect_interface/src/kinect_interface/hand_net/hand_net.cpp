#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <cmath>
#include <sstream>
#include "kinect_interface/hand_net/hand_net.h"
#include "kinect_interface/hand_net/hand_image_generator.h"
#include "jtorch/torch_stage.h"
#include "jtorch/parallel.h"
#include "jtorch/sequential.h"
#include "jtorch/table.h"
#include "jtorch/tensor.h"
#include "kinect_interface/hand_net/hand_model_coeff.h"  // for HandCoeff
#include "kinect_interface/hand_net/hand_model.h"  // for HandModel
#include "kinect_interface/open_ni_funcs.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"  // GDT_MAX_DIST
#include "jtil/image_util/image_util.h"
#include "jtil/data_str/vector.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/renderer/renderer.h"
#include "jtil/renderer/geometry/geometry_manager.h"
#include "jtil/threading/thread_pool.h"
#include "jtil/math/lm_fit.h"
#include "jtil/math/bfgs.h"
#include "jtil/math/pso_parallel.h"
#include "jtil/clk/clk.h"
#include "jtil/renderer/camera/camera.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using jtil::threading::ThreadPool;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using jtil::math::Float3;
using namespace jtil::image_util;
using kinect_interface::hand_net::HandCoeff;
using namespace jtil::math;
using namespace jtorch;
using namespace jtil::renderer;
using namespace jtil::data_str;

namespace kinect_interface {
namespace hand_net {

  HandNet* HandNet::g_hand_net_;
 
  HandNet::HandNet() {
    conv_network_ = NULL;
    image_generator_ = NULL;
    heat_map_convnet_ = NULL;
    lm_fit_x_vals_ = NULL;
    hm_temp_ = NULL;
    gauss_coeff_ = NULL;
    gauss_coeff_hm_ = NULL;
    heat_map_size_ = 0;
    num_output_features_ = 0;
    rest_pose_ = NULL;
    rhand_cur_pose_ = NULL;
    rhand_prev_pose_ = NULL;
    rhand_ = NULL;
    heat_map_lm_ = NULL;
    camera_ = NULL;
    bfgs_ = NULL;
    pso_ = NULL;
    hand_size_ = 1.0f;
  }

  HandNet::~HandNet() {
    releaseData();
  }

  void HandNet::releaseData() {
    SAFE_DELETE(conv_network_);
    SAFE_DELETE(image_generator_);
    SAFE_DELETE_ARR(heat_map_convnet_);
    SAFE_DELETE_ARR(gauss_coeff_);
    SAFE_DELETE_ARR(gauss_coeff_hm_);
    SAFE_DELETE_ARR(hm_temp_);
    SAFE_DELETE_ARR(lm_fit_x_vals_);
    SAFE_DELETE(rest_pose_);
    SAFE_DELETE(rhand_cur_pose_);
    SAFE_DELETE(rhand_prev_pose_);
    SAFE_DELETE(rhand_);
    SAFE_DELETE(heat_map_lm_);
    SAFE_DELETE(camera_);
    SAFE_DELETE(bfgs_);
    SAFE_DELETE(pso_);
  }

  void HandNet::loadFromFile(const std::string& filename) {
    if (jtorch::cl_context == NULL) {
      throw std::wruntime_error("HandNet::loadFromFile() - "
        "ERROR: jtorch has not been initialized!"); 
    }
    releaseData();

    std::cout << "loading HandNet from " << filename << std::endl;

    conv_network_ = TorchStage::loadFromFile(filename);

    // Check the basic structure...
    if (conv_network_->type() != TorchStageType::SEQUENTIAL_STAGE) {
      throw std::wruntime_error("HandNet::loadFromFile() - ERROR: "
        "Convnet structure may be corrupt!");
    }
    Sequential* network = (Sequential*)conv_network_;
    if (network->get(0)->type() != TorchStageType::PARALLEL_STAGE) {
      throw std::wruntime_error("HandNet::loadFromFile() - ERROR: "
        "Convnet structure may be corrupt!");
   } 
    Parallel* banks = (Parallel*)network->get(0);

    num_conv_banks_ = (int32_t)banks->numBanks();
    image_generator_ = new HandImageGenerator(num_conv_banks_);

    // Do one forward prop to load all the kernels and figure out the output
    // size (of the heat maps)
    TorchData* im = (TorchData*)image_generator_->hand_image();
    conv_network_->forwardProp(*im);
    Tensor<float>* output_tensor = (Tensor<float>*)(conv_network_->output);
    uint32_t data_size = output_tensor->dataSize();
    heat_map_convnet_ = new float[data_size];
    num_output_features_ = (HAND_NUM_COEFF_CONVNET / FEATURE_SIZE);
    heat_map_size_ = (uint32_t)sqrtf((float)(data_size / num_output_features_));
    // Check that the heatmap is square:
    if (heat_map_size_ * heat_map_size_ * num_output_features_ != data_size) {
      throw std::wruntime_error("HandNet::loadFromFile() - ERROR: Heat map"
        "size is not what we expect!");
    }

    hm_temp_ = new float[heat_map_size_ * heat_map_size_];
    gauss_coeff_ = new float[NUM_COEFFS_PER_GAUSSIAN * num_output_features_];
    gauss_coeff_hm_ = new float[NUM_COEFFS_PER_GAUSSIAN * num_output_features_];
    lm_fit_x_vals_ = new float[heat_map_size_ * heat_map_size_ * X_DIM_LM_FIT];
    for (uint32_t v = 0, i = 0; v < heat_map_size_; v++) {
      for (uint32_t u = 0; u < heat_map_size_; u++, i++) {
        lm_fit_x_vals_[i * X_DIM_LM_FIT] = (float)u;
        lm_fit_x_vals_[i * X_DIM_LM_FIT + 1] = (float)v;
      }
    }
    heat_map_lm_ = new LMFit<float>(NUM_COEFFS_PER_GAUSSIAN, X_DIM_LM_FIT,
      heat_map_size_ * heat_map_size_);
    bfgs_ = new BFGS<double>(BFGSHandCoeff::BFGS_NUM_PARAMETERS);
    pso_ = new PSOParallel(BFGSHandCoeff::BFGS_NUM_PARAMETERS, HN_PSO_SWARM_SIZE,
      HN_PSO_SWARM_SIZE);
    bfgs_->max_iterations = 100;
    rest_pose_ = new HandModelCoeff(HandType::RIGHT);
    rhand_cur_pose_ = new HandModelCoeff(HandType::RIGHT);
    rhand_prev_pose_ = new HandModelCoeff(HandType::RIGHT);
    rest_pose_->loadFromFile("./", "coeff_hand_rest_pose.bin");
    resetTracking();

    setPSORadius();

    // Initialize the camera
    FloatQuat eye_rot; eye_rot.identity();
    Float3 eye_pos(0, 0, 0);
    float fov_vert_deg = 360.0f * OpenNIFuncs::fVFOV_primesense_109 / 
      (2.0f * (float)M_PI);
    camera_ = new jtil::renderer::Camera(eye_rot, eye_pos, src_width, 
      src_height, fov_vert_deg, -10.0f, -3000.0f);
    camera_->updateProjection();
    camera_->updateView();  // THIS SHOULD BE FIRST, BUT IT BREAKS IF IT IS!
    for (uint32_t i = 0; i < 16; i++) {
      pv_mat_[i] = (double)camera_->proj_view()[i];
    }

  }

  void HandNet::calcHandImage(const int16_t* depth, const uint8_t* label) {
    image_generator_->calcHandImage(depth, label);
  }

  void HandNet::loadHandModels() {
    rhand_ = new HandModel(HandType::RIGHT);
    rhand_->updateMatrices(rest_pose_->coeff(), hand_size_);
  }

  void HandNet::setModelVisibility(const bool visible) {
    rhand_->setRenderVisiblity(visible);
  }

  void HandNet::setHandSize(const float size) {
    hand_size_ = size;
  }

  void HandNet::calcConvnetHeatMap(const int16_t* depth, 
    const uint8_t* label) {
    if (conv_network_ == NULL || image_generator_ == NULL) {
      std::cout << "HandNet::calcHandCoeff() - ERROR: Convnet not loaded";
      std::cout << " from file!" << std::endl;
    }

    calcHandImage(depth, label);  // Creates HPF hand image

    // Copy over the hand images in the input data structures
    TorchData* im = image_generator_->hpf_hand_image();

    // Now propogate through the network
    conv_network_->forwardProp(*im);
    //jtorch::cl_context->sync(jtorch::deviceid);  // Not necessary
    Tensor<float>* output_tensor = (Tensor<float>*)(conv_network_->output);
    output_tensor->getData(heat_map_convnet_);

    // For each of the heat maps, fit a gaussian to it
    const Int4* pos_wh = &image_generator_->hand_pos_wh();
    for (uint32_t i = 0; i < num_output_features_; i++) {
      uint32_t istart = i * NUM_COEFFS_PER_GAUSSIAN;
      calcGaussDistCoeff(&gauss_coeff_hm_[istart], 
        &heat_map_convnet_[i * heat_map_size_ * heat_map_size_]);
      // Transform the gaussian into the kinect image space (just a viewport
      // transform!):
      gauss_coeff_[istart + GaussMeanU] = 
        gauss_coeff_hm_[istart + GaussMeanU] * (float)(*pos_wh)[2] + (float)(*pos_wh)[0];
      gauss_coeff_[istart + GaussMeanV] = 
        gauss_coeff_hm_[istart + GaussMeanV] * (float)(*pos_wh)[3] + (float)(*pos_wh)[1];
      gauss_coeff_[istart + GaussVarU] = 
        gauss_coeff_hm_[istart + GaussVarU] * (float)(*pos_wh)[2];
      gauss_coeff_[istart + GaussVarV] = 
        gauss_coeff_hm_[istart + GaussVarV] * (float)(*pos_wh)[3];
    }
  }



  void HandNet::renormalizeBFGSCoeffs(double* coeff) {
    // Set all angles 0 --> 2pi
    for (uint32_t i = BFGS_HAND_ORIENT_X; i < BFGS_NUM_PARAMETERS; i++) {
      WrapTwoPI(coeff[i]);
    }
  }

  void HandNet::renormalizePSOCoeffs(float* coeff) {
    // Set all angles 0 --> 2pi
    for (uint32_t i = BFGS_HAND_ORIENT_X; i < BFGS_NUM_PARAMETERS; i++) {
      WrapTwoPI(coeff[i]);
    }
  }

  void HandNet::resetTracking() {
    rhand_prev_pose_->copyCoeffFrom(rest_pose_);
    rhand_cur_pose_->copyCoeffFrom(rest_pose_);
  }


//#define USE_BFGS
#define USE_PSO

  void HandNet::calcConvnetPose(const int16_t* depth, const uint8_t* label,
    const float smoothing_factor, const uint64_t max_pso_iterations) {
    // Try fitting in projected space from the rest pose:
    rhand_prev_pose_->copyCoeffFrom(rhand_cur_pose_);
    g_hand_net_ = this;

    calc3DPos(depth, label);

#ifdef USE_PSO
    // The objfunc parameters are a sub-set, make a copy of the current pose 
    // into this smaller space
    HandCoeffToBFGSHandCoeff<float>(pso_coeff_start_, rhand_cur_pose_->coeff());
    pso_->verbose = false;
    pso_->delta_coeff_termination = 1e-4f;
    pso_->max_iterations = max_pso_iterations;
    pso_->minimize(pso_coeff_end_, pso_coeff_start_, pso_radius_, 
      HandModel::angle_coeffs(), objFuncParallel, HandNet::renormalizePSOCoeffs);
    BFGSHandCoeffToHandCoeff<float>(rhand_cur_pose_->coeff(), pso_coeff_end_);
#endif

#ifdef USE_BFGS
    // The objfunc parameters are a sub-set, make a copy of the current pose 
    // into this smaller space
    for (uint32_t i = 0; i < HandCoeff::NUM_PARAMETERS; i++) {
      cur_double_coeff[i] = (double)rhand_cur_pose_->coeff()[i];
    }
    HandCoeffToBFGSHandCoeff<double>(bfgs_coeff_start_, cur_double_coeff);
    bfgs_->verbose = false;
    bfgs_->delta_f_term = 1e-21;
    bfgs_->delta_x_2norm_term = 1e-21;
    bfgs_->jac_2norm_term = 1e-12;
    bfgs_->max_iterations = 150;
    bfgs_->descent_cond = jtil::math::SufficientDescentCondition::ARMIJO;
    bfgs_->c1 = 1e-12;
    bfgs_->minimize(bfgs_coeff_end_, bfgs_coeff_start_, 
      HandModel::angle_coeffs(), objFunc, jacobFunc, 
      HandNet::renormalizeBFGSCoeffs);
    BFGSHandCoeffToHandCoeff<double>(cur_double_coeff, bfgs_coeff_end_);
    for (uint32_t i = 0; i < HandCoeff::NUM_PARAMETERS; i++) {
      rhand_cur_pose_->coeff()[i] = (float)cur_double_coeff[i];
    }
#endif

    // Some better ideas here: 
    // http://msdn.microsoft.com/en-us/library/jj131429.aspx
    if (smoothing_factor > 0) {
      float* prev_coeff = rhand_prev_pose_->coeff();
      float* curr_coeff = rhand_cur_pose_->coeff();
      const float scale_fact_a = (1.0f - smoothing_factor);
      const float scale_fact_b = smoothing_factor;
      const bool* angle_coeffs = rhand_->angle_coeffs();
      for (uint32_t i = 0; i < HAND_NUM_COEFF; i++) {
        if (angle_coeffs[i]) {
          // The current coefficient is an angle and needs to be interpolated
          // correctly
          float real_a = cosf(curr_coeff[i]);
          float imag_a = sinf(curr_coeff[i]);
          float real_b = cosf(prev_coeff[i]);
          float imag_b = sinf(prev_coeff[i]);
          float real_interp = real_a * scale_fact_a + real_b * scale_fact_b;
          float imag_interp = imag_a * scale_fact_a + imag_b * scale_fact_b;
          float interp_angle = atan2(imag_interp, real_interp);
          curr_coeff[i] = interp_angle;
        } else {
          curr_coeff[i] = scale_fact_a * curr_coeff[i] + 
            scale_fact_b * prev_coeff[i];
        }
      }
    }

    rhand_->updateMatrices(rhand_cur_pose_->coeff(), hand_size_);
    rhand_->updateHeirachyMatrices();
  }

  void HandNet::calc3DPos(const int16_t* depth, const uint8_t* label) {
    // For each feature, find the Z point
    // If non exists (or it is not part of the hand) then D is the same as last
    // frame
    for (uint32_t i = 0; i < num_convnet_feats; i++) {
      float u = gauss_coeff_[i * NUM_COEFFS_PER_GAUSSIAN + GaussMeanU];
      float v = gauss_coeff_[i * NUM_COEFFS_PER_GAUSSIAN + GaussMeanV];
      uvd_pos_[i * 3] = u;
      uvd_pos_[i * 3 + 1] = v;
      int16_t d = MAX_INT16;

      //// Now search for some small window for the minimum point
      //uint32_t v_start = std::max<uint32_t>((uint32_t)floor(v) - RAD_UVD_SEARCH, 0);
      //uint32_t v_end = std::min<uint32_t>((uint32_t)floor(v) + RAD_UVD_SEARCH, src_height-1);
      //for (uint32_t v_search = v_start; v_search <= v_end; v_search++) {
      //  uint32_t u_start = std::max<uint32_t>((uint32_t)floor(u) - RAD_UVD_SEARCH, 0);
      //  uint32_t u_end = std::min<uint32_t>((uint32_t)floor(u) + RAD_UVD_SEARCH, src_width-1);
      //  for (uint32_t u_search = u_start; u_search <= u_end; u_search++) {
      //    uint32_t index = v_search * src_height + u_search;
      //    if (depth[index] > 0 && label[index] == 1) {
      //      d = std::min<int16_t>(depth[index], d);
      //    }
      //  }
      //}

      // No search, just grab the point
      uint32_t v_ind = std::min<uint32_t>((uint32_t)std::max<int32_t>((int32_t)floor(v), 0), src_height - 1);
      uint32_t u_ind = std::min<uint32_t>((uint32_t)std::max<int32_t>((int32_t)floor(u), 0), src_width - 1);
      uint32_t index = v_ind * src_height + u_ind;
      if (label[index] == 1) {
        d = depth[index];
      } 

      if (d == MAX_INT16) {
        uvd_pos_[i * 3 + 2] = 0;
      } else {
        if (i < 3 || i == THUMB_KNU_U / FEATURE_SIZE) {
          // Push the depth back a little bit (to account for sphere in hand)
          d += 20;  // 2.0cm for palm and thumb knuckle positions
        } else {
          d += 5;  // 0.5cm for all other positions
        }
        uvd_pos_[i * 3 + 2] = d;
      }
    }
  }

  const float* HandNet::hpf_hand_image() const {
    return image_generator_->hpf_hand_image_cpu();
  }

  const float* HandNet::hand_image() const {
    return image_generator_->hpf_hand_image_cpu();
  }

  const int32_t HandNet::size_images() const {
    Table* im_banks = image_generator_->hand_image();
    uint32_t size = 0;
    for (uint32_t i = 0; i < im_banks->tableSize(); i++) {
      size += (*im_banks)(i)->dataSize();
    }
    return size;
  } 

  const jtil::math::Float3& HandNet::uvd_com() const {
    return image_generator_->uvd_com();
  }

  // calcGaussDistCoeff - This is a replica of test_gauss_fit.m in C code
  void HandNet::calcGaussDistCoeff(float* gauss_coeff, const float* im_data) {
    // First calculate the weighted mean and var, which will be a seed for the
    // non-linear fit.  Note that variance and mean will be thrown off by
    // outliers: which is why we want a least-sqs fit, which tends to find a
    // better gaussian.
    const uint32_t im_size = heat_map_size_ * heat_map_size_;
    jtil::math::Float2 mean;
    jtil::math::Float2 var;

    // Normalize the heatmap so that the sum of the weights is 1 and weights 
    // are all positive
    float sum_weights = 0;
    for (uint32_t i = 0; i < im_size; i++) {
      sum_weights += fabsf(im_data[i]);
    }
    if (sum_weights < LOOSE_EPSILON) {
      throw std::wruntime_error("HandNet::calcGaussDistCoeff() - ERROR: "
        "Heat map sum is zero!");
    }
    for (uint32_t i = 0; i < im_size; i++) {
      hm_temp_[i] = fabsf(im_data[i]) / sum_weights;
    }
    
    // Calculate the weighted mean
    mean.zeros();
    float max_weight = 0;
    uint32_t i = 0;
    sum_weights = 0;
    for (uint32_t v = 0; v < heat_map_size_; v++) {
      for (uint32_t u = 0; u < heat_map_size_; u++, i++) {
        if (hm_temp_[i] <= 0.01f) {
          hm_temp_[i] = 0;
        }
        sum_weights += hm_temp_[i];
        mean[0] += hm_temp_[i] * (float)u;
        mean[1] += hm_temp_[i] * (float)v;
        max_weight = std::max<float>(max_weight, hm_temp_[i]);
      }
    }
    Float2::scale(mean, 1.0f / sum_weights);

    // Calculate the weighted variance (= std^2)
    var.zeros();
    i = 0;
    uint32_t non_zero_weights = 0;
    for (uint32_t v = 0; v < heat_map_size_; v++) {
      for (uint32_t u = 0; u < heat_map_size_; u++, i++) {
        if (hm_temp_[i] > LOOSE_EPSILON) {
          non_zero_weights++;
          float du = (float)u - mean[0];
          float dv = (float)v - mean[1];
          var[0] += hm_temp_[i] * du * du;
          var[1] += hm_temp_[i] * dv * dv;
        }
      }
    }
    if (non_zero_weights <= 1) {
      gauss_coeff[GaussAmp] = 1;
      gauss_coeff[GaussMeanU] = 0.5;
      gauss_coeff[GaussMeanV] = 0.5;
      gauss_coeff[GaussVarU] = 10000.0;
      gauss_coeff[GaussVarV] = 10000.0;
    }
    Float2::scale(var, 1.0f / (sum_weights * (non_zero_weights - 1) / 
      non_zero_weights));

#if 0
    float c_start[NUM_COEFFS_PER_GAUSSIAN];
    c_start[GaussAmp] = max_weight;
    c_start[GaussMeanU] = mean[0];
    c_start[GaussMeanV] = mean[1];
    c_start[GaussVarU] = var[0];
    c_start[GaussVarV] = var[1];
    heat_map_lm_->fitModel(gauss_coeff, c_start, hm_temp_, lm_fit_x_vals_,
      gauss2D, jacobGauss2D);
    gauss_coeff[GaussMeanU] /= static_cast<float>(heat_map_size_ - 1);
    gauss_coeff[GaussMeanV] /= static_cast<float>(heat_map_size_ - 1);
    gauss_coeff[GaussVarU] /= static_cast<float>(heat_map_size_ - 1);
    gauss_coeff[GaussVarV] /= static_cast<float>(heat_map_size_ - 1);
#else
    // LM is too slow.  Just use the std and mean directly:
    gauss_coeff[GaussAmp] = max_weight;
    gauss_coeff[GaussMeanU] = mean[0] / static_cast<float>(heat_map_size_ - 1);
    gauss_coeff[GaussMeanV] = mean[1] / static_cast<float>(heat_map_size_ - 1);
    gauss_coeff[GaussVarU] = var[0] / static_cast<float>(heat_map_size_ - 1);
    gauss_coeff[GaussVarV] = var[1] / static_cast<float>(heat_map_size_ - 1);
#endif
  }

  float HandNet::gauss2D(const float* x, const float* c) {
    float du = (x[0] - c[GaussMeanU]);
    float dv = (x[1] - c[GaussMeanV]);
    return c[GaussAmp] * exp(-(du * du / (2.0f * c[GaussVarU]) + dv * dv / (2.0f * c[GaussVarV])));
  }

  double HandNet::gauss2D(const double* x, const double* c) {
    double du = (x[0] - c[GaussMeanU]);
    double dv = (x[1] - c[GaussMeanV]);
    return c[GaussAmp] * exp(-(du * du / (2.0 * c[GaussVarU]) + dv * dv / (2.0 * c[GaussVarV])));
  }

  void HandNet::jacobGauss2D(float* jacob, const float* x, const float* c) {
    float du = (x[0] - c[GaussMeanU]);
    float dv = (x[1] - c[GaussMeanV]);
    // I assume that the optimizer will clean up a lot of these redundant 
    // computations...
    jacob[0] = exp(-(du * du / (2.0f * c[GaussVarU]) + dv * dv / (2.0f * c[GaussVarV])));
    jacob[1] = (c[GaussAmp]*exp(((du * du)*(-1.0f/2.0f))/c[GaussVarU]-((dv * dv)*(1.0f/2.0f))/c[GaussVarV])*(x[0]*2.0f-c[GaussMeanU]*2.0f)*(1.0f/2.0f))/c[GaussVarU];
    jacob[2] = (c[GaussAmp]*exp(((du * du)*(-1.0f/2.0f))/c[GaussVarU]-((dv * dv)*(1.0f/2.0f))/c[GaussVarV])*(x[1]*2.0f-c[GaussMeanV]*2.0f)*(1.0f/2.0f))/c[GaussVarV];
    jacob[3] = c[GaussAmp]*1.0f/(c[GaussVarU]*c[GaussVarU]) *exp(((du * du)*(-1.0f/2.0f))/c[GaussVarU]-((dv * dv)*(1.0f/2.0f))/c[GaussVarV])*(du * du)*(1.0f/2.0f);
    jacob[4] = c[GaussAmp]*1.0f/(c[GaussVarV]*c[GaussVarV]) *exp(((du * du)*(-1.0f/2.0f))/c[GaussVarU]-((dv * dv)*(1.0f/2.0f))/c[GaussVarV])*(dv * dv)*(1.0f/2.0f);
  }

  double HandNet::quad2D(const double* x, const double* c) {
    double du = (x[0] - c[GaussMeanU]);
    double dv = (x[1] - c[GaussMeanV]);
    return du * du / (2.0 * c[GaussVarU]) + dv * dv / (2.0 * c[GaussVarV]);
  }

  double HandNet::linear2D(const double* x, const double* c) {
    double du = fabs(x[0] - c[GaussMeanU]);
    double dv = fabs(x[1] - c[GaussMeanV]);
    return du  / (2.0 * sqrt(c[GaussVarU])) + dv / (2.0 * sqrt(c[GaussVarV]));
  }

  double HandNet::objFunc(const double* bfgs_hand_coeff) {
    // Update the matrix heirachy
    BFGSHandCoeffToHandCoeff<double>(g_hand_net_->cur_double_coeff, 
      bfgs_hand_coeff);

    g_hand_net_->rhand_->updateDoubleMatrices(g_hand_net_->cur_double_coeff);
    g_hand_net_->rhand_->updateDoubleHeirachyMatrices();
    double ret_val = 0;

    if (num_convnet_feats != g_hand_net_->num_output_features_) {
      throw std::wruntime_error("HandNet::bfgsFunc() - ERROR: "
        "incorrect number of convnet features!");
    }

    // Calculate the projected sphere positions:
    // TO DO: PARALLELIZE THIS!
    Double3 uvd;
    for (uint32_t i = 0; i < num_convnet_feats; i++) {
      g_hand_net_->rhand_->calcBoundingSphereUVDPos(uvd.m, 
        convnet_sphere_indices[i], g_hand_net_->pv_mat_);

      Double3 uvd_data;
      uvd_data[0] = (double)g_hand_net_->uvd_pos_[i * 3];
      uvd_data[1] = (double)g_hand_net_->uvd_pos_[i * 3 + 1];
      uvd_data[2] = (double)g_hand_net_->uvd_pos_[i * 3 + 2];
      if (uvd_data[2] < EPSILON) {
        uvd.m[2] = uvd_data[2];  // Match in UV only
      }
      Double3 vec;
      Double3::sub(vec, uvd_data, uvd);
      ret_val += sqrt(Double3::dot(vec, vec)) * 1e-3;
    }

    return ret_val + g_hand_net_->calcPenalty(g_hand_net_->cur_double_coeff);
  }

  float HandNet::objFunc(const float* bfgs_hand_coeff) {
    // Update the matrix heirachy
    BFGSHandCoeffToHandCoeff<float>(g_hand_net_->rhand_cur_pose_->coeff(), 
      bfgs_hand_coeff);
    return objFuncInternal();
  }

  void HandNet::objFuncParallel(Vector<float>& residues, 
    Vector<float*>& coeffs) {
    for (uint32_t i = 0; i < coeffs.size(); i++) {
      BFGSHandCoeffToHandCoeff<float>(g_hand_net_->rhand_cur_pose_->coeff(), 
        coeffs[i]);
      residues[i] = objFuncInternal();
    }
  }

  float HandNet::objFuncInternal() {
    g_hand_net_->rhand_->updateMatrices(g_hand_net_->rhand_cur_pose_->coeff(), 
      g_hand_net_->hand_size_);
    g_hand_net_->rhand_->updateHeirachyMatrices();
    float ret_val = 0;

    if (num_convnet_feats != g_hand_net_->num_output_features_) {
      throw std::wruntime_error("HandNet::bfgsFunc() - ERROR: "
        "incorrect number of convnet features!");
    }

    // Calculate the projected sphere positions:
    Float3 uvd;
    for (uint32_t i = 0; i < num_convnet_feats; i++) {
      g_hand_net_->rhand_->calcBoundingSphereUVDPos(uvd.m, 
        convnet_sphere_indices[i], g_hand_net_->camera_->proj_view());

      Float3 uvd_data(&g_hand_net_->uvd_pos_[i * 3]);
      if (uvd_data[2] < EPSILON) {
        uvd.m[2] = uvd_data[2];  // Match in UV only
      }
      Float3 vec;
      Float3::sub(vec, uvd_data, uvd);
      ret_val += Float3::dot(vec, vec) * 0.000001f;
      //ret_val += sqrtf(Float3::dot(vec, vec)) * 1e-3f;
    }

    return ret_val + g_hand_net_->calcPenalty(g_hand_net_->rhand_cur_pose_->coeff());
  }

  double tmp_coeff[BFGSHandCoeff::BFGS_NUM_PARAMETERS];
  void HandNet::jacobFunc(double* jacob, const double* bfgs_hand_coeff) {
    // APPROXIMATE USING CENTRAL DIFFERENCING
    for (uint32_t i = 0; i < BFGS_NUM_PARAMETERS; i++) {
      tmp_coeff[i] = bfgs_hand_coeff[i];
    }

    const double eps = 0.00001;
    for (uint32_t i = 0; i < BFGS_NUM_PARAMETERS; i++) {
      tmp_coeff[i] = bfgs_hand_coeff[i] - eps;
      double f0 = objFunc(tmp_coeff);
      tmp_coeff[i] = bfgs_hand_coeff[i] + eps;
      double f1 = objFunc(tmp_coeff);
      tmp_coeff[i] = bfgs_hand_coeff[i];
      jacob[i] = (f1 - f0) / (2.0 * eps);
    }
  }

#define LINEAR_PENALTY

  float HandNet::calcPenalty(const float* coeff) {
    float penalty = 1.0f;
    const uint32_t coeff_dim = HAND_NUM_COEFF;
    const uint32_t coeff_dim_per_model = HAND_NUM_COEFF;
    const float* penalty_scale = kinect_interface::hand_net::HandModel::coeff_penalty_scale();
    const float* max_limit = kinect_interface::hand_net::HandModel::coeff_max_limit_conservative();
    const float* min_limit = kinect_interface::hand_net::HandModel::coeff_min_limit_conservative();

    for (uint32_t i = 0; i < coeff_dim; i++) {
      if (penalty_scale[i % coeff_dim_per_model] > EPSILON) {
        float cur_coeff_val = coeff[i];

#ifdef LINEAR_PENALTY
        // Linear penalty
        if (cur_coeff_val > max_limit[i % coeff_dim_per_model]) {
          penalty += penalty_scale[i % coeff_dim_per_model] * 
            fabsf(cur_coeff_val - max_limit[i % coeff_dim_per_model]) / 10.0f;
        }
        if (cur_coeff_val < min_limit[i % coeff_dim_per_model]) { 
          penalty += penalty_scale[i % coeff_dim_per_model] * 
            fabsf(min_limit[i % coeff_dim_per_model] - cur_coeff_val) / 10.0f;
        }
#else
        // Quadratic penalty
        if (cur_coeff_val > max_limit[i % coeff_dim_per_model]) {
          float cur_penalty = penalty_scale[i % coeff_dim_per_model] * 
            fabsf(cur_coeff_val - max_limit[i % coeff_dim_per_model]) / 2.0f;
          penalty += (cur_penalty * cur_penalty);
        }
        if (cur_coeff_val < min_limit[i % coeff_dim_per_model]) { 
          float cur_penalty = penalty_scale[i % coeff_dim_per_model] * 
            fabsf(min_limit[i % coeff_dim_per_model] - cur_coeff_val) / 2.0f;
          penalty += (cur_penalty * cur_penalty);
        }
#endif
      }
    }

    return penalty;
  }

  double HandNet::calcPenalty(const double* coeff) {
    double penalty = 1.0;
    const uint32_t coeff_dim = HAND_NUM_COEFF;
    const uint32_t coeff_dim_per_model = HAND_NUM_COEFF;
    const float* penalty_scale = kinect_interface::hand_net::HandModel::coeff_penalty_scale();
    const float* max_limit = kinect_interface::hand_net::HandModel::coeff_max_limit_conservative();
    const float* min_limit = kinect_interface::hand_net::HandModel::coeff_min_limit();;

    for (uint32_t i = 0; i < coeff_dim; i++) {
      if (penalty_scale[i % coeff_dim_per_model] > EPSILON) {
        double cur_coeff_val = coeff[i];

#ifdef LINEAR_PENALTY
        // Linear penalty
        if (cur_coeff_val > (double)max_limit[i % coeff_dim_per_model]) {
          penalty += (double)penalty_scale[i % coeff_dim_per_model] * 
            fabs(cur_coeff_val - (double)max_limit[i % coeff_dim_per_model]) / 10.0;
        }
        if (cur_coeff_val < (double)min_limit[i % coeff_dim_per_model]) { 
          penalty += (double)penalty_scale[i % coeff_dim_per_model] * 
            fabs((double)min_limit[i % coeff_dim_per_model] - cur_coeff_val) / 10.0;
        }
#else
        // Quadratic penalty
        if (cur_coeff_val > (double)max_limit[i % coeff_dim_per_model]) {
          double cur_penalty = (double)penalty_scale[i % coeff_dim_per_model] * 
            fabs(cur_coeff_val - (double)max_limit[i % coeff_dim_per_model]) / 2.0;
          penalty += (cur_penalty * cur_penalty);
        }
        if (cur_coeff_val < (double)min_limit[i % coeff_dim_per_model]) { 
          double cur_penalty = (double)penalty_scale[i % coeff_dim_per_model] * 
            fabs((double)min_limit[i % coeff_dim_per_model] - cur_coeff_val) / 2.0;
          penalty += (cur_penalty * cur_penalty);
        }
#endif
      }
    }

    return penalty;
  }

  void HandNet::setPSORadius() {
    const float* cmax = kinect_interface::hand_net::HandModel::coeff_max_limit();
    const float* cmin = kinect_interface::hand_net::HandModel::coeff_min_limit();

    // Set the PSO static radius
    for (uint32_t i = BFGS_HAND_POS_X; i <= BFGS_HAND_POS_Z; i++) {
      pso_radius_[i] = HN_PSO_RAD_POSITION;
    }
    for (uint32_t i = BFGS_HAND_ORIENT_X; i <= BFGS_HAND_ORIENT_Z; i++) {
      pso_radius_[i] = HN_PSO_RAD_EULER;
    }
    for (uint32_t i = BFGS_THUMB_THETA; i <= BFGS_THUMB_K2_PHI; i++) {  // thumb
      pso_radius_[i] = (cmax[i] - cmin[i]) * HN_PSO_RAD_THUMB;
    }
    for (uint32_t i = 0; i < 4; i++) {  // All fingers
      pso_radius_[BFGS_F0_THETA+i*BFGS_FINGER_NUM_COEFF] = 
        (cmax[BFGS_F0_THETA+i*BFGS_FINGER_NUM_COEFF] - 
        cmin[BFGS_F0_THETA+i*BFGS_FINGER_NUM_COEFF]) * HN_PSO_RAD_FINGERS;
      pso_radius_[BFGS_F0_PHI+i*BFGS_FINGER_NUM_COEFF] = 
        (cmax[BFGS_F0_PHI+i*BFGS_FINGER_NUM_COEFF] - 
        cmin[BFGS_F0_PHI+i*BFGS_FINGER_NUM_COEFF]) * HN_PSO_RAD_FINGERS;
      pso_radius_[BFGS_F0_CURL+i*BFGS_FINGER_NUM_COEFF] = 
        (cmax[BFGS_F0_CURL+i*BFGS_FINGER_NUM_COEFF] - 
        cmin[BFGS_F0_CURL+i*BFGS_FINGER_NUM_COEFF]) * HN_PSO_RAD_FINGERS;
    }
  }

}  // namespace hand_net
}  // namespace kinect_interface
