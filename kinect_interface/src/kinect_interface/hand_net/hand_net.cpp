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
#include "kinect_interface/hand_detector/decision_tree_structs.h"  // for GDT_MAX_DIST
#include "jtil/image_util/image_util.h"
#include "jtil/data_str/vector.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/renderer/renderer.h"
#include "jtil/renderer/geometry/geometry_manager.h"
#include "jtil/threading/thread_pool.h"
#include "jtil/math/lm_fit.h"
#include "jtil/math/bfgs.h"
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
    heat_map_size_ = 0;
    num_output_features_ = 0;
    rest_pose_ = NULL;
    lhand_cur_pose_ = NULL;
    rhand_cur_pose_ = NULL;
    lhand_ = NULL;
    rhand_ = NULL;
    heat_map_lm_ = NULL;
    camera_ = NULL;
    bfgs_ = NULL;
  }

  HandNet::~HandNet() {
    releaseData();
  }

  void HandNet::releaseData() {
    SAFE_DELETE(conv_network_);
    SAFE_DELETE(image_generator_);
    SAFE_DELETE_ARR(heat_map_convnet_);
    SAFE_DELETE_ARR(gauss_coeff_);
    SAFE_DELETE_ARR(hm_temp_);
    SAFE_DELETE_ARR(lm_fit_x_vals_);
    SAFE_DELETE(rest_pose_);
    SAFE_DELETE(lhand_cur_pose_);
    SAFE_DELETE(rhand_cur_pose_);
    SAFE_DELETE(rhand_);
    SAFE_DELETE(lhand_);
    SAFE_DELETE(heat_map_lm_);
    SAFE_DELETE(camera_);
    SAFE_DELETE(bfgs_);
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
    data_type_ = HPF_DEPTH_DATA;

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
    gauss_coeff_ = new float[NUM_GAUSSIAN_COEFFS * num_output_features_];
    lm_fit_x_vals_ = new float[heat_map_size_ * heat_map_size_ * X_DIM_LM_FIT];
    for (uint32_t v = 0, i = 0; v < heat_map_size_; v++) {
      for (uint32_t u = 0; u < heat_map_size_; u++, i++) {
        lm_fit_x_vals_[i * X_DIM_LM_FIT] = (float)u;
        lm_fit_x_vals_[i * X_DIM_LM_FIT + 1] = (float)v;
      }
    }
    heat_map_lm_ = new LMFit<float>(NUM_GAUSSIAN_COEFFS, X_DIM_LM_FIT,
      heat_map_size_ * heat_map_size_);
    bfgs_ = new BFGS<float>(BFGSHandCoeff::BFGS_NUM_PARAMETERS);
    rest_pose_ = new HandModelCoeff(HandType::LEFT);
    rhand_cur_pose_ = new HandModelCoeff(HandType::RIGHT);
    lhand_cur_pose_ = new HandModelCoeff(HandType::LEFT);
    rest_pose_->loadFromFile("./", "coeff_hand_rest_pose.bin");
    rhand_cur_pose_->copyCoeffFrom(rest_pose_);

    // Initialize the camera
    FloatQuat eye_rot; eye_rot.identity();
    Float3 eye_pos(0, 0, 0);
    float fov_vert_deg = 360.0f * OpenNIFuncs::fVFOV_primesense_109 / 
      (2.0f * (float)M_PI);
    camera_ = new jtil::renderer::Camera(eye_rot, eye_pos, src_width, 
      src_height, fov_vert_deg, -10.0f, -3000.0f);
    camera_->updateProjection();
    camera_->updateView();
  }

  void HandNet::calcHandImage(const int16_t* depth, const uint8_t* label) {
    image_generator_->calcHandImage(depth, label, data_type_ == HPF_DEPTH_DATA);
  }

  void HandNet::loadHandModels() {
    rhand_ = new HandModel(HandType::RIGHT);
    rhand_->updateMatrices(rest_pose_->coeff());
  }

  void HandNet::calcConvnetHeatMap(const int16_t* depth, 
    const uint8_t* label) {
    if (conv_network_ == NULL || image_generator_ == NULL) {
      std::cout << "HandNet::calcHandCoeff() - ERROR: Convnet not loaded";
      std::cout << " from file!" << std::endl;
    }

    calcHandImage(depth, label);

    // Copy over the hand images in the input data structures
    TorchData* im;
    switch (data_type_) {
    case DEPTH_DATA:
      im = image_generator_->hand_image();
      break;
    case HPF_DEPTH_DATA:
      im = image_generator_->hpf_hand_image();
      break;
    default:
      throw std::wruntime_error("HandNet::calcHandCoeffConvnet() - ERROR: "
        "data_type value is not supported!");
    }

    // Now propogate through the network
    conv_network_->forwardProp(*im);
    //jtorch::cl_context->sync(jtorch::deviceid);  // Not necessary
    Tensor<float>* output_tensor = (Tensor<float>*)(conv_network_->output);
    output_tensor->getData(heat_map_convnet_);
  }

  void HandNet::calcConvnetPose() {
    // For each of the heat maps, fit a gaussian to it
    const Int4& pos_wh = image_generator_->hand_pos_wh();
    for (uint32_t i = 0; i < num_output_features_; i++) {
      calcGaussDistCoeff(&gauss_coeff_[i * NUM_GAUSSIAN_COEFFS], 
        &heat_map_convnet_[i * heat_map_size_ * heat_map_size_]);
      // Transform the gaussian into the kinect image space (just a viewport
      // transform!):
      gauss_coeff_[GaussMeanU] = gauss_coeff_[GaussMeanU] * (float)pos_wh[2] + 
        (float)pos_wh[0];
      gauss_coeff_[GaussMeanV] = gauss_coeff_[GaussMeanV] * (float)pos_wh[3] + 
        (float)pos_wh[1];
      gauss_coeff_[GaussVarU] *= (float)pos_wh[2];
      gauss_coeff_[GaussVarV] *= (float)pos_wh[3];
    }

    // Try fitting in projected space from the rest pose:
    memcpy(cur_coeff, rest_pose_->coeff(), sizeof(cur_coeff[0]) * 
      HandCoeff::NUM_PARAMETERS);
    g_hand_net_ = this;
    bfgs_->minimize(rhand_cur_pose_->coeff(), rest_pose_->coeff(), 
      HandModel::angle_coeffs(), bfgsFunc, bfgsJacobFunc, NULL);

    rhand_->updateMatrices(rhand_cur_pose_->coeff());
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
    
    // Calculate the weighted mean
    mean.zeros();
    float max_weight = 0;
    uint32_t i = 0;
    for (uint32_t v = 0; v < heat_map_size_; v++) {
      for (uint32_t u = 0; u < heat_map_size_; u++, i++) {
        hm_temp_[i] = fabsf(im_data[i]) / sum_weights;
        mean[0] += hm_temp_[i] * (float)u;
        mean[1] += hm_temp_[i] * (float)v;
        max_weight = std::max<float>(max_weight, hm_temp_[i]);
      }
    }
    sum_weights = 1.0f;
    Float2::scale(mean, 1.0f / sum_weights);  // Leave it here for clarity!

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
      throw std::wruntime_error("HandNet::calcGaussDistCoeff() - ERROR: "
        "Not enough non-zero heat map weights!");
    }
    Float2::scale(var, 1.0f / (sum_weights * (non_zero_weights - 1) / non_zero_weights));

    //float c_start[NUM_GAUSSIAN_COEFFS];
    //c_start[GaussAmp] = max_weight;
    //c_start[GaussMeanU] = mean[0];
    //c_start[GaussMeanV] = mean[1];
    //c_start[GaussVarU] = var[0];
    //c_start[GaussVarV] = var[1];
    //heat_map_lm_->fitModel(gauss_coeff, c_start, hm_temp_, lm_fit_x_vals_,
    //  gauss2D, jacobGauss2D);

    // LM is too slow.  Just use the std and mean directly:
    gauss_coeff[0] = max_weight;
    gauss_coeff[1] = mean[0];
    gauss_coeff[2] = mean[1];
    gauss_coeff[3] = var[0];
    gauss_coeff[4] = var[1];
  }

  float HandNet::gauss2D(const float* x, const float* c) {
    float du = (x[0] - c[GaussMeanU]);
    float dv = (x[1] - c[GaussMeanV]);
    return c[GaussAmp] * exp(-(du * du / (2.0f * c[GaussVarU]) + dv * dv / (2.0f * c[GaussVarV])));
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

  float HandNet::bfgsFunc(const float* bfgs_hand_coeff) {
    // Update the matrix heirachy
    BFGSHandCoeffToHandCoeff(g_hand_net_->cur_coeff, bfgs_hand_coeff);
    g_hand_net_->rhand_->updateMatrices(g_hand_net_->cur_coeff);

    // Calculate the projected sphere positions:
    float uv[2];
    g_hand_net_->rhand_->calcBoundingSphereUVPos(uv, 
      HandSphereIndices::PALM_3, g_hand_net_->camera_->proj_view());

    g_hand_net_->rhand_->calcBoundingSphereUVPos(uv, 
      HandSphereIndices::PALM_1,  g_hand_net_->camera_->proj_view());

    g_hand_net_->rhand_->calcBoundingSphereUVPos(uv, 
      HandSphereIndices::PALM_2,  g_hand_net_->camera_->proj_view());

    g_hand_net_->rhand_->calcBoundingSphereUVPos(uv, 
      HandSphereIndices::TH_KNU3_A,  g_hand_net_->camera_->proj_view());

    for (uint32_t i = 0; i < 4; i++) {
      g_hand_net_->rhand_->calcBoundingSphereUVPos(uv, 
        HandSphereIndices::F1_KNU3_A + NSPH_PER_GROUP * i, 
        g_hand_net_->camera_->proj_view());
    }
    return 0;
  }

  void HandNet::bfgsJacobFunc(float* jacob, const float* bfgs_hand_coeff) {
    memset(jacob, 0, BFGSHandCoeff::BFGS_NUM_PARAMETERS * sizeof(jacob[0]));
  }

  void HandNet::BFGSHandCoeffToHandCoeff(float* hand_coeff, 
    const float* bfgs_hand_coeff) {
    hand_coeff[HAND_POS_X] = bfgs_hand_coeff[BFGS_HAND_POS_X];
    hand_coeff[HAND_POS_Y] = bfgs_hand_coeff[BFGS_HAND_POS_Y];
    hand_coeff[HAND_POS_Z] = bfgs_hand_coeff[BFGS_HAND_POS_Z];
    hand_coeff[HAND_ORIENT_X] = bfgs_hand_coeff[BFGS_HAND_ORIENT_X];
    hand_coeff[HAND_ORIENT_Y] = bfgs_hand_coeff[BFGS_HAND_ORIENT_Y];
    hand_coeff[HAND_ORIENT_Z] = bfgs_hand_coeff[BFGS_HAND_ORIENT_Z];
    hand_coeff[THUMB_THETA] = bfgs_hand_coeff[BFGS_THUMB_THETA];
    hand_coeff[THUMB_PHI] = bfgs_hand_coeff[BFGS_THUMB_PHI];
    hand_coeff[THUMB_K1_THETA] = bfgs_hand_coeff[BFGS_THUMB_K1_THETA];
    hand_coeff[THUMB_K1_PHI] = bfgs_hand_coeff[BFGS_THUMB_K1_PHI];
    hand_coeff[THUMB_K2_PHI] = bfgs_hand_coeff[BFGS_THUMB_K2_PHI];
    hand_coeff[F0_THETA] = bfgs_hand_coeff[BFGS_F0_THETA];
    hand_coeff[F0_PHI] = bfgs_hand_coeff[BFGS_F0_PHI];
    hand_coeff[F0_KNUCKLE_MID] = bfgs_hand_coeff[BFGS_F0_KNUCKLE_MID];
    hand_coeff[F0_KNUCKLE_END] = bfgs_hand_coeff[BFGS_F0_KNUCKLE_END];
    hand_coeff[F1_THETA] = bfgs_hand_coeff[BFGS_F1_THETA];
    hand_coeff[F1_PHI] = bfgs_hand_coeff[BFGS_F1_PHI];
    hand_coeff[F1_KNUCKLE_MID] = bfgs_hand_coeff[BFGS_F1_KNUCKLE_MID];
    hand_coeff[F1_KNUCKLE_END] = bfgs_hand_coeff[BFGS_F1_KNUCKLE_END];
    hand_coeff[F2_THETA] = bfgs_hand_coeff[BFGS_F2_THETA];
    hand_coeff[F2_PHI] = bfgs_hand_coeff[BFGS_F2_PHI];
    hand_coeff[F2_KNUCKLE_MID] = bfgs_hand_coeff[BFGS_F2_KNUCKLE_MID];
    hand_coeff[F2_KNUCKLE_END] = bfgs_hand_coeff[BFGS_F2_KNUCKLE_END];
    hand_coeff[F3_THETA] = bfgs_hand_coeff[BFGS_F3_THETA];
    hand_coeff[F3_PHI] = bfgs_hand_coeff[BFGS_F3_PHI];
    hand_coeff[F3_KNUCKLE_MID] = bfgs_hand_coeff[BFGS_F3_KNUCKLE_MID];
    hand_coeff[F3_KNUCKLE_END] = bfgs_hand_coeff[BFGS_F3_KNUCKLE_END];
  }

  void HandNet::HandCoeffToBFGSHandCoeff(float* bfgs_hand_coeff, 
    const float* hand_coeff) {
    bfgs_hand_coeff[BFGS_HAND_POS_X] = hand_coeff[HAND_POS_X];
    bfgs_hand_coeff[BFGS_HAND_POS_Y] = hand_coeff[HAND_POS_Y];
    bfgs_hand_coeff[BFGS_HAND_POS_Z] = hand_coeff[HAND_POS_Z];
    bfgs_hand_coeff[BFGS_HAND_ORIENT_X] = hand_coeff[HAND_ORIENT_X];
    bfgs_hand_coeff[BFGS_HAND_ORIENT_Y] = hand_coeff[HAND_ORIENT_Y];
    bfgs_hand_coeff[BFGS_HAND_ORIENT_Z] = hand_coeff[HAND_ORIENT_Z];
    bfgs_hand_coeff[BFGS_THUMB_THETA] = hand_coeff[THUMB_THETA];
    bfgs_hand_coeff[BFGS_THUMB_PHI] = hand_coeff[THUMB_PHI];
    bfgs_hand_coeff[BFGS_THUMB_K1_THETA] = hand_coeff[THUMB_K1_THETA];
    bfgs_hand_coeff[BFGS_THUMB_K1_PHI] = hand_coeff[THUMB_K1_PHI];
    bfgs_hand_coeff[BFGS_THUMB_K2_PHI] = hand_coeff[THUMB_K2_PHI];
    bfgs_hand_coeff[BFGS_F0_THETA] = hand_coeff[F0_THETA];
    bfgs_hand_coeff[BFGS_F0_PHI] = hand_coeff[F0_PHI];
    bfgs_hand_coeff[BFGS_F0_KNUCKLE_MID] = hand_coeff[F0_KNUCKLE_MID];
    bfgs_hand_coeff[BFGS_F0_KNUCKLE_END] = hand_coeff[F0_KNUCKLE_END];
    bfgs_hand_coeff[BFGS_F1_THETA] = hand_coeff[F1_THETA];
    bfgs_hand_coeff[BFGS_F1_PHI] = hand_coeff[F1_PHI];
    bfgs_hand_coeff[BFGS_F1_KNUCKLE_MID] = hand_coeff[F1_KNUCKLE_MID];
    bfgs_hand_coeff[BFGS_F1_KNUCKLE_END] = hand_coeff[F1_KNUCKLE_END];
    bfgs_hand_coeff[BFGS_F2_THETA] = hand_coeff[F2_THETA];
    bfgs_hand_coeff[BFGS_F2_PHI] = hand_coeff[F2_PHI];
    bfgs_hand_coeff[BFGS_F2_KNUCKLE_MID] = hand_coeff[F2_KNUCKLE_MID];
    bfgs_hand_coeff[BFGS_F2_KNUCKLE_END] = hand_coeff[F2_KNUCKLE_END];
    bfgs_hand_coeff[BFGS_F3_THETA] = hand_coeff[F3_THETA];
    bfgs_hand_coeff[BFGS_F3_PHI] = hand_coeff[F3_PHI];
    bfgs_hand_coeff[BFGS_F3_KNUCKLE_MID] = hand_coeff[F3_KNUCKLE_MID];
    bfgs_hand_coeff[BFGS_F3_KNUCKLE_END] = hand_coeff[F3_KNUCKLE_END];
  }

}  // namespace hand_net
}  // namespace kinect_interface
