#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <cmath>
#include <sstream>
#include "kinect_interface/hand_net/hand_net.h"
#include "kinect_interface/hand_net/hand_image_generator.h"
#include "kinect_interface/hand_net/torch_stage.h"
#include "kinect_interface/hand_net/parallel.h"
#include "kinect_interface/hand_net/sequential.h"
#include "kinect_interface/hand_net/table.h"
#include "kinect_interface/hand_net/float_tensor.h"
#include "kinect_interface/hand_net/hand_model_coeff.h"  // for HandCoeff
#include "kinect_interface/open_ni_funcs.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"  // for GDT_MAX_DIST
#include "jtil/image_util/image_util.h"
#include "jtil/data_str/vector.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread_pool.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using jtil::math::Float4x4;
using jtil::math::FloatQuat;
using jtil::math::Float3;
using jtil::math::Float4;
using jtil::math::Float2;
using jtil::math::Int2;
using jtil::threading::ThreadPool;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using jtil::math::Float3;
using namespace jtil::image_util;
using kinect_interface::hand_net::HandCoeff;

namespace kinect_interface {
namespace hand_net {
 
  HandNet::HandNet() {
    conv_network_ = NULL;
    image_generator_ = NULL;
    conv_network_input_ = NULL;
    tp_ = NULL;
  }

  HandNet::~HandNet() {
    releaseData();
  }

  void HandNet::releaseData() {
    if (tp_) {
      tp_->stop();
    }
    SAFE_DELETE(tp_);
    SAFE_DELETE(conv_network_);
    SAFE_DELETE(image_generator_);
    SAFE_DELETE(conv_network_input_);
  }

  void HandNet::loadFromFile(const std::string& filename) {
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

    conv_network_input_ = new Table();
    for (int32_t j = 0; j < num_conv_banks_; j++) {
      int32_t w = HN_IM_SIZE / (1 << j);
      int32_t h = HN_IM_SIZE / (1 << j);
      conv_network_input_->add(new FloatTensor(Int2(w, h)));
    }

    tp_ = new ThreadPool(HN_NUM_WORKER_THREADS);
  }

  void HandNet::calcHandImage(const int16_t* depth, const uint8_t* label) {
    image_generator_->calcHandImage(depth, label, 
      data_type_ == HPF_DEPTH_DATA, tp_);
  }

  void HandNet::calcHandCoeffConvnet(const int16_t* depth, 
    const uint8_t* label) {
    if (conv_network_ == NULL || image_generator_ == NULL) {
      std::cout << "HandNet::calcHandCoeff() - ERROR: Convnet not loaded";
      std::cout << " from file!" << std::endl;
    }

    calcHandImage(depth, label);

    // Copy over the hand images in the input data structures
    const float* im;
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

    for (int32_t j = 0; j < num_conv_banks_; j++) {
      int32_t w = HN_IM_SIZE / (1 << j);
      int32_t h = HN_IM_SIZE / (1 << j);
      FloatTensor* cur_dst_im = (FloatTensor*)((*conv_network_input_)(j));
      memcpy(cur_dst_im->data(), im, w * h * sizeof(cur_dst_im->data()[0]));
      im = &im[w*h];
    }

    // Now propogate through the network
    conv_network_->forwardProp(*conv_network_input_, *tp_);
    FloatTensor* output_tensor = (FloatTensor*)(conv_network_->output);

    memcpy(coeff_convnet_, output_tensor->data(), HAND_NUM_COEFF_CONVNET * 
      sizeof(coeff_convnet_[0]));
  }

  const float* HandNet::hpf_hand_image() const {
    return image_generator_->hpf_hand_image();
  }

  const float* HandNet::hand_image() const {
    return image_generator_->hpf_hand_image();
  }

  const int32_t HandNet::size_images() const {
    return image_generator_->size_images();
  }

  const jtil::math::Float3& HandNet::uvd_com() const {
    return image_generator_->uvd_com();
  }

}  // namespace hand_net
}  // namespace kinect_interface
