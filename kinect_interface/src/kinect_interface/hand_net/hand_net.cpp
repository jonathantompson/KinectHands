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
#include "kinect_interface/open_ni_funcs.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"  // for GDT_MAX_DIST
#include "jtil/image_util/image_util.h"
#include "jtil/data_str/vector.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread_pool.h"

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

namespace kinect_interface {
namespace hand_net {
 
  HandNet::HandNet() {
    conv_network_ = NULL;
    image_generator_ = NULL;
    heat_map_convnet_ = NULL;
    heat_map_size_ = 0;
  }

  HandNet::~HandNet() {
    releaseData();
  }

  void HandNet::releaseData() {
    SAFE_DELETE(conv_network_);
    SAFE_DELETE(image_generator_);
    SAFE_DELETE_ARR(heat_map_convnet_);
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

    // Do one forward prop to load all the kernels and figure out the output
    // size (of the heat maps)
    TorchData* im = (TorchData*)image_generator_->hand_image();
    conv_network_->forwardProp(*im);
    Tensor<float>* output_tensor = (Tensor<float>*)(conv_network_->output);
    uint32_t data_size = output_tensor->dataSize();
    heat_map_convnet_ = new float[data_size];
    const uint32_t num_features = (HAND_NUM_COEFF_CONVNET / FEATURE_SIZE);
    heat_map_size_ = (uint32_t)sqrtf((float)(data_size / num_features));
    // Check that the heatmap is square:
    if (heat_map_size_ * heat_map_size_ * num_features != data_size) {
      throw std::wruntime_error("HandNet::loadFromFile() - ERROR: Heat map"
        "size is not what we expect!");
    }
  }

  void HandNet::calcHandImage(const int16_t* depth, const uint8_t* label) {
    image_generator_->calcHandImage(depth, label, data_type_ == HPF_DEPTH_DATA);
  }

  void HandNet::calcHandCoeffConvnet(const int16_t* depth, 
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
    Tensor<float>* output_tensor = (Tensor<float>*)(conv_network_->output);
    output_tensor->getData(heat_map_convnet_);
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

}  // namespace hand_net
}  // namespace kinect_interface
