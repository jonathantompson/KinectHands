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
#include "kinect_interface/hand_net/hand_model.h"  // for HandCoeff
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
    num_conv_banks_ = 0;
    conv_stages_ = NULL;
    image_generator_ = NULL;
    nn_stages_ = NULL;
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
    SAFE_DELETE_ARR(conv_stages_);
    SAFE_DELETE(nn_stages_);
    SAFE_DELETE(image_generator_);
  }

  void HandNet::loadFromFile(const std::string& filename) {
    releaseData();

    std::cout << "loading HandNet from " << filename << std::endl;

    /*
    // TO DO: UPDATE THIS
    std::ifstream file(filename.c_str(), std::ios::in|std::ios::binary);
    if (file.is_open()) {

      file.seekg(0, std::ios::beg);

      // Get the meta data
      file.read(reinterpret_cast<char*>(&n_conv_stages_), 
        sizeof(n_conv_stages_));
      file.read(reinterpret_cast<char*>(&n_nn_stages_), sizeof(n_nn_stages_));
      file.read(reinterpret_cast<char*>(&num_conv_banks_), sizeof(num_conv_banks_));

      int32_t data_type;
      file.read(reinterpret_cast<char*>(&data_type), sizeof(data_type));
      data_type_ = (HandNetDataType)data_type;

      conv_stages_ = new ConvStage*[n_conv_stages_ * num_conv_banks_];
      for (int32_t j = 0; j < num_conv_banks_; j++) {
        // Load in the convolution stages
        for (int32_t i = 0; i < n_conv_stages_; i++) {
          conv_stages_[j * n_conv_stages_ + i] = new ConvStage();
          conv_stages_[j * n_conv_stages_ + i]->loadFromFile(file);
        }
      }

      // Load in the neural network stages
      nn_stages_ = new NNStage*[n_nn_stages_];
      for (int32_t i = 0; i < n_nn_stages_; i++) {
        nn_stages_[i] = new NNStage();
        nn_stages_[i]->loadFromFile(file);
      }

      // clean up file io
      file.close();

      // Now create sufficient temporary data so that we can propogate the
      // forward model
      int32_t total_conv_output_size = 0;
      conv_datcur_ = new float*[num_conv_banks_];
      conv_datnext_ = new float*[num_conv_banks_];
      for (int32_t j = 0; j < num_conv_banks_; j++) {
        ConvStage* cstage;
        int32_t im_sizeu = HN_IM_SIZE / (1 << j);
        int32_t im_sizev = HN_IM_SIZE / (1 << j);
        int32_t max_size = im_sizeu * im_sizev;
        for (int32_t i = 0; i < n_conv_stages_; i++) {
          cstage = conv_stages_[j * n_conv_stages_ + i];
          max_size = std::max<int32_t>(max_size, 
            cstage->dataSizeReq(im_sizeu, im_sizev));
          im_sizeu = cstage->calcOutWidth(im_sizeu);
          im_sizev = cstage->calcOutHeight(im_sizev);
        }
        total_conv_output_size += im_sizeu * im_sizev * 
          cstage->n_output_features();
        conv_datcur_[j] = new float[max_size];
        conv_datnext_[j] = new float[max_size];
      }
      

      // Quick check to make sure the sizes match up!
      if (total_conv_output_size != nn_stages_[0]->n_inputs()) {
          throw std::wruntime_error("HandNet::loadFromFile() - INTERNAL ERROR:"
            " convolution output size doesn't match neural net intput size");
      }

      uint32_t max_size = 0;
      for (int32_t i = 0; i < n_nn_stages_; i++) {
        max_size = std::max<uint32_t>(max_size, nn_stages_[i]->dataSizeReq());
        if (i < n_nn_stages_ - 1) {
          if (nn_stages_[i]->n_outputs() != nn_stages_[i+1]->n_inputs()) {
            throw std::wruntime_error("HandNet::loadFromFile() - INTERNAL "
              "ERROR: neural net out size doesn't match neural net in size");
          }
        }
      }

      image_generator_ = new HandImageGenerator(num_conv_banks_);

      // Finally, allocate size for the data that will flow through convnet
      nn_datcur_ = new float[max_size];
      nn_datnext_ = new float[max_size];

      // Now set up multithreaded callbacks.
      tp_ = new ThreadPool(HN_NUM_WORKER_THREADS);

    } else {
      std::stringstream ss;
      ss << "HandNet::loadFromFile() - ERROR: Could not open convnet";
      ss << " file " << filename << std::endl;
      throw std::wruntime_error(ss.str());
    }
    */
  }

  void HandNet::calcHandImage(const int16_t* depth, const uint8_t* label) {
    image_generator_->calcHandImage(depth, label, 
      data_type_ == HPF_DEPTH_DATA);
  }

  void HandNet::calcHandCoeffConvnet(const int16_t* depth, 
    const uint8_t* label) {
    if (num_conv_banks_ == 0 || image_generator_ == NULL) {
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
      im = image_generator_->hpf_hand_images();
      break;
    default:
      throw std::wruntime_error("HandNet::calcHandCoeffConvnet() - ERROR: "
        "data_type value is not supported!");
    }

    /*
    // TO DO: UPDATE THIS
    for (int32_t j = 0; j < num_conv_banks_; j++) {
      int32_t w = HN_IM_SIZE / (1 << j);
      int32_t h = HN_IM_SIZE / (1 << j);
      memcpy(conv_datcur_[j], im, w * h * sizeof(conv_datcur_[j][0]));
      im = &im[w*h];
    }

    // Now propogate the outputs through each of the banks and copy each
    // output into the nn input
    float* nn_input = nn_datcur_;
    ConvStage* stage = NULL;
    for (int32_t j = 0; j < num_conv_banks_; j++) {
      int32_t w = HN_IM_SIZE / (1 << j);
      int32_t h = HN_IM_SIZE / (1 << j);
      for (int32_t i = 0; i < n_conv_stages_; i++) {
        stage = conv_stages_[j * n_conv_stages_ + i];
        stage->forwardProp(conv_datcur_[j], w, h, conv_datnext_[j], tp_);
        // Ping-pong the buffers
        float* tmp = conv_datnext_[j];
        conv_datnext_[j] = conv_datcur_[j];
        conv_datcur_[j] = tmp;
        // Calculate the next stage size
        w = stage->calcOutWidth(w);
        h = stage->calcOutHeight(h);
      }
      int32_t s = w * h * stage->n_output_features();
      memcpy(nn_input, conv_datcur_[j], s * sizeof(nn_input[0]));
      nn_input = &nn_input[s];
      // print3DTensorToStdCout<float>(conv_datcur_[j], 0, w, h, 2, 2, 6, 6);
    }

    // print3DTensorToStdCout<float>(nn_datcur_, 1, 
    //   nn_stages_[0]->n_inputs(), 1);

    for (int32_t i = 0; i < n_nn_stages_; i++) {
      nn_stages_[i]->forwardProp(nn_datcur_, nn_datnext_, tp_);
      // Ping-pong the buffers
      float* tmp = nn_datnext_;
      nn_datnext_ = nn_datcur_;
      nn_datcur_ = tmp; 
      // print3DTensorToStdCout<float>(nn_datcur_, 1, 
      //   nn_stages_[i]->n_outputs(), 1);
    }

    memcpy(coeff_convnet_, nn_datcur_, HAND_NUM_COEFF_CONVNET * 
      sizeof(coeff_convnet_[0]));

    // print3DTensorToStdCout<float>(nn_datcur_, 1, HAND_NUM_COEFF_CONVNET, 1);
    */
  }

  const float* HandNet::hpf_hand_images() const {
    return image_generator_->hpf_hand_images();
  }

  const float* HandNet::hand_image() const {
    return image_generator_->hpf_hand_images();
  }

  const int32_t HandNet::size_images() const {
    return image_generator_->size_images();
  }

  const jtil::math::Float3& HandNet::uvd_com() const {
    return image_generator_->uvd_com();
  }

}  // namespace hand_net
}  // namespace kinect_interface
