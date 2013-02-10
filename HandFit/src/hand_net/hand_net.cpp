#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include "hand_net/hand_net.h"
#include "hand_net/conv_stage.h"
#include "hand_net/nn_stage.h"
#include "exceptions/wruntime_error.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }

using math::Float4x4;
using math::FloatQuat;
using math::Float3;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using math::Float3;

namespace hand_net {
 
  HandNet::HandNet(const std::string& convnet_filename) {
    n_conv_stages_ = 0;
    conv_stages_ = NULL;
    n_nn_stages_ = 0;
    nn_stages_ = NULL;

    loadFromFile(convnet_filename);
  }

  HandNet::~HandNet() {
    for (int32_t i = 0; i < n_conv_stages_; i++) {
      SAFE_DELETE(conv_stages_[i]);
    }
    SAFE_DELETE(conv_stages_);
    for (int32_t i = 0; i < n_nn_stages_; i++) {
      SAFE_DELETE(nn_stages_[i]);
    }
    SAFE_DELETE(nn_stages_);
  }

  void HandNet::loadFromFile(const std::string& convnet_filename) {
    std::cout << "loading HandNet from " << convnet_filename << std::endl;

    std::ifstream file(convnet_filename.c_str(), std::ios::in | std::ios::binary);
    if (!file.is_open()) {
      throw std::wruntime_error("HandNet::loadFromFile() - ERROR: Could not "
        "open convnet file!");
    }
    file.seekg(0, std::ios::beg);

    // Get the meta data
    file.read(reinterpret_cast<char*>(&n_conv_stages_), sizeof(n_conv_stages_));
    file.read(reinterpret_cast<char*>(&n_nn_stages_), sizeof(n_nn_stages_));

    // Load in the convolution stages
    conv_stages_ = new ConvStage*[n_conv_stages_];
    for (int32_t i = 0; i < n_conv_stages_; i++) {
      conv_stages_[i] = new ConvStage();
      conv_stages_[i]->loadFromFile(file);
#if defined(DEBUG) || defined(_DEBUG)
      std::cout << "ConvStage[" << i << "]" << std::endl;
      conv_stages_[i]->printToStdOut();
#endif
    }

    // Load in the neural network stages
    nn_stages_ = new NNStage*[n_nn_stages_];
    for (int32_t i = 0; i < n_nn_stages_; i++) {
      nn_stages_[i] = new NNStage();
      nn_stages_[i]->loadFromFile(file);
#if defined(DEBUG) || defined(_DEBUG)
      std::cout << "NNStage[" << i << "]" << std::endl;
      nn_stages_[i]->printToStdOut();
#endif
    }

    // clean up
    file.close();
  }

}  // namespace hand_model
