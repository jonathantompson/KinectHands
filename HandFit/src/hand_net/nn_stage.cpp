#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <iomanip>
#include <fstream>
#include "hand_net/nn_stage.h"

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
 
  NNStage::NNStage() {
    n_inputs_ = 0;
    n_outputs_ = 0;
    nonlin_type_ = UndefinedNNNonlin;
    weights_ = NULL;
  }

  NNStage::~NNStage() {
    SAFE_DELETE(weights_);
  }

  void NNStage::loadFromFile(std::ifstream& file) {
    file.read((char*)(&n_outputs_), sizeof(n_outputs_));
    file.read((char*)(&n_inputs_), sizeof(n_inputs_));

    int32_t n_weights = n_outputs_ * n_inputs_;

    weights_ = new float[n_weights];
    file.read((char*)(weights_), sizeof(weights_[0]) * n_weights);

    int32_t nonlin_type;
    file.read((char*)(&nonlin_type), sizeof(nonlin_type));
    nonlin_type_ = (NNNonlinType)nonlin_type;
  }

  void NNStage::printToStdOut() const {
    std::cout << "  n_outputs_ = " << n_outputs_ << std::endl;
    std::cout << "  n_inputs_ = " << n_inputs_ << std::endl;
    std::cout << "  nonlin_type_ = " << (int32_t)nonlin_type_ << std::endl;

    std::cout << std::setprecision(6);
    std::cout << std::fixed;

    uint32_t i = 0;
    uint32_t n_print = std::min<int32_t>(n_outputs_ * n_inputs_, 
      NN_MAX_PRINT_LENGTH);
    std::cout.setf(0, std::ios::showpos);
    std::cout << "  weights_ =" << std::endl;
    for (int32_t v = 0; v < n_outputs_ && i < n_print; v++) {
      if (v == 0) {
        std::cout << "    (0,0) ";
      } else {
        std::cout << "          ";
      }
      std::cout.setf(std::ios::showpos);
      for (int32_t u = 0; u < n_inputs_ && i < n_print; u++) {
        std::cout << weights_[v * n_inputs_ + u];
        if (u != n_inputs_ - 1) {
          std::cout << ", ";
        } else {
          std::cout << std::endl;
        }
        i++;
      }
    }

    std::cout << std::endl;
    std::cout << std::resetiosflags(std::ios_base::showpos);
  }

  int32_t NNStage::dataSizeReq() const {
    return n_inputs_ * n_outputs_;
  }

  void NNStage::forwardProp(float*& in, float*& out) const {
    // TO DO:
  }

}  // namespace hand_model
