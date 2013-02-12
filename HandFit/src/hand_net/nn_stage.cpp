#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <iomanip>
#include <fstream>
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
 
  NNStage::NNStage() {
    n_inputs_ = 0;
    n_outputs_ = 0;
    nonlin_type_ = UndefinedNNNonlin;
    weights_ = NULL;
    bias_ = NULL;
  }

  NNStage::~NNStage() {
    SAFE_DELETE(weights_);
    SAFE_DELETE(bias_);
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

    bias_ = new float[n_outputs_];
    file.read((char*)(bias_), sizeof(bias_[0]) * n_outputs_);
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
    performLinearNetwork(out, n_outputs_, (const float*&)in, n_inputs_);
    performNonlinearity(out, n_outputs_);

    // Result is in output
  }

  void NNStage::performNonlinearity(float*&data, const int32_t size) const {
    switch (nonlin_type_) {
    case NoneNNNonlin:
      // Nothing to do for this non-linearity type
      break;
    case TanhNNNonlin:
      for (int32_t outf = 0; outf < size; outf++) {
        data[outf] = tanh(data[outf]);
      }
      break;
    default:
      throw std::wruntime_error("NNStage::performNonlinearity() - ERROR: "
        "Only TanhNNNonlin supported for now.");
    }
  }

  void NNStage::performLinearNetwork(float*&out, const int32_t outsize, 
    const float*& in, const int32_t insize) const {
    for (int32_t outf = 0; outf < outsize; outf++) {
      out[outf] = bias_[outf];
      for (int32_t inf = 0; inf < insize; inf++) {
        out[outf] += weights_[outf * insize + inf] * in[inf];
      }
    }
  }

}  // namespace hand_model
