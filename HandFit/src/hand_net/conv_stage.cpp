#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <iomanip>
#include <fstream>
#include "hand_net/conv_stage.h"
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
 
  ConvStage::ConvStage() {
    filt_width_ = 0;
    filt_height_ = 0;
    n_input_features_ = 0;
    n_output_features_ = 0;
    filt_fan_in_ = 0;
    norm_type_ = UndefinedNorm;
    pool_type_ = UndefinedPool;
    pool_size_ = 0;
    nonlin_type_ = UndefinedNonlin;
    weights_ = NULL;
    conn_table_ = NULL;
  }

  ConvStage::~ConvStage() {
    if (weights_) {
      for (int32_t i = 0; i < n_output_features_ * filt_fan_in_; i++) {
        SAFE_DELETE(weights_[i]);
      }
    }
    SAFE_DELETE(weights_);
    if (conn_table_) {
      for (int32_t i = 0; i < n_output_features_; i++) {
        SAFE_DELETE(conn_table_[i]);
      }
    }
    SAFE_DELETE(conn_table_);
  }

  void ConvStage::loadFromFile(std::ifstream& file) {
    file.read((char*)(&filt_width_), sizeof(filt_width_));
    file.read((char*)(&filt_height_), sizeof(filt_height_));
    file.read((char*)(&n_input_features_), sizeof(n_input_features_));
    file.read((char*)(&n_output_features_), sizeof(n_output_features_));
    file.read((char*)(&filt_fan_in_), sizeof(filt_fan_in_));

    int32_t filt_dim = filt_width_ * filt_height_;

    weights_ = new float*[n_output_features_ * filt_fan_in_];
    for (int32_t i = 0; i < n_output_features_ * filt_fan_in_; i++) {
      weights_[i] = new float[filt_dim];
      file.read((char*)(weights_[i]), sizeof(weights_[i][0]) * filt_dim);
    }

    conn_table_ = new int16_t*[n_output_features_];
    for (int32_t i = 0; i < n_output_features_; i++) {
      conn_table_[i] = new int16_t[filt_fan_in_ * 2];
      file.read((char*)(conn_table_[i]), 
        sizeof(conn_table_[i][0]) * filt_fan_in_ * 2);
    }

    int32_t norm_type;
    file.read((char*)(&norm_type), sizeof(norm_type));
    norm_type_ = (NormType)norm_type;

    int32_t pool_type;
    file.read((char*)(&pool_type), sizeof(pool_type));
    pool_type_ = (PoolType)pool_type;
    file.read((char*)(&pool_size_), sizeof(pool_size_));

    int32_t nonlin_type;
    file.read((char*)(&nonlin_type), sizeof(nonlin_type));
    nonlin_type_ = (NonlinType)nonlin_type;
  }

  void ConvStage::printToStdOut() const {
    std::cout << "  filt_width_ = " << filt_width_ << std::endl;
    std::cout << "  filt_height_ = " << filt_height_ << std::endl;
    std::cout << "  n_input_features_ = " << n_input_features_ << std::endl;
    std::cout << "  n_output_features_ = " << n_output_features_ << std::endl;
    std::cout << "  filt_fan_in_ = " << filt_fan_in_ << std::endl;
    std::cout << "  norm_type_ = " << (int32_t)norm_type_ << std::endl;
    std::cout << "  pool_type_ = " << (int32_t)pool_type_ << std::endl;
    std::cout << "  pool_size_ = " << pool_size_ << std::endl;
    std::cout << "  nonlin_type_ = " << (int32_t)nonlin_type_ << std::endl;

    std::cout << std::setprecision(6);
    std::cout << std::fixed;

    for (int32_t i = 0; i < std::min<int32_t>(n_output_features_ * 
      filt_fan_in_, MAX_PRINT_LENGTH); i++) {
      std::cout.setf(0, std::ios::showpos);
      std::cout << "  weights_[" << i << "] =" << std::endl;
      for (int32_t v = 0; v < filt_height_; v++) {
        if (v == 0) {
          std::cout << "    (0,0) ";
        } else {
          std::cout << "          ";
        }
        std::cout.setf(std::ios::showpos);
        for (int32_t u = 0; u < filt_width_; u++) {
          std::cout << weights_[i][v * filt_width_ + u];
          if (u != filt_width_ - 1) {
            std::cout << ", ";
          } else {
            std::cout << std::endl;
          }
        }
      }
    }

    std::cout.setf(0, std::ios::showpos);
    for (int32_t i = 0; i < std::min<int32_t>(n_output_features_, 
      MAX_PRINT_LENGTH); i++) {
      std::cout << "  conn[" << i << "] = " << std::endl;
      for (int32_t v = 0; v < filt_fan_in_; v++) {
        if (v == 0) {
          std::cout << "    (0,0) ";
        } else {
          std::cout << "          ";
        }
        for (int32_t u = 0; u < 2; u++) {
          std::cout << conn_table_[i][v * 2 + u];
          if (u != 1) {
            std::cout << ", ";
          } else {
            std::cout << std::endl;
          }
        }
      }
    }

    std::cout << std::endl;
    std::cout << std::resetiosflags(std::ios_base::showpos);
  }

  const int32_t ConvStage::dataSizeReq(int32_t inw, int32_t inh) const {
    // input size requirement
    int32_t in_req = inw * inh * n_input_features_;
    // calculate intermediate image size
    int32_t out_req = (inw - filt_width_ + 1) * (inh - filt_height_ + 1) * 
      n_output_features_;
    // The downsampled image is only less than this, so we don't need to worry
    // about it.
    return std::max<int32_t>(in_req, out_req);
  }

  const int32_t ConvStage::calc_out_im_width(int32_t inw) const {
    return (inw - filt_width_ + 1) / pool_size_;
  }

  const int32_t ConvStage::calc_out_im_height(int32_t inh) const {
    return (inh - filt_height_ + 1) / pool_size_;
  }

}  // namespace hand_model
