#include <sstream>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include "kinect_interface/hand_net/float_tensor.h"
#include "jtil/exceptions/wruntime_error.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::math;

namespace kinect_interface {
namespace hand_net {

  FloatTensor::FloatTensor(const jtil::math::Uint4& dim) {
    dim_.set(dim);
    data_ = new float[dim_[0] * dim_[1] * dim_[2] * dim_[3]]; 
  }

  FloatTensor::FloatTensor(const jtil::math::Uint3& dim) {
    dim_.set(dim[0], dim[1], dim[2], 1);
    data_ = new float[dim_[0] * dim_[1] * dim_[2] * dim_[3]]; 
  }

  FloatTensor::FloatTensor(const jtil::math::Uint2& dim) {
    dim_.set(dim[0], dim[1], 1, 1);
    data_ = new float[dim_[0] * dim_[1] * dim_[2] * dim_[3]]; 
  }

  FloatTensor::FloatTensor(const int dim) {
    dim_.set(dim, 1, 1, 1);
    data_ = new float[dim_[0] * dim_[1] * dim_[2] * dim_[3]]; 
  }

  FloatTensor::~FloatTensor() {
    delete[] data_;
  }

  float& FloatTensor::operator()(const uint32_t x, const uint32_t y, 
    const uint32_t z, const uint32_t w) {
#if defined(DEBUG) || defined(_DEBUG)
    if (x >= dim_[0] || y >= dim_[1] || z >= dim_[2] || w >= dim_[3]) {
      throw std::wruntime_error("FloatTensor::operator() - ERROR: index out "
        "of bounds!");
    }
#endif
    // We want: X + dim[0]*Y + dim[0]*dim[1]*Z + dim[0]*dim[1]*dim[2]*W
    //        = X + dim[0]*(Y + dim[1]*(Z + dim[2]*W)
    return data_[x + dim_[0]*(y + dim_[1]*(z + dim_[2]*w))];
  }

  void FloatTensor::print() const {
    const int32_t dim = dim_[0] * dim_[1];
    for (int32_t j = 0; j < dim_[3]; j++) {
      for (int32_t i = 0; i < dim_[2]; i++) {
        std::cout.setf(0, std::ios::showpos);
        std::cout << "  3dtensor[" << j << ", " << i << ", *, *] =";
        std::cout << std::endl;
        float* data = &data_[j * dim_[2]*dim_[1]*dim_[0] +
          i * dim_[1]*dim_[0]];
        for (int32_t v = 0; v < dim_[1]; v++) {
          if (v == 0) {
            std::cout << "    (0,0) ";
          } else {
            std::cout << "          ";
          }
          std::cout.setf(std::ios::showpos);
          for (int32_t u = 0; u < dim_[0]; u++) {
            std::cout << data[v * dim_[0] + u];
            if (u != dim_[0] - 1) {
              std::cout << ", ";
            } else {
              std::cout << std::endl;
            }
          }
        }
      }
    }
    std::cout << std::resetiosflags(std::ios_base::showpos);
  };

  FloatTensor* FloatTensor::gaussian1D(const int32_t kernel_size) {
    FloatTensor* ret = new FloatTensor(Uint4(kernel_size, 1, 1, 1));
    const float sigma = 0.25f;
    const float amplitude = 1.0f;
    const float size = (float)kernel_size;
    const float center = size/2.0f + 0.5f;
    for (int32_t i = 0; i < kernel_size; i++) {
      ret->data_[i] = amplitude * expf(-(powf(((float)(i+1)-center) / (sigma*size),
        2.0f)/2.0f));
    }
    return ret;
  }

  FloatTensor* FloatTensor::ones1D(const int32_t kernel_size) {
    FloatTensor* ret = new FloatTensor(Uint4(kernel_size, 1, 1, 1));
    for (int32_t i = 0; i < kernel_size; i++) {
      ret->data_[i] = 1.0f;
    }
    return ret;
  }

  FloatTensor* FloatTensor::copy() const {
    FloatTensor* ret = new FloatTensor(dim_);
    memcpy(ret->data_, data_, sizeof(ret->data_[0]) * dataSize());
  }

}  // namespace hand_net
}  // namespace kinect_interface
