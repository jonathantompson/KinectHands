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

  FloatTensor::FloatTensor(const jtil::math::Int4& dim) {
    dim_.set(dim);
    data_ = new float[dim_[0] * dim_[1] * dim_[2] * dim_[3]]; 
  }

  FloatTensor::FloatTensor(const jtil::math::Int3& dim) {
    dim_.set(dim[0], dim[1], dim[2], 1);
    data_ = new float[dim_[0] * dim_[1] * dim_[2] * dim_[3]]; 
  }

  FloatTensor::FloatTensor(const jtil::math::Int2& dim) {
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

  float& FloatTensor::operator()(const uint32_t u, const uint32_t v, 
    const uint32_t f, const uint32_t b) {
#if defined(DEBUG) || defined(_DEBUG)
    if (u >= (uint32_t)dim_[0] || v >= (uint32_t)dim_[1] || 
      f >= (uint32_t)dim_[2] || b >= (uint32_t)dim_[3]) {
      throw std::wruntime_error("FloatTensor::operator() - ERROR: index out "
        "of bounds!");
    }
#endif
    // We want: X + dim[0]*Y + dim[0]*dim[1]*Z + dim[0]*dim[1]*dim[2]*W
    //        = X + dim[0]*(Y + dim[1]*(Z + dim[2]*W)
    return data_[u + dim_[0]*(v + dim_[1]*(f + dim_[2]*b))];
  }

  void FloatTensor::print() const {
    const int32_t dim = dim_[0] * dim_[1];
    for (int32_t j = 0; j < dim_[3]; j++) {
      for (int32_t i = 0; i < dim_[2]; i++) {
        std::cout.setf(0, std::ios::showpos);
        std::cout << "  4dtensor[" << j << ", " << i << ", *, *] =";
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

  void FloatTensor::print(const jtil::math::Int2& interval0, 
    const jtil::math::Int2& interval1, const jtil::math::Int2& interval2, 
    const jtil::math::Int2& interval3) const {
    if (interval0[0] > interval0[1] || interval0[0] > interval0[1] || 
      interval0[0] > interval0[1] || interval0[0] > interval0[1]) {
      throw std::wruntime_error("FloatTensor::print() - ERROR: "
        "intervals must be monotonic");
    }
    if (interval0[0] < 0 || interval0[1] >= dim_[0] || 
      interval1[0] < 0 || interval1[1] >= dim_[1] ||
      interval2[0] < 0 || interval2[1] >= dim_[2] ||
      interval3[0] < 0 || interval3[1] >= dim_[3]) {
      throw std::wruntime_error("FloatTensor::print() - ERROR: "
        "intervals out of range");
    }
    for (int32_t b = interval3[0]; b <= interval3[1]; b++) {
      for (int32_t f = interval2[0]; f <= interval2[1]; f++) {
        std::cout.setf(0, std::ios::showpos);
        std::cout << "  4dtensor[" << b << ", " << f << ", *, *] =";
        std::cout << std::endl;
        float* data = &data_[b * dim_[2]*dim_[1]*dim_[0] +
          f * dim_[1]*dim_[0]];
        for (int32_t v = interval1[0]; v <= interval1[1]; v++) {
          if (v == interval1[0]) {
            std::cout << "    (" << interval1[0] << "," <<  interval0[0] << ") ";
          } else {
            std::cout << "          ";
          }
          std::cout.setf(std::ios::showpos);
          for (int32_t u = interval0[0]; u <= interval0[1]; u++) {
            std::cout << data[v * dim_[0] + u];
            if (u != interval0[1]) {
              std::cout << ", ";
            } else {
              std::cout << std::endl;
            }
          }
          }
      }
    }
  }

  FloatTensor* FloatTensor::gaussian1D(const int32_t kernel_size) {
    FloatTensor* ret = new FloatTensor(Int4(kernel_size, 1, 1, 1));
    const float sigma = 0.25f;
    const float amplitude = 1.0f;
    const float size = (float)kernel_size;
    const float center = size/2.0f + 0.5f;
    for (int32_t i = 0; i < kernel_size; i++) {
      ret->data_[i] = amplitude * expf(-(powf(((float)(i+1) - center) / 
        (sigma*size), 2.0f) / 2.0f));
    }
    return ret;
  }

  FloatTensor* FloatTensor::ones1D(const int32_t kernel_size) {
    FloatTensor* ret = new FloatTensor(Int4(kernel_size, 1, 1, 1));
    for (int32_t i = 0; i < kernel_size; i++) {
      ret->data_[i] = 1.0f;
    }
    return ret;
  }

  FloatTensor* FloatTensor::copy() const {
    FloatTensor* ret = new FloatTensor(dim_);
    memcpy(ret->data_, data_, sizeof(ret->data_[0]) * dataSize());
    return ret;
  }

}  // namespace hand_net
}  // namespace kinect_interface
