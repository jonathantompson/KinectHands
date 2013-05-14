#include <sstream>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include "jtorch/float_tensor.h"
#include "jtorch/jtorch.h"
#include "jtil/exceptions/wruntime_error.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::math;
using namespace jcl;

namespace jtorch {

  FloatTensor::FloatTensor(const jtil::math::Int3& dim) {
    dim_.set(dim[0], dim[1], dim[2]);
    data_ = jtorch::cl_context->allocateBuffer(CLBufferTypeReadWrite,
      dim_[0], dim_[1], dim_[2]);
  }

  FloatTensor::FloatTensor(const jtil::math::Int2& dim) {
    dim_.set(dim[0], dim[1], 1);
    data_ = jtorch::cl_context->allocateBuffer(CLBufferTypeReadWrite,
      dim_[0], dim_[1], dim_[2]);
  }

  FloatTensor::FloatTensor(const int dim) {
    dim_.set(dim, 1, 1);
    data_ = jtorch::cl_context->allocateBuffer(CLBufferTypeReadWrite,
      dim_[0], dim_[1], dim_[2]);
  }

  FloatTensor::~FloatTensor() {
    // Nothing to do
  }

  void FloatTensor::setData(const float* data) {
    jtorch::cl_context->writeToBuffer(data, jtorch::deviceid, data_, true);
  }

  void FloatTensor::getData(float* data) {
    jtorch::cl_context->readFromBuffer(data, jtorch::deviceid, data_, true);
  }

  void FloatTensor::print() {
    float* d = new float[dataSize()];
    getData(d);
    const int32_t dim = dim_[0] * dim_[1];
    for (int32_t i = 0; i < dim_[2]; i++) {
      std::cout.setf(0, std::ios::showpos);
      std::cout << "  3dtensor[" << i << ", *, *] =";
      std::cout << std::endl;
      float* data = &d[i * dim_[1]*dim_[0]];
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
    std::cout << std::resetiosflags(std::ios_base::showpos);
    delete[] d;
  };

  void FloatTensor::print(const jtil::math::Int2& interval0, 
    const jtil::math::Int2& interval1, const jtil::math::Int2& interval2) {
    if (interval0[0] > interval0[1] || interval1[0] > interval1[1] || 
      interval2[0] > interval2[1]) {
      throw std::wruntime_error("FloatTensor::print() - ERROR: "
        "intervals must be monotonic");
    }
    if (interval0[0] < 0 || interval0[1] >= dim_[0] || 
      interval1[0] < 0 || interval1[1] >= dim_[1] ||
      interval2[0] < 0 || interval2[1] >= dim_[2]) {
      throw std::wruntime_error("FloatTensor::print() - ERROR: "
        "intervals out of range");
    }
    float* d = new float[dataSize()];
    getData(d);
    for (int32_t f = interval2[0]; f <= interval2[1]; f++) {
      std::cout.setf(0, std::ios::showpos);
      std::cout << "  3dtensor[" << f << ", *, *] =";
      std::cout << std::endl;
      float* data = &d[f * dim_[1]*dim_[0]];
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
    delete[] d;
  }

  FloatTensor* FloatTensor::gaussian1D(const int32_t kernel_size) {
    FloatTensor* ret = new FloatTensor(Int3(kernel_size, 1, 1));
    const float sigma = 0.25f;
    const float amplitude = 1.0f;
    const float size = (float)kernel_size;
    const float center = size/2.0f + 0.5f;
    float* data = new float[kernel_size];
    for (int32_t i = 0; i < kernel_size; i++) {
      data[i] = amplitude * expf(-(powf(((float)(i+1) - center) / 
        (sigma*size), 2.0f) / 2.0f));
    }
    ret->setData(data);
    delete[] data;
    return ret;
  }

  FloatTensor* FloatTensor::ones1D(const int32_t kernel_size) {
    FloatTensor* ret = new FloatTensor(Int3(kernel_size, 1, 1));
    float* data = new float[kernel_size];
    for (int32_t i = 0; i < kernel_size; i++) {
      data[i] = 1.0f;
    }
    ret->setData(data);
    delete[] data;
    return ret;
  }

  FloatTensor* FloatTensor::copy() const {
    FloatTensor* ret = new FloatTensor(dim_);
    float* data = new float[dataSize()];
    ret->setData(data);
    delete[] data;
    return ret;
  }

}  // namespace jtorch
