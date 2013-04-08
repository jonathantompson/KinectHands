//
//  float_tensor.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  Simplified C++ replica of torch.FloatTensor.  Up to 4D is supported
//

#ifndef KINECT_INTERFACE_HAND_NET_FLOAT_TENSOR_HEADER
#define KINECT_INTERFACE_HAND_NET_FLOAT_TENSOR_HEADER

#include <iostream>
#include <iomanip>
#include <fstream>
#include "jtil/math/math_types.h"

namespace jtil { namespace threading { class ThreadPool; } }

namespace kinect_interface {
namespace hand_net {
  
  class FloatTensor {
  public:
    // Constructor / Destructor
    FloatTensor(const jtil::math::Uint4& dim);  
    FloatTensor(const jtil::math::Uint3& dim);  // Assumes dim[3]=1
    FloatTensor(const jtil::math::Uint2& dim);  // Assumes dim[3]=1, dim[2]=1
    FloatTensor(const int dim);  // Assumes dim[3]=1, dim[2]=1, dim[1]=1
    virtual ~FloatTensor();

    float& operator()(const uint32_t x, const uint32_t y, const uint32_t z, 
      const uint32_t w);  // x is the lowest contiguous dimension
    const jtil::math::Uint4& dim() const { return dim_; }

    void print() const;  // print to std::cout

    // Deep copy
    FloatTensor* copy() const;

    // gaussian1D In torch: >> normkernel = image.gaussian1D(n)
    // It's a gaussian of x = -2*sigma to 2*sigma, where sigma = size / 2
    static FloatTensor* gaussian1D(const int32_t kernel_size);
    static FloatTensor* ones1D(const int32_t kernel_size);

    inline float* data() { return data_; }
    inline uint32_t dataSize() const { return dim_[0]*dim_[1]*dim_[2]*dim_[3]; }

  protected:
    float* data_;  // Internal data
    jtil::math::Uint4 dim_;  // dim_[0] is lowest contiguous dimension, 
                             // dim_[3] is highest dimension

    // Non-copyable, non-assignable.
    FloatTensor(FloatTensor&);
    FloatTensor& operator=(const FloatTensor&);
  };

};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_FLOAT_TENSOR_HEADER
