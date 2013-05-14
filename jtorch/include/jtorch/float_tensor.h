//
//  float_tensor.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  Simplified C++ replica of torch.FloatTensor.  Up to 4D is supported
//

#ifndef JTORCH_FLOAT_TENSOR_HEADER
#define JTORCH_FLOAT_TENSOR_HEADER

#include <iostream>
#include <iomanip>
#include <fstream>
#include "jtil/math/math_types.h"
#include "jtorch/torch_data.h"

namespace jtil { namespace threading { class ThreadPool; } }

namespace jtorch {
  
  class FloatTensor : public TorchData {
  public:
    // Constructor / Destructor
    FloatTensor(const jtil::math::Int4& dim);  
    FloatTensor(const jtil::math::Int3& dim);  // Assumes dim[3]=1
    FloatTensor(const jtil::math::Int2& dim);  // Assumes dim[3]=1, dim[2]=1
    FloatTensor(const int dim);  // Assumes dim[3]=1, dim[2]=1, dim[1]=1
    virtual ~FloatTensor();

    virtual TorchDataType type() const { return FLOAT_TENSOR_DATA; }

    float& operator()(const uint32_t u, const uint32_t v, const uint32_t f, 
      const uint32_t b);  // u is the lowest contiguous dimension
    const jtil::math::Int4& dim() const { return dim_; }

    virtual void print() const;  // print to std::cout
    void print(const jtil::math::Int2& interval0, 
      const jtil::math::Int2& interval1, const jtil::math::Int2& interval2, 
      const jtil::math::Int2& interval3) const;

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
    jtil::math::Int4 dim_;  // dim_[0] is lowest contiguous dimension, 
                             // dim_[3] is highest dimension

    // Non-copyable, non-assignable.
    FloatTensor(FloatTensor&);
    FloatTensor& operator=(const FloatTensor&);
  };

};  // namespace jtorch

#endif  // JTORCH_FLOAT_TENSOR_HEADER
