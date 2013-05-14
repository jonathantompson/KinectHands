//
//  float_tensor.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  Simplified C++ replica of torch.FloatTensor.  Up to 3D is supported.
//
//  This is escentially a wrap around my opencl buffer class.  To write data
//

#ifndef JTORCH_FLOAT_TENSOR_HEADER
#define JTORCH_FLOAT_TENSOR_HEADER

#include <iostream>
#include <iomanip>
#include <fstream>
#include "jcl/jcl.h"
#include "jtil/math/math_types.h"
#include "jtorch/torch_data.h"

namespace jtil { namespace threading { class ThreadPool; } }

namespace jtorch {
  
  class FloatTensor : public TorchData {
  public:
    // Constructor / Destructor
    FloatTensor(const jtil::math::Int3& dim);  // Assumes dim[3]=1
    FloatTensor(const jtil::math::Int2& dim);  // Assumes dim[3]=1, dim[2]=1
    FloatTensor(const int dim);  // Assumes dim[3]=1, dim[2]=1, dim[1]=1
    virtual ~FloatTensor();

    virtual TorchDataType type() const { return FLOAT_TENSOR_DATA; }

    // setData and getData are EXPENSIVE --> They require a CPU to GPU copy
    void setData(const float* data);
    void getData(float* data);

    const jtil::math::Int3& dim() const { return dim_; }

    // Print --> EXPENSIVE
    virtual void print();  // print to std::cout
    void print(const jtil::math::Int2& interval0, 
      const jtil::math::Int2& interval1, 
      const jtil::math::Int2& interval2);

    // Deep copy --> EXPENSIVE
    FloatTensor* copy() const;

    // gaussian1D In torch: >> normkernel = image.gaussian1D(n)
    // It's a gaussian of x = -2*sigma to 2*sigma, where sigma = size / 2
    static FloatTensor* gaussian1D(const int32_t kernel_size);
    static FloatTensor* ones1D(const int32_t kernel_size);

    inline jcl::JCLBuffer data() { return data_; }
    inline uint32_t dataSize() const { return dim_[0]*dim_[1]*dim_[2]; }

  protected:
    jcl::JCLBuffer data_;  // Internal data
    jtil::math::Int3 dim_;  // dim_[0] is lowest contiguous dimension, 
                             // dim_[2] is highest dimension

    // Non-copyable, non-assignable.
    FloatTensor(FloatTensor&);
    FloatTensor& operator=(const FloatTensor&);
  };

};  // namespace jtorch

#endif  // JTORCH_FLOAT_TENSOR_HEADER
