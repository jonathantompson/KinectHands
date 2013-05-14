//
//  spatial_convolution_map.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  Same as SpatialConvolution except that the output features aren't fully
//  connected to the input features (so we need to keep around a connection
//  table).
//
//  Multithreading is not all that efficient:  Threads are split up per output 
//  feature.
//

#ifndef JTORCH_SPATIAL_CONVOLUTION_MAP_HEADER
#define JTORCH_SPATIAL_CONVOLUTION_MAP_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "jtorch/torch_stage.h"
#include "jcl/jcl.h"  // For jcl::JCLBuffer

namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace jtorch {
  
  class SpatialConvolutionMap : public TorchStage {
  public:
    // Constructor / Destructor
    SpatialConvolutionMap(const int32_t feats_in, const int32_t feats_out,
      const int32_t fan_in, const int32_t filt_height, 
      const int32_t filt_width);
    virtual ~SpatialConvolutionMap();

    virtual TorchStageType type() const { return SPATIAL_CONVOLUTION_MAP_STAGE; }
    virtual void forwardProp(TorchData& input);

    void setWeights(const float* weights);
    void setBiases(const float* biases);
    void setConnTable(const int* conn_table);

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    int32_t filt_width_;
    int32_t filt_height_;
    int32_t feats_in_;
    int32_t feats_out_;
    int32_t fan_in_;

    // weights_buf_:    dim[2] --> matrix_index (size = feats_out_ * fan_in_) 
    //                  dim[1] --> filter height
    //                  dim[0] --> filter width
    Tensor<float>* weights_;
    // biases_buf_:     dim[0] --> feats_out_t
    Tensor<float>* biases_;
    // conn_table_buf_: dim[1] --> feats_out_t
    //                  dim[0] --> 2 (0 is the input feature and 1 is the matrix index)
    Tensor<int>* conn_table_;

    jtil::math::Int3 local_worgroup_size;

    void init(TorchData& input);

    // Non-copyable, non-assignable.
    SpatialConvolutionMap(SpatialConvolutionMap&);
    SpatialConvolutionMap& operator=(const SpatialConvolutionMap&);
  };
  
};  // namespace jtorch

#endif  // JTORCH_SPATIAL_CONVOLUTION_MAP_HEADER
