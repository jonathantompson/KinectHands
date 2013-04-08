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

#ifndef KINECT_INTERFACE_HAND_SPATIAL_CONVOLUTION_MAP_HEADER
#define KINECT_INTERFACE_HAND_SPATIAL_CONVOLUTION_MAP_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "kinect_interface/hand_net/torch_stage.h"

namespace kinect_interface {
namespace hand_net {
  
  struct SpatialConvolutionMap : public TorchStage {
  public:
    // Constructor / Destructor
    SpatialConvolutionMap(const int32_t feats_in, const int32_t feats_out,
      const int32_t fan_in, const int32_t filt_height, 
      const int32_t filt_width, const int32_t height, const int32_t width);
    virtual ~SpatialConvolutionMap();

    virtual TorchStageType type() const { return SPATIAL_CONVOLUTION_MAP_STAGE; }
    virtual void forwardProp(float* input, jtil::threading::ThreadPool* tp);
    virtual int32_t outWidth() const;
    virtual int32_t outHeight() const;
    virtual int32_t outNFeats() const { return feats_out_; }

    float** weights_;
    float* biases_;
    int16_t** conn_table_;  // This is the same as conn_table_rev in Torch
                            // For each output: [0] is input feature and [1]
                            // is the weight matrix (filter) to use

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    int32_t filt_width_;
    int32_t filt_height_;
    int32_t feats_in_;
    int32_t feats_out_;
    int32_t fan_in_;
    int32_t width_;
    int32_t height_;

    // Multithreading primatives and functions
    float* cur_input_;
    int32_t threads_finished_;
    std::mutex thread_update_lock_;
    std::condition_variable not_finished_;
    jtil::threading::Callback<void>** thread_cbs_;  
    void forwardPropThread(const int32_t outf);

    // Non-copyable, non-assignable.
    SpatialConvolutionMap(SpatialConvolutionMap&);
    SpatialConvolutionMap& operator=(const SpatialConvolutionMap&);
  };
  
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_SPATIAL_CONVOLUTION_MAP_HEADER
