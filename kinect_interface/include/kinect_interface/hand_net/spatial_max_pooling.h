//
//  spatial_max_pooling.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  Multithreading is not all that efficient:  Threads are split up per output 
//  feature.
//

#ifndef KINECT_INTERFACE_HAND_SPATIAL_MAX_POOLING_HEADER
#define KINECT_INTERFACE_HAND_SPATIAL_MAX_POOLING_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "kinect_interface/hand_net/torch_stage.h"

namespace kinect_interface {
namespace hand_net {
  
  struct SpatialMaxPooling : public TorchStage {
  public:
    // Constructor / Destructor
    SpatialMaxPooling(const int32_t n_feats, const int32_t poolsize_v, 
      const int32_t poolsize_u, const int32_t height, const int32_t width);
    virtual ~SpatialMaxPooling();

    virtual TorchStageType type() const { return SPATIAL_MAX_POOLING_STAGE; }
    virtual void forwardProp(float* input, jtil::threading::ThreadPool* tp);
    virtual int32_t outWidth() const;
    virtual int32_t outHeight() const;
    virtual int32_t outNFeats() const { return n_feats_; }

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    int32_t n_feats_;
    int32_t poolsize_v_;
    int32_t poolsize_u_;
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
    SpatialMaxPooling(SpatialMaxPooling&);
    SpatialMaxPooling& operator=(const SpatialMaxPooling&);
  };
  
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_SPATIAL_MAX_POOLING_HEADER
