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

namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace kinect_interface {
namespace hand_net {
  
  class SpatialMaxPooling : public TorchStage {
  public:
    // Constructor / Destructor
    SpatialMaxPooling(const int32_t poolsize_v, const int32_t poolsize_u);
    virtual ~SpatialMaxPooling();

    virtual TorchStageType type() const { return SPATIAL_MAX_POOLING_STAGE; }
    virtual void forwardProp(FloatTensor& input, 
      jtil::threading::ThreadPool& tp);

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    int32_t poolsize_v_;
    int32_t poolsize_u_;

    // Multithreading primatives and functions
    FloatTensor* cur_input_;
    int32_t threads_finished_;
    std::mutex thread_update_lock_;
    std::condition_variable not_finished_;
    jtil::data_str::VectorManaged<jtil::threading::Callback<void>*>* thread_cbs_; 

    void forwardPropThread(const int32_t outf, const int32_t outb);

    void init(FloatTensor& input, jtil::threading::ThreadPool& tp);

    // Non-copyable, non-assignable.
    SpatialMaxPooling(SpatialMaxPooling&);
    SpatialMaxPooling& operator=(const SpatialMaxPooling&);
  };
  
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_SPATIAL_MAX_POOLING_HEADER
