//
//  threshold.h
//
//  Created by Jonathan Tompson on 4/1/13.
//

#ifndef KINECT_INTERFACE_HAND_NET_THRESHOLD_HEADER
#define KINECT_INTERFACE_HAND_NET_THRESHOLD_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "kinect_interface/hand_net/torch_stage.h"

namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace kinect_interface {
namespace hand_net {
  
  class Threshold : public TorchStage {
  public:
    // Constructor / Destructor
    Threshold();
    virtual ~Threshold();

    virtual TorchStageType type() const { return THRESHOLD_STAGE; }
    virtual void forwardProp(TorchData& input, 
      jtil::threading::ThreadPool& tp);

    float threshold;  // Single threshold value
    float val;  // Single output value (when input < threshold)

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    // Multithreading primatives and functions
    float* cur_input_;
    float* cur_output_;
    int32_t threads_finished_;
    std::mutex thread_update_lock_;
    std::condition_variable not_finished_;
    jtil::data_str::VectorManaged<jtil::threading::Callback<void>*>* thread_cbs_;  

    void forwardPropThread(const int32_t start, const int32_t end);

    void init(TorchData& input, jtil::threading::ThreadPool& tp);

    // Non-copyable, non-assignable.
    Threshold(Threshold&);
    Threshold& operator=(const Threshold&);
  };
  
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_THRESHOLD_HEADER
