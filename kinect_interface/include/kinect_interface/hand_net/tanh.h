//
//  tanh.h
//
//  Created by Jonathan Tompson on 4/1/13.
//

#ifndef KINECT_INTERFACE_HAND_NET_TANH_HEADER
#define KINECT_INTERFACE_HAND_NET_TANH_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "kinect_interface/hand_net/torch_stage.h"

namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace kinect_interface {
namespace hand_net {
  
  class Tanh : public TorchStage {
  public:
    // Constructor / Destructor
    Tanh();
    virtual ~Tanh();

    virtual TorchStageType type() const { return TANH_STAGE; }
    virtual void forwardProp(FloatTensor& input, 
      jtil::threading::ThreadPool& tp);

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

    void init(FloatTensor& input, jtil::threading::ThreadPool& tp);

    // Non-copyable, non-assignable.
    Tanh(Tanh&);
    Tanh& operator=(const Tanh&);
  };
  
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_TANH_HEADER
