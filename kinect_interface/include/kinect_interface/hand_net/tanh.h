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

namespace kinect_interface {
namespace hand_net {
  
  struct Tanh : public TorchStage {
  public:
    // Constructor / Destructor
    Tanh(const int32_t feats, const int32_t height, const int32_t width, 
      const int32_t n_threads);
    virtual ~Tanh();

    virtual TorchStageType type() const { return TANH_STAGE; }
    virtual void forwardProp(float* input, jtil::threading::ThreadPool* tp);
    static TorchStage* loadFromFile(std::ifstream& file);
    virtual int32_t outWidth() const { return width_; }
    virtual int32_t outHeight() const { return height_; }
    virtual int32_t outNFeats() const { return feats_; }

  private:
    int32_t feats_;
    int32_t width_;
    int32_t height_;
    int32_t n_threads_;

    // Multithreading primatives and functions
    float* cur_input_;
    int32_t threads_finished_;
    std::mutex thread_update_lock_;
    std::condition_variable not_finished_;
    jtil::threading::Callback<void>** thread_cbs_;  
    void forwardPropThread(const int32_t start, const int32_t end);

    // Non-copyable, non-assignable.
    Tanh(Tanh&);
    Tanh& operator=(const Tanh&);
  };
  
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_TANH_HEADER
