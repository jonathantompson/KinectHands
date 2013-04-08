//
//  linear.h
//
//  Created by Jonathan Tompson on 4/1/13.
//

#ifndef KINECT_INTERFACE_HAND_LINEAR_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "kinect_interface/hand_net/torch_stage.h"

namespace kinect_interface {
namespace hand_net {
  
  struct Linear : public TorchStage {
  public:
    // Constructor / Destructor
    Linear(const int32_t n_inputs, const int32_t n_outputs, 
      const int32_t n_threads);
    virtual ~Linear();

    virtual TorchStageType type() const { return LINEAR_STAGE; }
    virtual void forwardProp(float* input, jtil::threading::ThreadPool* tp);
    virtual int32_t outWidth() const { return n_outputs_; }
    virtual int32_t outHeight() const { return 1; }
    virtual int32_t outNFeats() const { return 1; }

    float* weights;
    float* bias;

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    int32_t n_inputs_;
    int32_t n_outputs_;
    int32_t n_threads_;

    // Multithreading primatives and functions
    float* cur_input_;
    int32_t threads_finished_;
    std::mutex thread_update_lock_;
    std::condition_variable not_finished_;
    jtil::threading::Callback<void>** thread_cbs_;  
    void forwardPropThread(const int32_t start_outf, const int32_t end_outf);

    // Non-copyable, non-assignable.
    Linear(Linear&);
    Linear& operator=(const Linear&);
  };
  
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_LINEAR_HEADER
