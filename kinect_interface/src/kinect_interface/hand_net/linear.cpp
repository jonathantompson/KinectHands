#include "kinect_interface/hand_net/linear.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread_pool.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::threading;

namespace kinect_interface {
namespace hand_net {

  Linear::Linear(const int32_t n_inputs, const int32_t n_outputs, 
    const int32_t n_threads) : TorchStage() {
    n_inputs_ = n_inputs;
    n_outputs_ = n_outputs;
    n_threads_ = n_threads;

    output = new float[n_outputs_];

    int32_t n_weights = n_outputs_ * n_inputs_;
    weights = new float[n_weights];
    bias = new float[n_outputs_];

    // outputs_per_thread is rounded up (so we always cover the correct amount)
    int32_t outputs_per_thread = 1 + n_outputs_ / n_threads_;
    thread_cbs_ = new Callback<void>*[n_threads_];
    for (int32_t i = 0; i < n_threads_; i++) {
      int32_t start = i * outputs_per_thread;
      // Note: end is inclusive
      int32_t end = std::min<int32_t>((i + 1) * outputs_per_thread - 1,
        n_outputs_ - 1);
      thread_cbs_[i] = MakeCallableMany(&Linear::forwardPropThread, this, 
        start, end);
    }
  }

  Linear::~Linear() {
    SAFE_DELETE_ARR(output);
    SAFE_DELETE_ARR(weights);
    SAFE_DELETE_ARR(bias);
    for (int32_t i = 0; i < n_threads_; i++) {
      SAFE_DELETE(thread_cbs_[i]);
    }
    SAFE_DELETE_ARR(thread_cbs_);
  }

  void Linear::forwardProp(float* input, 
    jtil::threading::ThreadPool* tp) { 
    cur_input_ = input;
    threads_finished_ = 0;
    for (int32_t i = 0; i < n_threads_; i++) {
      tp->addTask(thread_cbs_[i]);
    } 

    // Wait for all threads to finish
    std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
    while (threads_finished_ != n_threads_) {
      not_finished_.wait(ul);
    }
    ul.unlock();  // Release lock
  }

  void Linear::forwardPropThread(const int32_t start_outf, 
    const int32_t end_outf) {
    for (int32_t outf = start_outf; outf <= end_outf; outf++) {
      output[outf] = bias[outf];
      for (int32_t inf = 0; inf < n_inputs_; inf++) {
        output[outf] += weights[outf * n_inputs_ + inf] * cur_input_[inf];
      }
    }

    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  TorchStage* Linear::loadFromFile(std::ifstream& file) {
    throw std::wruntime_error("Not yet implemented");
  }

}  // namespace hand_net
}  // namespace kinect_interface
