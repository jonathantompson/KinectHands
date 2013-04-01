#include "kinect_interface/hand_net/tanh.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread_pool.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::threading;

namespace kinect_interface {
namespace hand_net {

  Tanh::Tanh(const int32_t feats, const int32_t height, 
    const int32_t width, const int32_t n_threads) : TorchStage() {
    output = new float[feats * width * height];
    feats_ = feats;
    width_ = width;
    height_ = height;
    n_threads_ = n_threads;

    // pix_per_thread is rounded up (so we always cover the correct amount)
    int32_t pix_per_thread = 1 + feats_ * width_ * height_ / n_threads_;
    thread_cbs_ = new Callback<void>*[n_threads_];
    for (int32_t i = 0; i < n_threads_; i++) {
      int32_t start = i * pix_per_thread;
      // Note: end is inclusive
      int32_t end = std::min<int32_t>((i + 1) * pix_per_thread - 1,
        feats_ * width_ * height_ - 1);
      thread_cbs_[i] = MakeCallableMany(&Tanh::forwardPropThread, this, start,
        end);
    }
  }

  Tanh::~Tanh() {
    SAFE_DELETE_ARR(output);
    for (int32_t i = 0; i < n_threads_; i++) {
      SAFE_DELETE(thread_cbs_[i]);
    }
    SAFE_DELETE_ARR(thread_cbs_);
  }

  void Tanh::forwardProp(float* input, jtil::threading::ThreadPool* tp) { 
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

  void Tanh::forwardPropThread(const int32_t start, const int32_t end) {
    for (int32_t i = start; i <= end; i++) {
      // HERE IS THE CORE OF THE TANH FUNCTION
      output[i] = tanh(cur_input_[i]);
    }

    // Signify to main thread that we're done
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  TorchStage* Tanh::loadFromFile(std::ifstream& file) {
    throw std::wruntime_error("Not yet implemented");
  }

}  // namespace hand_net
}  // namespace kinect_interface
