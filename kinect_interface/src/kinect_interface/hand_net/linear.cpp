#include "kinect_interface/hand_net/linear.h"
#include "kinect_interface/hand_net/float_tensor.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread.h"
#include "jtil/threading/callback.h"
#include "jtil/threading/thread_pool.h"
#include "jtil/data_str/vector_managed.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::threading;
using namespace jtil::math;
using namespace jtil::data_str;

namespace kinect_interface {
namespace hand_net {

  Linear::Linear(const int32_t n_inputs, const int32_t n_outputs) 
    : TorchStage() {
    n_inputs_ = n_inputs;
    n_outputs_ = n_outputs;

    output = new FloatTensor(Int4(n_outputs_, 1, 1, 1));
    thread_cbs_ = NULL;

    int32_t n_weights = n_outputs_ * n_inputs_;
    weights = new float[n_weights];
    bias = new float[n_outputs_];
  }

  Linear::~Linear() {
    SAFE_DELETE(output);
    SAFE_DELETE_ARR(weights);
    SAFE_DELETE_ARR(bias);
    SAFE_DELETE(thread_cbs_);
  }

  void Linear::init(FloatTensor& input, ThreadPool& tp)  {
    if (input.dim()[1] != 1 || input.dim()[1] != 1 || input.dim()[1] != 1) {
      throw std::wruntime_error("Linear::init() - ERROR: 1D input expected!");
    }
    if (input.dim()[0] != n_inputs_) {
      throw std::wruntime_error("Linear::init() - ERROR: input size mismatch!");
    }
    if (thread_cbs_ == NULL) {
      int32_t n_threads = tp.num_workers();
      // pix_per_thread is rounded up (so we always cover the correct amount)
      int32_t input_size = input.dataSize();
      int32_t pix_per_thread = 1 + input.dataSize() / n_threads;
      thread_cbs_ = new VectorManaged<Callback<void>*>(n_threads);
      for (uint32_t i = 0; i < n_threads; i++) {
        int32_t start = i * pix_per_thread;
        // Note: end is inclusive
        int32_t end = std::min<int32_t>((i + 1) * pix_per_thread - 1,
          input_size - 1);
        thread_cbs_->pushBack(MakeCallableMany(&Linear::forwardPropThread, 
          this, start, end));
      }
    }
  }

  void Linear::forwardProp(FloatTensor& input, ThreadPool& tp) { 
    init(input, tp);
    cur_input_ = input.data();
    cur_output_ = output->data();
    threads_finished_ = 0;
    for (int32_t i = 0; i < thread_cbs_->size(); i++) {
      tp.addTask((*thread_cbs_)[i]);
    } 

    // Wait for all threads to finish
    std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
    while (threads_finished_ != thread_cbs_->size()) {
      not_finished_.wait(ul);
    }
    ul.unlock();  // Release lock
  }

  void Linear::forwardPropThread(const int32_t start_outf, 
    const int32_t end_outf) {
    for (int32_t outf = start_outf; outf <= end_outf; outf++) {
      cur_output_[outf] = bias[outf];
      for (int32_t inf = 0; inf < n_inputs_; inf++) {
        cur_output_[outf] += weights[outf * n_inputs_ + inf] * cur_input_[inf];
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
