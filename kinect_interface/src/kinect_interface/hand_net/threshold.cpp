#include "kinect_interface/hand_net/threshold.h"
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

  Threshold::Threshold() : TorchStage() {
    output = NULL;
    thread_cbs_ = NULL;
    threshold = 1e-6f;
    val = 0;
  }

  Threshold::~Threshold() {
    SAFE_DELETE(output);
    SAFE_DELETE(thread_cbs_);
  }

  void Threshold::init(FloatTensor& input, jtil::threading::ThreadPool& tp)  {
    if (output != NULL) {
      if (!Uint4::equal(input.dim(), output->dim())) {
        // Input dimension has changed!
        delete output;
        output = NULL;
        delete thread_cbs_;
        thread_cbs_ = NULL;
      }
    }
    if (output == NULL) {
      output = new FloatTensor(input.dim());
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
        thread_cbs_->pushBack(MakeCallableMany(&Threshold::forwardPropThread, 
          this, start, end));
      }
    }
  }

  void Threshold::forwardProp(FloatTensor& input, ThreadPool& tp) { 
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

  void Threshold::forwardPropThread(const int32_t start, const int32_t end) {
    for (int32_t i = start; i <= end; i++) {
      // HERE IS THE CORE OF THE THRESHOLD FUNCTION
      cur_output_[i] = cur_input_[i] > threshold ? cur_input_[i] : val;
    }

    // Signify to main thread that we're done
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  TorchStage* Threshold::loadFromFile(std::ifstream& file) {
    Threshold* ret = new Threshold();
    file.read((char*)(&ret->threshold), sizeof(ret->threshold));
    file.read((char*)(&ret->val), sizeof(ret->val));
  }

}  // namespace hand_net
}  // namespace kinect_interface
