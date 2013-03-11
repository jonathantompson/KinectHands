#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <iomanip>
#include <fstream>
#include "kinect_interface/hand_net/nn_stage.h"
#include "kinect_interface/hand_net/hand_net.h"  // for HN_NUM_WORKER_THREADS
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread_pool.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) do { if (x != NULL) { delete[] x; x = NULL; } } while (0); 

using jtil::math::Float4x4;
using jtil::math::FloatQuat;
using jtil::math::Float3;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using jtil::math::Float3;
using namespace jtil::threading;

namespace kinect_interface {
namespace hand_net {
 
  NNStage::NNStage() {
    n_inputs_ = 0;
    n_outputs_ = 0;
    nonlin_type_ = UndefinedNNNonlin;
    weights_ = NULL;
    bias_ = NULL;
    linnet_thread_cbs_ = NULL;
    nonlin_thread_cbs_ = NULL;
  }

  NNStage::~NNStage() {
    SAFE_DELETE(weights_);
    SAFE_DELETE(bias_);
    if (linnet_thread_cbs_) {
      for (int32_t i = 0; i < HN_NUM_WORKER_THREADS; i++) {
        SAFE_DELETE(linnet_thread_cbs_[i]);
      }
    }
    SAFE_DELETE_ARR(linnet_thread_cbs_);
    if (nonlin_thread_cbs_) {
      for (int32_t i = 0; i < HN_NUM_WORKER_THREADS; i++) {
        SAFE_DELETE(nonlin_thread_cbs_[i]);
      }
    }
    SAFE_DELETE_ARR(nonlin_thread_cbs_);
  }

  void NNStage::loadFromFile(std::ifstream& file) {
    file.read((char*)(&n_outputs_), sizeof(n_outputs_));
    file.read((char*)(&n_inputs_), sizeof(n_inputs_));

    int32_t n_weights = n_outputs_ * n_inputs_;

    weights_ = new float[n_weights];
    file.read((char*)(weights_), sizeof(weights_[0]) * n_weights);

    int32_t nonlin_type;
    file.read((char*)(&nonlin_type), sizeof(nonlin_type));
    nonlin_type_ = (NNNonlinType)nonlin_type;

    bias_ = new float[n_outputs_];
    file.read((char*)(bias_), sizeof(bias_[0]) * n_outputs_);

    // Now chunk up the output range into seperate groups so that one thread
    // can work on each chunk
    linnet_thread_cbs_ = new Callback<void>*[HN_NUM_WORKER_THREADS];
    nonlin_thread_cbs_ = new Callback<void>*[HN_NUM_WORKER_THREADS];
    const int32_t nthr = HN_NUM_WORKER_THREADS;
    int32_t n_outputs_per_thread = n_outputs_ / nthr;
    for (int32_t i = 0; i < (nthr - 1); i++) {
      int32_t start = i * n_outputs_per_thread;
      int32_t end = ((i + 1) * n_outputs_per_thread) - 1;
      linnet_thread_cbs_[i] = 
        MakeCallableMany(&NNStage::performLinearNetRange, this, start, end);
      nonlin_thread_cbs_[i] = 
        MakeCallableMany(&NNStage::performNonlinRange, this, start, end);
    }
    // The last thread may have more pixels then the other due to integer
    // round off, but this is OK.
    linnet_thread_cbs_[nthr-1] = 
      MakeCallableMany(&NNStage::performLinearNetRange, this, 
      (nthr-1) * n_outputs_per_thread, n_outputs_-1);
    nonlin_thread_cbs_[nthr-1] = 
      MakeCallableMany(&NNStage::performNonlinRange, this, 
      (nthr-1) * n_outputs_per_thread, n_outputs_-1);
  }

  void NNStage::printToStdOut() const {
    std::cout << "  n_outputs_ = " << n_outputs_ << std::endl;
    std::cout << "  n_inputs_ = " << n_inputs_ << std::endl;
    std::cout << "  nonlin_type_ = " << (int32_t)nonlin_type_ << std::endl;

    std::cout << std::setprecision(6);
    std::cout << std::fixed;

    uint32_t i = 0;
    uint32_t n_print = std::min<int32_t>(n_outputs_ * n_inputs_, 
      NN_MAX_PRINT_LENGTH);
    std::cout.setf(0, std::ios::showpos);
    std::cout << "  weights_ =" << std::endl;
    for (int32_t v = 0; v < n_outputs_ && i < n_print; v++) {
      if (v == 0) {
        std::cout << "    (0,0) ";
      } else {
        std::cout << "          ";
      }
      std::cout.setf(std::ios::showpos);
      for (int32_t u = 0; u < n_inputs_ && i < n_print; u++) {
        std::cout << weights_[v * n_inputs_ + u];
        if (u != n_inputs_ - 1) {
          std::cout << ", ";
        } else {
          std::cout << std::endl;
        }
        i++;
      }
    }

    std::cout << std::endl;
    std::cout << std::resetiosflags(std::ios_base::showpos);
  }

  int32_t NNStage::dataSizeReq() const {
    return std::max<int32_t>(n_inputs_, n_outputs_);
  }

  void NNStage::forwardProp(float*& in, float*& out, ThreadPool* tp) {
    performLinearNet(out, (const float*&)in, tp);
    performNonlin(out, tp);

    // Result is in output
  }

  void NNStage::performNonlin(float*&data, ThreadPool* tp) {
    if (tp->num_workers() != HN_NUM_WORKER_THREADS) {
      throw std::wruntime_error("NNStage::performNonlin() - INTERNAL ERROR: "
        "thread pool size is not what we expected!");
    }
    cur_out_ = data;
    threads_finished_ = 0;
    for (uint32_t i = 0; i < HN_NUM_WORKER_THREADS; i++) {
      tp->addTask(nonlin_thread_cbs_[i]);
    }

    // Wait until the other threads are done
    std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
    while (threads_finished_ != HN_NUM_WORKER_THREADS) {
      not_finished_.wait(ul);
    }
    ul.unlock();  // Release lock
  }

void NNStage::performNonlinRange(const int32_t start, const int32_t end) {
    switch (nonlin_type_) {
    case NoneNNNonlin:
      // Nothing to do for this non-linearity type
      break;
    case TanhNNNonlin:
      for (int32_t outf = start; outf <= end; outf++) {
        cur_out_[outf] = tanh(cur_out_[outf]);
      }
      break;
    default:
      throw std::wruntime_error("NNStage::performNonlinearity() - ERROR: "
        "Only TanhNNNonlin supported for now.");
    }

    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  void NNStage::performLinearNet(float*&out, const float*& in, 
    ThreadPool* tp) {
    if (tp->num_workers() != HN_NUM_WORKER_THREADS) {
      throw std::wruntime_error("NNStage::performNonlin() - INTERNAL ERROR: "
        "thread pool size is not what we expected!");
    }
    cur_out_ = out;
    cur_in_ = in;
    threads_finished_ = 0;
    for (uint32_t i = 0; i < HN_NUM_WORKER_THREADS; i++) {
      tp->addTask(linnet_thread_cbs_[i]);
    }

    // Wait until the other threads are done
    std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
    while (threads_finished_ != HN_NUM_WORKER_THREADS) {
      not_finished_.wait(ul);
    }
    ul.unlock();  // Release lock
  }

  void NNStage::performLinearNetRange(const int32_t start, const int32_t end) {
    for (int32_t outf = start; outf <= end; outf++) {
      cur_out_[outf] = bias_[outf];
      for (int32_t inf = 0; inf < n_inputs_; inf++) {
        cur_out_[outf] += weights_[outf * n_inputs_ + inf] * cur_in_[inf];
      }
    }

    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

}  // namespace hand_net
}  // namespace kinect_interface
