#include "kinect_interface/hand_net/spatial_lp_pooling.h"
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

  SpatialLPPooling::SpatialLPPooling(const float p_norm, 
    const int32_t poolsize_v, const int32_t poolsize_u) : TorchStage() {
    p_norm_ = p_norm;
    poolsize_v_ = poolsize_v;
    poolsize_u_ = poolsize_u;
    output = NULL;
    thread_cbs_ = NULL;
  }

  SpatialLPPooling::~SpatialLPPooling() {
    SAFE_DELETE(output);
    SAFE_DELETE(thread_cbs_);
  }

  void SpatialLPPooling::init(TorchData& input, ThreadPool& tp)  {
    if (input.type() != TorchDataType::FLOAT_TENSOR_DATA) {
      throw std::wruntime_error("SpatialLPPooling::init() - "
        "FloatTensor expected!");
    }
    FloatTensor& in = (FloatTensor&)input;
    if (output != NULL) {
      if (!Int4::equal(in.dim(), ((FloatTensor*)output)->dim())) {
        // Input dimension has changed!
        SAFE_DELETE(output);
        SAFE_DELETE(thread_cbs_);
      }
    }
    if (output == NULL) {
      if (in.dim()[0] % poolsize_u_ != 0 || 
        in.dim()[1] % poolsize_v_ != 0) {
        throw std::wruntime_error("width or height is not a multiple of "
          "the poolsize!");
      }
      Int4 out_dim(in.dim());
      out_dim[0] /= poolsize_u_;
      out_dim[1] /= poolsize_v_;
      output = new FloatTensor(out_dim);
    }

    if (thread_cbs_ == NULL) {
      int32_t n_threads = ((FloatTensor*)output)->dim()[2] * 
        ((FloatTensor*)output)->dim()[3];
      thread_cbs_ = new VectorManaged<Callback<void>*>(n_threads);
      for (int32_t b = 0; b < ((FloatTensor*)output)->dim()[3]; b++) {
        for (int32_t f = 0; f < ((FloatTensor*)output)->dim()[2]; f++) {
          thread_cbs_->pushBack(MakeCallableMany(
            &SpatialLPPooling::forwardPropThread, this, f, b));
        }
      }
    }
  }

  void SpatialLPPooling::forwardProp(TorchData& input, 
    jtil::threading::ThreadPool& tp) { 
    init(input, tp);
    cur_input_ = &((FloatTensor&)input);
    threads_finished_ = 0;
    for (uint32_t i = 0; i < thread_cbs_->size(); i++) {
      tp.addTask((*thread_cbs_)[i]);
    } 

    // Wait for all threads to finish
    std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
    while (threads_finished_ != thread_cbs_->size()) {
      not_finished_.wait(ul);
    }
    ul.unlock();  // Release lock
  }

  void SpatialLPPooling::forwardPropThread(const int32_t outf, 
    const int32_t outb) {
    const int32_t out_w = ((FloatTensor*)output)->dim()[0];
    const int32_t out_h = ((FloatTensor*)output)->dim()[1];
    const int32_t in_w = cur_input_->dim()[0];
    const float one_over_p_norm = 1.0f / p_norm_;

    float* out = &(*((FloatTensor*)output))(0, 0, outf, outb);
    float* in = &(*cur_input_)(0, 0, outf, outb);

    for (int32_t outv = 0; outv < out_h; outv++) {
      for (int32_t outu = 0; outu < out_w; outu++) {
        int32_t out_index = outv * out_w + outu;
        out[out_index] = 0.0f;
        // Now perform max pooling:
        for (int32_t inv = outv * poolsize_v_; inv < (outv + 1) * poolsize_v_; inv++) {
          for (int32_t inu = outu * poolsize_u_; inu < (outu + 1) * poolsize_u_; inu++) {
            float val = fabsf(in[inv * in_w + inu]);
            out[out_index] += powf(val, p_norm_);
          }
        }
        out[outv * out_w + outu] = powf(out[outv * out_w + outu], 
          one_over_p_norm);
      }
    }
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  TorchStage* SpatialLPPooling::loadFromFile(std::ifstream& file) {
    int filt_width, filt_height, pnorm;
    file.read((char*)(&filt_width), sizeof(filt_width));
    file.read((char*)(&filt_height), sizeof(filt_height));
    file.read((char*)(&pnorm), sizeof(pnorm));
    return new SpatialLPPooling((float)pnorm, filt_height, filt_width);
  }

}  // namespace hand_net
}  // namespace kinect_interface
