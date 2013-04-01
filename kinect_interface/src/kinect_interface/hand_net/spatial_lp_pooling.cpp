#include "kinect_interface/hand_net/spatial_lp_pooling.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread_pool.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::threading;

namespace kinect_interface {
namespace hand_net {

  SpatialLPPooling::SpatialLPPooling(const int32_t n_feats, 
    const float p_norm, const int32_t poolsize_v, const int32_t poolsize_u, 
    const int32_t height, const int32_t width) : TorchStage() {
    n_feats_ = n_feats;
    p_norm_ = p_norm;
    poolsize_v_ = poolsize_v;
    poolsize_u_ = poolsize_u;
    width_ = width;
    height_ = height;

    if (width_ % poolsize_u_ != 0 || height_ % poolsize_v_ != 0) {
      throw std::wruntime_error("width or height is not a multiple of "
        "the poolsize!");
    }

    output = new float[n_feats_ * outWidth() * outHeight()];

    thread_cbs_ = new Callback<void>*[n_feats_];
    for (int32_t i = 0; i < n_feats_; i++) {
      thread_cbs_[i] = 
        MakeCallableMany(&SpatialLPPooling::forwardPropThread, this, i);
    }
  }

  SpatialLPPooling::~SpatialLPPooling() {
    SAFE_DELETE_ARR(output);
    for (int32_t i = 0; i < n_feats_; i++) {
      SAFE_DELETE(thread_cbs_[i]);
    }
    SAFE_DELETE_ARR(thread_cbs_);
  }

  void SpatialLPPooling::forwardProp(float* input, 
    jtil::threading::ThreadPool* tp) { 
    cur_input_ = input;
    threads_finished_ = 0;
    for (int32_t i = 0; i < n_feats_; i++) {
      tp->addTask(thread_cbs_[i]);
    } 

    // Wait for all threads to finish
    std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
    while (threads_finished_ != n_feats_) {
      not_finished_.wait(ul);
    }
    ul.unlock();  // Release lock
  }

  void SpatialLPPooling::forwardPropThread(const int32_t outf) {
    const int32_t out_w = outWidth();
    const int32_t out_h = outHeight();
    const int32_t out_offset = outf * out_w * out_h;
    const int32_t in_offset = outf * width_ * height_;
    const int32_t in_dim = width_ * height_;

    const float one_over_p_norm = 1.0f / p_norm_;

    for (int32_t outv = 0; outv < out_h; outv++) {
      for (int32_t outu = 0; outu < out_w; outu++) {
        output[out_offset + outv * out_w + outu] = 0.0f;
        // Now perform L2 pooling:
        for (int32_t inv = outv * poolsize_v_; inv < (outv + 1) * poolsize_v_; inv++) {
          for (int32_t inu = outu * poolsize_u_; inu < (outu + 1) * poolsize_u_; inu++) {
            float val = fabsf(cur_input_[in_offset + inv * width_ + inu]);
            output[out_offset + outv * out_w + outu] += powf(val, p_norm_);
          }
        }
        output[out_offset + outv * out_w + outu] = 
          powf(output[out_offset + outv * out_w + outu], one_over_p_norm);
      }
    }
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  TorchStage* SpatialLPPooling::loadFromFile(std::ifstream& file) {
    throw std::wruntime_error("Not yet implemented");
  }

  int32_t SpatialLPPooling::outWidth() const {
    return width_ / poolsize_u_;
  }

  int32_t SpatialLPPooling::outHeight() const {
    return height_ / poolsize_v_;
  }

}  // namespace hand_net
}  // namespace kinect_interface
