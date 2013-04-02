#include "kinect_interface/hand_net/spatial_max_pooling.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread_pool.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::threading;

namespace kinect_interface {
namespace hand_net {

  SpatialMaxPooling::SpatialMaxPooling(const int32_t n_feats, 
    const int32_t poolsize_v, const int32_t poolsize_u, const int32_t height, 
    const int32_t width) : TorchStage() {
    n_feats_ = n_feats;
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
        MakeCallableMany(&SpatialMaxPooling::forwardPropThread, this, i);
    }
  }

  SpatialMaxPooling::~SpatialMaxPooling() {
    SAFE_DELETE_ARR(output);
    for (int32_t i = 0; i < n_feats_; i++) {
      SAFE_DELETE(thread_cbs_[i]);
    }
    SAFE_DELETE_ARR(thread_cbs_);
  }

  void SpatialMaxPooling::forwardProp(float* input, 
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

  void SpatialMaxPooling::forwardPropThread(const int32_t outf) {
    const int32_t out_w = outWidth();
    const int32_t out_h = outHeight();
    const int32_t out_offset = outf * out_w * out_h;
    const int32_t in_offset = outf * width_ * height_;
    const int32_t in_dim = width_ * height_;

    for (int32_t outv = 0; outv < out_h; outv++) {
      for (int32_t outu = 0; outu < out_w; outu++) {
        int32_t out_index = out_offset + outv * out_w + outu;
        output[out_index] = -std::numeric_limits<float>::infinity();
        // Now perform L2 pooling:
        for (int32_t inv = outv * poolsize_v_; inv < (outv + 1) * poolsize_v_; inv++) {
          for (int32_t inu = outu * poolsize_u_; inu < (outu + 1) * poolsize_u_; inu++) {
            float val = cur_input_[in_offset + inv * width_ + inu];
            output[out_index] = std::max<float>(output[out_index], val);
          }
        }
      }
    }
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  TorchStage* SpatialMaxPooling::loadFromFile(std::ifstream& file) {
    throw std::wruntime_error("Not yet implemented");
  }

  int32_t SpatialMaxPooling::outWidth() const {
    return width_ / poolsize_u_;
  }

  int32_t SpatialMaxPooling::outHeight() const {
    return height_ / poolsize_v_;
  }

}  // namespace hand_net
}  // namespace kinect_interface
