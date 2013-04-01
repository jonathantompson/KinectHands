#include "kinect_interface/hand_net/spatial_subtractive_normalization.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread_pool.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::threading;

namespace kinect_interface {
namespace hand_net {

  SpatialSubtractiveNormalization::SpatialSubtractiveNormalization(
    const int32_t n_feats, const uint32_t kernel1d_size, const float* kernel1d, 
    const int32_t height, const int32_t width) : TorchStage() {
    kernel1d_size_ = kernel1d_size;
    n_feats_ = n_feats;
    width_ = width;
    height_ = height;

    output = new float[n_feats_ * outWidth() * outHeight()];
    kernel1d_ = new float[kernel1d_size_];
    memcpy(kernel1d_, kernel1d, kernel1d_size_ * sizeof(kernel1d_[0]));
    // Now normalize the kernel!
    float sum = 0.0f;
    for (int32_t i = 0; i < kernel1d_size_; i++) {
      sum += kernel1d_[i];
    }
    for (int32_t i = 0; i < kernel1d_size_; i++) {
      kernel1d_[i] /= sum;
    }

    // Filter an image of all 1 values to create the normalization constants
    // See norm_test.lua for proof that this works as well as:
    // https://github.com/andresy/torch/blob/master/extra/nn/SpatialSubtractiveNormalization.lua
    // The filter is seperable, but we'll just do the dumb 2D version since
    // we only do this once on startup.  --> O(n * m), rather than O(n * 2)
    norm_coef_ = new float[width * height];
    int32_t filt_rad = (kernel1d_size_ - 1) / 2;
    for (int32_t v = 0; v < height; v++) {
      for (int32_t u = 0; u < width; u++) {
        norm_coef_[v * width + u] = 0.0f;
        for (int32_t v_filt = -filt_rad; v_filt <= filt_rad; v_filt++) {
          for (int32_t u_filt = -filt_rad; u_filt <= filt_rad; u_filt++) {
            int32_t u_in = u + u_filt;
            int32_t v_in = v + v_filt;
            if (u_in >= 0 && u_in < width && v_in >= 0 && v_in < height) {
              // Pixel is inside --> We'll effectively clamp zeros elsewhere.
              norm_coef_[v * width + u] += (kernel1d_[v_filt + filt_rad] *
                kernel1d_[u_filt + filt_rad]);
            }
          }
        }
        norm_coef_[v * width + u] /= n_feats_;
      }
    }

    // Also allocate temporary space we might need:
    norm_accum_ = new float[width * height];
    norm_filt_tmp_ = new float[width * height];

    thread_cbs_ = new Callback<void>*[n_feats_];
    for (int32_t i = 0; i < n_feats_; i++) {
      thread_cbs_[i] = 
        MakeCallableMany(&SpatialSubtractiveNormalization::normalizeFeature, 
        this, i);
    }
  }

  SpatialSubtractiveNormalization::~SpatialSubtractiveNormalization() {
    SAFE_DELETE_ARR(output);
    SAFE_DELETE_ARR(kernel1d_);
    SAFE_DELETE_ARR(norm_coef_);
    SAFE_DELETE_ARR(norm_accum_);
    SAFE_DELETE_ARR(norm_filt_tmp_);
    for (int32_t i = 0; i < n_feats_; i++) {
      SAFE_DELETE(thread_cbs_[i]);
    }
    SAFE_DELETE_ARR(thread_cbs_);
  }

  void SpatialSubtractiveNormalization::forwardProp(float* input, 
    jtil::threading::ThreadPool* tp) { 
    const int32_t im_dim = width_ * height_;
    int32_t filt_rad = (kernel1d_size_ - 1) / 2;

    // Zero out the accumulator
    for (int32_t i = 0; i < im_dim; i++) {
      norm_accum_[i] = 0.0f;
    }
    for (int32_t outf = 0; outf < n_feats_; outf++) {
      // The filter is seperable --> Filter HORIZONTALLY first
      for (int32_t v = 0; v < height_; v++) {
        for (int32_t u = 0; u < width_; u++) {
          norm_filt_tmp_[v * width_ + u] = 0.0f;
          int32_t i = 0;
          for (int32_t u_filt = -filt_rad; u_filt <= filt_rad; u_filt++) {
            int32_t u_in = u + u_filt;
            if (u_in >= 0 && u_in < width_) {
              norm_filt_tmp_[v * width_ + u] += kernel1d_[i] * 
                input[outf * im_dim + v * width_ + u_in];
            }
            i++;
          }
        }
      }
      // The filter is seperable --> Filter VERTICALLY second
      for (int32_t v = 0; v < height_; v++) {
        for (int32_t u = 0; u < width_; u++) {
          output[v * width_ + u] = 0.0f;
          int32_t i = 0;
          for (int32_t v_filt = -filt_rad; v_filt <= filt_rad; v_filt++) {
            int32_t v_in = v + v_filt;
            if (v_in >= 0 && v_in < height_) {
              output[v * width_ + u] += kernel1d_[i] * 
                norm_filt_tmp_[v_in * width_ + u];
            }
            i++;
          }

          // Add the filtered pixel to the output accumulator
          norm_accum_[v * width_ + u] += output[v * width_ + u];
        }
      }
    }

    // The accumulator now needs to be normalized
    for (int32_t i = 0; i < im_dim; i++) {
      norm_accum_[i] /= (n_feats_ * n_feats_);
      norm_accum_[i] /= norm_coef_[i];
    }

    // At this point norm_accum should be the same as <substage>.adjustedsums
    // in torch
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

  void SpatialSubtractiveNormalization::normalizeFeature(const int32_t outf) {
    const int32_t im_dim = width_ * height_;
    for (int32_t v = 0; v < height_; v++) {
      for (int32_t u = 0; u < width_; u++) {
        output[outf * im_dim + v * width_ + u] = 
          cur_input_[outf * im_dim + v * width_ + u] - norm_accum_[v * width_ + u];
      }
    }
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  TorchStage* SpatialSubtractiveNormalization::loadFromFile(std::ifstream& file) {
    throw std::wruntime_error("Not yet implemented");
  }

}  // namespace hand_net
}  // namespace kinect_interface
