#include "kinect_interface/hand_net/spatial_divisive_normalization.h"
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

  // kernel1d default is either TorchStage::gaussian1D<float>(n) or just a
  // vector of 1 values.
  SpatialDivisiveNormalization::SpatialDivisiveNormalization(
    const FloatTensor& kernel1d, const float threshold) : TorchStage() {
    if (kernel1d.dataSize() % 2 == 0 || kernel1d.dim()[1] != 1 ||
      kernel1d.dim()[2] != 1 || kernel1d.dim()[3] != 1) {
      throw std::wruntime_error("SpatialDivisiveNormalization() - ERROR: "
        "Averaging kernel must be 1D and have odd size!");
    }

    kernel1d_ = kernel1d.copy();
    kernel1d_norm_ = NULL;

    output = NULL;
    std_coef_ = NULL;
    std_accum_ = NULL;
    filt_tmp_ = NULL;

    threshold_ = threshold;
  }

  SpatialDivisiveNormalization::~SpatialDivisiveNormalization() {
    SAFE_DELETE(output);
    SAFE_DELETE(kernel1d_);
    SAFE_DELETE(kernel1d_norm_);
    SAFE_DELETE_ARR(std_coef_);
    SAFE_DELETE_ARR(std_accum_);
    SAFE_DELETE_ARR(filt_tmp_);
    SAFE_DELETE(thread_cbs_);
  }

  void SpatialDivisiveNormalization::init(FloatTensor& input, 
    jtil::threading::ThreadPool& tp)  {
    if (output != NULL) {
      if (!Int4::equal(input.dim(), output->dim())) {
        // Input dimension has changed!
        SAFE_DELETE(output);
        SAFE_DELETE(kernel1d_norm_);
        SAFE_DELETE_ARR(std_coef_);
        SAFE_DELETE_ARR(std_accum_);
        SAFE_DELETE_ARR(filt_tmp_);
        SAFE_DELETE(thread_cbs_);
      }
    }
    if (output == NULL) {
      output = new FloatTensor(input.dim());
    }
    if (kernel1d_norm_ == NULL) {
      kernel1d_norm_ = kernel1d_->copy();
      // Now normalize the kernel!
      const float n_feats = (float)input.dim()[2];
      float sum = 0.0f;
      for (int32_t i = 0; i < kernel1d_norm_->dim()[0]; i++) {
        sum += kernel1d_norm_->data()[i];
      }
      for (int32_t i = 0; i < kernel1d_norm_->dim()[0]; i++) {
        kernel1d_norm_->data()[i] /= (sum * sqrtf(n_feats));
      }
    }
    if (std_coef_ == NULL) {
      std_coef_ = new float[output->dim()[0] * output->dim()[1]];
      // Filter an image of all 1 values to create the normalization constants
      // See norm_test.lua for proof that this works as well as:
      // https://github.com/andresy/torch/blob/master/extra/nn/SpatialSubtractiveNormalization.lua
      // The filter is seperable, but we'll just do the dumb 2D version since
      // we only do this once on startup.  --> O(n * m)
      int32_t kernel1d_size = kernel1d_->dim()[0];
      int32_t filt_rad = (kernel1d_size - 1) / 2;
      int32_t n_feats = output->dim()[2];
      int32_t height = output->dim()[1];
      int32_t width = output->dim()[0];
      for (int32_t v = 0; v < height; v++) {
        for (int32_t u = 0; u < width; u++) {
          std_coef_[v * width + u] = 0.0f;
          for (int32_t v_filt = -filt_rad; v_filt <= filt_rad; v_filt++) {
            for (int32_t u_filt = -filt_rad; u_filt <= filt_rad; u_filt++) {
              int32_t u_in = u + u_filt;
              int32_t v_in = v + v_filt;
              if (u_in >= 0 && u_in < width && v_in >= 0 && v_in < height) {
                // Pixel is inside --> We'll effectively clamp zeros elsewhere.
                std_coef_[v * width + u] += 
                  (kernel1d_->data()[v_filt + filt_rad] *
                   kernel1d_->data()[u_filt + filt_rad]);
              }
            }
          }
          std_coef_[v * width + u] /= n_feats;
        }
      }
    }
    if (std_accum_ == NULL) {
      std_accum_ = new float[output->dim()[0] * output->dim()[1]];
    }
    if (std_accum_ == NULL) {
      filt_tmp_ = new float[output->dim()[0] * output->dim()[1]];
    }
    if (thread_cbs_ == NULL) {
      int32_t n_feats = input.dim()[2];
      int32_t n_threads = n_feats;
      thread_cbs_ = new VectorManaged<Callback<void>*>(n_threads);
      for (int32_t dim2 = 0; dim2 < (int32_t)input.dim()[2]; dim2++) {
        thread_cbs_->pushBack(MakeCallableMany(
          &SpatialDivisiveNormalization::normalizeFeature, 
          this, dim2));
      }
    }
  }

  void SpatialDivisiveNormalization::forwardProp(FloatTensor& input, 
    ThreadPool& tp) { 
    init(input, tp);
    const int32_t width = input.dim()[0];
    const int32_t height = input.dim()[1];
    const int32_t n_feats = input.dim()[2];
    const int32_t n_banks = input.dim()[3];
    const int32_t im_dim = input.dim()[0] * input.dim()[1];
    const float* kernel = kernel1d_norm_->data();
    int32_t filt_rad = (kernel1d_norm_->dim()[0] - 1) / 2;

    for (int32_t outb = 0; outb < n_banks; outb++) {
      // Replicate the self.stdestimator in the torch module:
      for (int32_t i = 0; i < im_dim; i++) {
        std_accum_[i] = 0.0f;
      }
      float* cur_in = &input(0, 0, 0, outb);
      float* cur_out = &((*output)(0, 0, 0, outb));
      for (int32_t outf = 0; outf < n_feats; outf++) {
        // The filter is seperable --> Filter HORIZONTALLY first
        for (int32_t v = 0; v < height; v++) {
          for (int32_t u = 0; u < width; u++) {
            filt_tmp_[v * width + u] = 0.0f;
            int32_t i = 0;
            for (int32_t u_filt = -filt_rad; u_filt <= filt_rad; u_filt++) {
              int32_t u_in = u + u_filt;
              if (u_in >= 0 && u_in < width) {
                filt_tmp_[v * width + u] += kernel[i] * 
                  (cur_in[outf * im_dim + v * width + u_in] *  // Sum the squares 
                   cur_in[outf * im_dim + v * width + u_in]);
              }
              i++;
            }
          }
        }
        // The filter is seperable --> Filter VERTICALLY second
        for (int32_t v = 0; v < height; v++) {
          for (int32_t u = 0; u < width; u++) {
            cur_out[v * width + u] = 0.0f;  // Use output array as temp space
            int32_t i = 0;
            for (int32_t v_filt = -filt_rad; v_filt <= filt_rad; v_filt++) {
              int32_t v_in = v + v_filt;
              if (v_in >= 0 && v_in < height) {
                cur_out[v * width + u] += kernel[i] * 
                  filt_tmp_[v_in * width + u];
              }
              i++;
            }

            // Add the filtered pixel to the output accumulator
            std_accum_[v * width + u] += cur_out[v * width + u];
          }
        }
      }

      for (int32_t i = 0; i < im_dim; i++) {
        std_accum_[i] = sqrtf(std_accum_[i]);
      }

      // At this point std_accum should be the same as <substage>.localstds
      // in the torch module

      // The accumulator now needs to be normalized
      for (int32_t i = 0; i < im_dim; i++) {
        std_accum_[i] /= (n_feats * n_feats);
        std_accum_[i] /= std_coef_[i];
        // same as Threshold stage: Which prevents divide by zero
        std_accum_[i] = std::max<float>(std_accum_[i], threshold_);  
      }

      // At this point std_accum should be the same as <substage>.thresholdedstds
      // in the torch module
      cur_input_ = cur_in;
      cur_output_ = cur_out;
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
  }

  void SpatialDivisiveNormalization::normalizeFeature(const int32_t outf) {
    const int32_t width = (int32_t)output->dim()[0];
    const int32_t height = (int32_t)output->dim()[1];
    const int32_t im_dim = width * height;
    for (int32_t v = 0; v < height; v++) {
      for (int32_t u = 0; u < width; u++) {
        cur_output_[outf * im_dim + v * width + u] = 
          cur_input_[outf * im_dim + v * width + u] / std_accum_[v * width + u];
      }
    }
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  TorchStage* SpatialDivisiveNormalization::loadFromFile(std::ifstream& file) {
    throw std::wruntime_error("Not yet implemented");
  }

}  // namespace hand_net
}  // namespace kinect_interface
