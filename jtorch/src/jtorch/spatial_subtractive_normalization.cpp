#include "jtorch/spatial_subtractive_normalization.h"
#include "jtorch/float_tensor.h"
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

namespace jtorch {

  // kernel1d default is either TorchStage::gaussian1D<float>(n) or just a
  // vector of 1 values.
  SpatialSubtractiveNormalization::SpatialSubtractiveNormalization(
    const FloatTensor& kernel1d) : TorchStage() {
    if (kernel1d.dataSize() % 2 == 0 || kernel1d.dim()[1] != 1 ||
      kernel1d.dim()[2] != 1 || kernel1d.dim()[3] != 1) {
      throw std::wruntime_error("SpatialSubtractiveNormalization() - ERROR: "
        "Averaging kernel must be 1D and have odd size!");
    }

    kernel1d_ = kernel1d.copy();
    // Now normalize the kernel!
    float sum = 0.0f;
    for (int32_t i = 0; i < kernel1d_->dim()[0]; i++) {
      sum += kernel1d_->data()[i];
    }
    for (int32_t i = 0; i < kernel1d_->dim()[0]; i++) {
      kernel1d_->data()[i] /= sum;
    }

    output = NULL;
    mean_coef_ = NULL;
    mean_accum_ = NULL;
    filt_tmp_ = NULL;
    thread_cbs_ = NULL;
  }

  void SpatialSubtractiveNormalization::init(TorchData& input, 
    jtil::threading::ThreadPool& tp)  {
    if (input.type() != TorchDataType::FLOAT_TENSOR_DATA) {
      throw std::wruntime_error("SpatialSubtractiveNormalization::init() - "
        "FloatTensor expected!");
    }
    FloatTensor& in = (FloatTensor&)input;
    if (output != NULL) {
      if (!Int4::equal(in.dim(), ((FloatTensor*)output)->dim())) {
        // Input dimension has changed!
        SAFE_DELETE(output);
        SAFE_DELETE_ARR(mean_coef_);
        SAFE_DELETE_ARR(mean_accum_);
        SAFE_DELETE_ARR(filt_tmp_);
        SAFE_DELETE(thread_cbs_);
      }
    }
    if (output == NULL) {
      output = new FloatTensor(in.dim());
    }
    if (mean_coef_ == NULL) {
      mean_coef_ = new float[((FloatTensor*)output)->dim()[0] * 
        ((FloatTensor*)output)->dim()[1]];
      // Filter an image of all 1 values to create the normalization constants
      // See norm_test.lua for proof that this works as well as:
      // https://github.com/andresy/torch/blob/master/extra/nn/SpatialSubtractiveNormalization.lua
      // The filter is seperable, but we'll just do the dumb 2D version since
      // we only do this once on startup.  --> O(n * m)
      int32_t kernel1d_size = kernel1d_->dim()[0];
      int32_t filt_rad = (kernel1d_size - 1) / 2;
      int32_t n_feats = ((FloatTensor*)output)->dim()[2];
      int32_t height = ((FloatTensor*)output)->dim()[1];
      int32_t width = ((FloatTensor*)output)->dim()[0];
      for (int32_t v = 0; v < height; v++) {
        for (int32_t u = 0; u < width; u++) {
          mean_coef_[v * width + u] = 0.0f;
          for (int32_t v_filt = -filt_rad; v_filt <= filt_rad; v_filt++) {
            for (int32_t u_filt = -filt_rad; u_filt <= filt_rad; u_filt++) {
              int32_t u_in = u + u_filt;
              int32_t v_in = v + v_filt;
              if (u_in >= 0 && u_in < width && v_in >= 0 && v_in < height) {
                // Pixel is inside --> We'll effectively clamp zeros elsewhere.
                mean_coef_[v * width + u] += 
                  (kernel1d_->data()[v_filt + filt_rad] *
                   kernel1d_->data()[u_filt + filt_rad]);
              }
            }
          }
          mean_coef_[v * width + u] /= n_feats;
        }
      }
    }
    if (mean_accum_ == NULL) {
      mean_accum_ = new float[((FloatTensor*)output)->dim()[0] * 
        ((FloatTensor*)output)->dim()[1]];
    }
    if (filt_tmp_ == NULL) {
      filt_tmp_ = new float[((FloatTensor*)output)->dim()[0] * 
        ((FloatTensor*)output)->dim()[1]];
    }
    if (thread_cbs_ == NULL) {
      int32_t n_feats = in.dim()[2];
      int32_t n_threads = n_feats;
      thread_cbs_ = new VectorManaged<Callback<void>*>(n_threads);
      for (int32_t dim2 = 0; dim2 < (int32_t)in.dim()[2]; dim2++) {
        thread_cbs_->pushBack(MakeCallableMany(
          &SpatialSubtractiveNormalization::normalizeFeature, 
          this, dim2));
      }
    }
  }

  SpatialSubtractiveNormalization::~SpatialSubtractiveNormalization() {
    SAFE_DELETE(output);
    SAFE_DELETE(kernel1d_);
    SAFE_DELETE_ARR(mean_coef_);
    SAFE_DELETE_ARR(mean_accum_);
    SAFE_DELETE_ARR(filt_tmp_);
    SAFE_DELETE(thread_cbs_);
  }

  void SpatialSubtractiveNormalization::forwardProp(TorchData& input, 
    ThreadPool& tp) { 
    init(input, tp);
    FloatTensor& in = (FloatTensor&)input;
    const int32_t width = in.dim()[0];
    const int32_t height = in.dim()[1];
    const int32_t n_feats = in.dim()[2];
    const int32_t n_banks = in.dim()[3];
    const int32_t im_dim = in.dim()[0] * in.dim()[1];
    const float* kernel = kernel1d_->data();
    int32_t filt_rad = (kernel1d_->dim()[0] - 1) / 2;

    for (int32_t outb = 0; outb < n_banks; outb++) {
      // Zero out the accumulator
      for (int32_t i = 0; i < im_dim; i++) {
        mean_accum_[i] = 0.0f;
      }
      float* cur_in = &in(0, 0, 0, outb);
      float* cur_out = &((*((FloatTensor*)output))(0, 0, 0, outb));
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
                  cur_in[outf * im_dim + v * width + u_in];
              }
              i++;
            }
          }
        }
        // The filter is seperable --> Filter VERTICALLY second
        for (int32_t v = 0; v < height; v++) {
          for (int32_t u = 0; u < width; u++) {
            cur_out[v * width + u] = 0.0f;  // Use output as temp storage
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
            mean_accum_[v * width + u] += cur_out[v * width + u];
          }
        }
      }

      // The accumulator now needs to be normalized
      for (int32_t i = 0; i < im_dim; i++) {
        mean_accum_[i] /= (n_feats * n_feats);
        mean_accum_[i] /= mean_coef_[i];
      }

      // At this point norm_accum should be the same as <substage>.adjustedsums
      // in torch
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

  void SpatialSubtractiveNormalization::normalizeFeature(const int32_t outf) {
    const int32_t width = (int32_t)((FloatTensor*)output)->dim()[0];
    const int32_t height = (int32_t)((FloatTensor*)output)->dim()[1];
    const int32_t im_dim = width * height;
    for (int32_t v = 0; v < height; v++) {
      for (int32_t u = 0; u < width; u++) {
        cur_output_[outf * im_dim + v * width + u] = 
          cur_input_[outf * im_dim + v * width + u] - mean_accum_[v * width + u];
      }
    }
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  TorchStage* SpatialSubtractiveNormalization::loadFromFile(std::ifstream& file) {
    int32_t kernel_size;
    file.read((char*)(&kernel_size), sizeof(kernel_size));
    FloatTensor* kernel = new FloatTensor(kernel_size);
    file.read((char*)(kernel->data()), kernel_size * sizeof(*kernel->data()));
    TorchStage* ret = new SpatialSubtractiveNormalization(*kernel);
    delete kernel;
    return ret;
  }

}  // namespace jtorch