#include "kinect_interface/hand_net/spatial_convolution_map.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread_pool.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::threading;

namespace kinect_interface {
namespace hand_net {

  SpatialConvolutionMap::SpatialConvolutionMap(const int32_t feats_in, 
    const int32_t feats_out, const int32_t fan_in, const int32_t filt_height, 
    const int32_t filt_width, const int32_t height, const int32_t width) 
    : TorchStage() {
    filt_width_ = filt_width;
    filt_height_ = filt_height;
    feats_in_ = feats_in;
    feats_out_ = feats_out;
    fan_in_ = fan_in;
    width_ = width;
    height_ = height;

    output = new float[feats_out_ * outWidth() * outHeight()];

    weights_ = new float*[feats_out_ * fan_in_];
    for (int32_t i = 0; i < feats_out_ * fan_in_; i++) {
      weights_[i] = new float[filt_width_ * filt_height_];
    }
    conn_table_ = new int16_t*[feats_out_];
    for (int32_t i = 0; i < feats_out_; i++) {
      conn_table_[i] = new int16_t[fan_in_ * 2];
    }
    biases_ = new float[feats_out_];

    thread_cbs_ = new Callback<void>*[feats_out_];
    for (int32_t i = 0; i < feats_out_; i++) {
      thread_cbs_[i] = 
        MakeCallableMany(&SpatialConvolutionMap::forwardPropThread, this, i);
    }
  }

  SpatialConvolutionMap::~SpatialConvolutionMap() {
    SAFE_DELETE_ARR(output);
    for (int32_t i = 0; i < feats_out_; i++) {
      SAFE_DELETE(thread_cbs_[i]);
    }
    SAFE_DELETE_ARR(thread_cbs_);
    for (int32_t i = 0; i < feats_out_ * fan_in_; i++) {
      SAFE_DELETE_ARR(weights_[i]);
    }
    SAFE_DELETE(weights_);
    for (int32_t i = 0; i < feats_out_; i++) {
      SAFE_DELETE_ARR(conn_table_[i]);
    }
    SAFE_DELETE(conn_table_);
    SAFE_DELETE_ARR(biases_);
  }

  void SpatialConvolutionMap::forwardProp(float* input, 
    jtil::threading::ThreadPool* tp) { 
    cur_input_ = input;
    threads_finished_ = 0;
    for (int32_t i = 0; i < feats_out_; i++) {
      tp->addTask(thread_cbs_[i]);
    } 

    // Wait for all threads to finish
    std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
    while (threads_finished_ != feats_out_) {
      not_finished_.wait(ul);
    }
    ul.unlock();  // Release lock
  }

  void SpatialConvolutionMap::forwardPropThread(const int32_t outf) {
    const int32_t out_w = outWidth();
    const int32_t out_h = outHeight();
    const int32_t out_dim = out_w * out_h;
    const int32_t in_dim = width_ * height_;

    // Initialize the output array to the convolution bias:
    // http://www.torch.ch/manual/nn/index#spatialconvolution
    // Set the output layer to the current bias
    for (int32_t uv = outf * out_dim; uv < ((outf+1) * out_dim); uv++) {
      output[uv] = biases_[outf];
    }

    // Now iterate through the connection table:
    for (int32_t inf = 0; inf < fan_in_; inf++) {
      int32_t inf_index = (int32_t)conn_table_[outf][inf * 2];
      int32_t weight_index = (int32_t)conn_table_[outf][inf * 2 + 1];
      float* cur_filt = weights_[weight_index];

      // for each output pixel, perform the convolution over the input
      for (int32_t outv = 0; outv < out_h; outv++) {
        for (int32_t outu = 0; outu < out_w; outu++) {
          // Now perform the convolution of the inputs
          for (int32_t filtv = 0; filtv < filt_height_; filtv++) {
            for (int32_t filtu = 0; filtu < filt_width_; filtu++) {
              int32_t inu = outu + filtu;
              int32_t inv = outv + filtv;
              output[outf * out_dim + outv * out_w + outu] +=
                (cur_filt[filtv * filt_width_ + filtu] *
                cur_input_[inf_index * in_dim + inv * width_ + inu]);
            }
          }
        }
      }
      // Convolution finished for this input feature
    }
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  TorchStage* SpatialConvolutionMap::loadFromFile(std::ifstream& file) {
    throw std::wruntime_error("Not yet implemented");
  }

  int32_t SpatialConvolutionMap::outWidth() const {
    return (width_ - filt_width_ + 1);
  }

  int32_t SpatialConvolutionMap::outHeight() const {
    return (height_ - filt_height_ + 1);
  }

}  // namespace hand_net
}  // namespace kinect_interface
