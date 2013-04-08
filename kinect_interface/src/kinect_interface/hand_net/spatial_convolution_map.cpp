#include "kinect_interface/hand_net/spatial_convolution_map.h"
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

  SpatialConvolutionMap::SpatialConvolutionMap(const int32_t feats_in, 
    const int32_t feats_out, const int32_t fan_in, const int32_t filt_height, 
    const int32_t filt_width) 
    : TorchStage() {
    filt_width_ = filt_width;
    filt_height_ = filt_height;
    feats_in_ = feats_in;
    feats_out_ = feats_out;
    fan_in_ = fan_in;

    output = NULL;
    thread_cbs_ = NULL;

    weights_ = new float*[feats_out_ * fan_in_];
    for (int32_t i = 0; i < feats_out_ * fan_in_; i++) {
      weights_[i] = new float[filt_width_ * filt_height_];
    }
    conn_table_ = new int16_t*[feats_out_];
    for (int32_t i = 0; i < feats_out_; i++) {
      conn_table_[i] = new int16_t[fan_in_ * 2];
    }
    biases_ = new float[feats_out_];
  }

  SpatialConvolutionMap::~SpatialConvolutionMap() {
    SAFE_DELETE(output);
    SAFE_DELETE(thread_cbs_);
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

  void SpatialConvolutionMap::init(FloatTensor& input, 
    jtil::threading::ThreadPool& tp)  {
    if (input.dim()[2] != feats_in_) {
      throw std::wruntime_error("SpatialConvolutionMap::init() - ERROR: "
        "incorrect number of input features!");
    }
    if (output != NULL) {
      Int4 out_dim(input.dim());
      out_dim[0] -= out_dim[0] - filt_width_ + 1;
      out_dim[1] -= out_dim[1] - filt_height_ + 1;
      out_dim[2] = feats_out_;
      if (!Int4::equal(out_dim, output->dim())) {
        // Input dimension has changed!
        SAFE_DELETE(output);
        SAFE_DELETE(thread_cbs_);
      }
    }
    if (output == NULL) {
      Int4 out_dim(input.dim());
      out_dim[0] -= out_dim[0] - filt_width_ + 1;
      out_dim[1] -= out_dim[1] - filt_height_ + 1;
      out_dim[2] = feats_out_;
      output = new FloatTensor(out_dim);
    }
    if (thread_cbs_ == NULL) {
      int32_t n_feats = input.dim()[2];
      int32_t n_threads = n_feats;
      thread_cbs_ = new VectorManaged<Callback<void>*>(n_threads);
      for (int32_t dim2 = 0; dim2 < (int32_t)input.dim()[2]; dim2++) {
        thread_cbs_->pushBack(MakeCallableMany(
          &SpatialConvolutionMap::forwardPropThread, 
          this, dim2));
      }
    }
  }

  void SpatialConvolutionMap::forwardProp(FloatTensor& input, 
    ThreadPool& tp) { 
    init(input, tp);
    const int32_t n_banks = input.dim()[3];
    for (int32_t outb = 0; outb < n_banks; outb++) {
      float* cur_input_ = &input(0, 0, 0, outb);
      float* cur_output_ = &((*output)(0, 0, 0, outb));
      cur_input_width_ = input.dim()[0];
      cur_input_height_ = input.dim()[1];

      threads_finished_ = 0;
      for (int32_t i = 0; i < feats_out_; i++) {
        tp.addTask((*thread_cbs_)[i]);
      } 

      // Wait for all threads to finish
      std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
      while (threads_finished_ != feats_out_) {
        not_finished_.wait(ul);
      }
      ul.unlock();  // Release lock
    }
  }

  void SpatialConvolutionMap::forwardPropThread(const int32_t outf) {
    const int32_t out_w = output->dim()[0];
    const int32_t out_h = output->dim()[1];
    const int32_t out_dim = out_w * out_h;
    const int32_t in_dim = cur_input_width_ * cur_input_height_;

    // Initialize the output array to the convolution bias:
    // http://www.torch.ch/manual/nn/index#spatialconvolution
    // Set the output layer to the current bias
    for (int32_t uv = outf * out_dim; uv < ((outf+1) * out_dim); uv++) {
      cur_output_[uv] = biases_[outf];
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
              cur_output_[outf * out_dim + outv * out_w + outu] +=
                (cur_filt[filtv * filt_width_ + filtu] *
                cur_input_[inf_index * in_dim + inv * cur_input_width_ + inu]);
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

}  // namespace hand_net
}  // namespace kinect_interface
