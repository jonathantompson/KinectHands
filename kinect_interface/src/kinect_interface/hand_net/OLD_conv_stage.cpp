#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <iomanip>
#include <fstream>
#include "kinect_interface/hand_net/OLD_conv_stage.h"
#include "kinect_interface/hand_net/hand_net.h"  // for print3DTensorToStdCout
#include "kinect_interface/hand_net/torch_stage.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread_pool.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

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

  // In torch: >> normkernel = image.gaussian1D(7)
  const int32_t ConvStage::norm_kernel_size_ = 7;
  const float ConvStage::norm_1dkernel_[7] = {
    // Unnormalized values
    //0.230066298993809f,
    //0.520450121020702f,
    //0.849365816568312f,
    //1.000000000000000f,
    //0.849365816568312f,
    //0.520450121020702f,
    //0.230066298993809f
    // Normalized values (divided by the sum)
    0.054780762222219f,
    0.123923644848684f,
    0.202241297576406f,
    0.238108590705381f,
    0.202241297576406f,
    0.123923644848684f,
    0.054780762222219f
  };

  ConvStage::ConvStage() {
    filt_width_ = 0;
    filt_height_ = 0;
    n_input_features_ = 0;
    n_output_features_ = 0;
    filt_fan_in_ = 0;
    norm_type_ = UndefinedNorm;
    pool_type_ = UndefinedPool;
    pool_size_ = 0;
    nonlin_type_ = UndefinedNonlin;
    weights_ = NULL;
    conn_table_ = NULL;
    biases_ = NULL;
    norm_coef_w = 0;
    norm_coef_h = 0;
    norm_coef_ = NULL;
    norm_accum_ = NULL;
    norm_filt_tmp_ = NULL;
    conv_thread_cbs_ = NULL;
    nonlin_thread_cbs_ = NULL;
  }

  ConvStage::~ConvStage() {
    if (weights_) {
      for (int32_t i = 0; i < n_output_features_ * filt_fan_in_; i++) {
        SAFE_DELETE(weights_[i]);
      }
    }
    SAFE_DELETE(weights_);
    if (conn_table_) {
      for (int32_t i = 0; i < n_output_features_; i++) {
        SAFE_DELETE(conn_table_[i]);
      }
    }
    SAFE_DELETE(conn_table_);
    SAFE_DELETE(biases_);
    SAFE_DELETE(norm_coef_);
    if (conv_thread_cbs_) {
      for (int32_t i = 0; i < n_output_features_; i++) {
        SAFE_DELETE(conv_thread_cbs_[i]);
      }
    }
    SAFE_DELETE_ARR(conv_thread_cbs_);
    if (nonlin_thread_cbs_) {
      for (int32_t i = 0; i < n_output_features_; i++) {
        SAFE_DELETE(nonlin_thread_cbs_[i]);
      }
    }
    SAFE_DELETE_ARR(nonlin_thread_cbs_);
  }

  void ConvStage::loadFromFile(std::ifstream& file) {
    file.read((char*)(&filt_width_), sizeof(filt_width_));
    file.read((char*)(&filt_height_), sizeof(filt_height_));
    file.read((char*)(&n_input_features_), sizeof(n_input_features_));
    file.read((char*)(&n_output_features_), sizeof(n_output_features_));
    file.read((char*)(&filt_fan_in_), sizeof(filt_fan_in_));

    int32_t filt_dim = filt_width_ * filt_height_;

    weights_ = new float*[n_output_features_ * filt_fan_in_];
    for (int32_t i = 0; i < n_output_features_ * filt_fan_in_; i++) {
      weights_[i] = new float[filt_dim];
      file.read((char*)(weights_[i]), sizeof(weights_[i][0]) * filt_dim);
    }

    conn_table_ = new int16_t*[n_output_features_];
    for (int32_t i = 0; i < n_output_features_; i++) {
      conn_table_[i] = new int16_t[filt_fan_in_ * 2];
      file.read((char*)(conn_table_[i]), 
        sizeof(conn_table_[i][0]) * filt_fan_in_ * 2);
    }

    int32_t norm_type;
    file.read((char*)(&norm_type), sizeof(norm_type));
    norm_type_ = (NormType)norm_type;

    int32_t pool_type;
    file.read((char*)(&pool_type), sizeof(pool_type));
    pool_type_ = (PoolType)pool_type;
    file.read((char*)(&pool_size_), sizeof(pool_size_));

    int32_t nonlin_type;
    file.read((char*)(&nonlin_type), sizeof(nonlin_type));
    nonlin_type_ = (NonlinType)nonlin_type;

    biases_ = new float[n_output_features_];
    file.read((char*)(biases_), sizeof(biases_[0]) * n_output_features_);

    // Make thread callbacks for multithreaded execution
    conv_thread_cbs_ = new Callback<void>*[n_output_features_];
    nonlin_thread_cbs_ = new Callback<void>*[n_output_features_];
    for (int32_t i = 0; i < n_output_features_; i++) {
      conv_thread_cbs_[i] = 
        MakeCallableMany(&ConvStage::performSpacialConvolutionFeat, this, i);
      nonlin_thread_cbs_[i] = 
        MakeCallableMany(&ConvStage::performNonlinearityFeat, this, i);
    }
  }

  void ConvStage::forwardProp(float*& in, const int32_t inw, const int32_t inh, 
    float*& out, jtil::threading::ThreadPool* tp) {
    const int32_t interm_w = calcIntermWidth(inw);
    const int32_t interm_h = calcIntermHeight(inh);
    const int32_t out_w = calcOutWidth(inw);
    const int32_t out_h = calcOutHeight(inh);

    performSpacialConvolution(in, inw, inh, out, tp);  // Checked against torch

    performNonlinearity(out, interm_w, interm_h, tp);  // Checked against torch

    // Buffer swap: in <--> out
    float* tmp = in;
    in = out;
    out = tmp;

    if (pool_size_ > 1) {
      performPooling(in, interm_w, interm_h, out);  // Checked against torch

      // Buffer swap: in <--> out
      tmp = in;
      in = out;
      out = tmp;
    }

    initNormCoef(out_w, out_h);  // Only does work once on startup
    performNormalization(in, out_w, out_h, out);  // Checked against torch

    // Result is now in out.
  }

  void ConvStage::performSpacialConvolution(float*&in, const int32_t inw, 
    const int32_t inh, float*& out, jtil::threading::ThreadPool* tp) {
    cur_in_ = in;
    cur_inw_ = inw;
    cur_inh_ = inh;
    cur_out_ = out;

    // Now accumulate the connections by the correct weights into the output
    threads_finished_ = 0;
    for (int32_t outf = 0; outf < n_output_features_; outf++) {
      tp->addTask(conv_thread_cbs_[outf]);
    } 

    // Wait for all threads to finish
    std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
    while (threads_finished_ != n_output_features_) {
      not_finished_.wait(ul);
    }
    ul.unlock();  // Release lock
  }

  void ConvStage::performSpacialConvolutionFeat(const int32_t outf) {
    const int32_t interm_w = calcIntermWidth(cur_inw_);
    const int32_t interm_h = calcIntermHeight(cur_inh_);
    const int32_t interm_dim = interm_w * interm_h;
    const int32_t in_dim = cur_inw_ * cur_inh_;

    // Initialize the output array to the convolution bias:
    // http://www.torch.ch/manual/nn/index#spatialconvolution
    // Set the output layer to the current bias
    for (int32_t uv = outf * interm_dim; uv < ((outf+1) * interm_dim); uv++) {
      cur_out_[uv] = biases_[outf];
    }

    // Now iterate through the connection table:
    for (int32_t inf = 0; inf < filt_fan_in_; inf++) {
      int32_t inf_index = (int32_t)conn_table_[outf][inf * 2];
      int32_t weight_index = (int32_t)conn_table_[outf][inf * 2 + 1];
      float* cur_filt = weights_[weight_index];

      // for each output pixel, perform the convolution over the input
      for (int32_t outv = 0; outv < interm_h; outv++) {
        for (int32_t outu = 0; outu < interm_w; outu++) {
          // Now perform the convolution of the inputs
          for (int32_t filtv = 0; filtv < filt_height_; filtv++) {
            for (int32_t filtu = 0; filtu < filt_width_; filtu++) {
              int32_t inu = outu + filtu;
              int32_t inv = outv + filtv;
              cur_out_[outf * interm_dim + outv * interm_w + outu] +=
                (cur_filt[filtv * filt_width_ + filtu] *
                cur_in_[inf_index * in_dim + inv * cur_inw_ + inu]);
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

  void ConvStage::performNonlinearity(float*&data, const int32_t w, 
    const int32_t h, ThreadPool* tp) {
    cur_inw_ = w;
    cur_inh_ = h;
    cur_out_ = data;
    // Now accumulate the connections by the correct weights into the output
    threads_finished_ = 0;
    for (int32_t outf = 0; outf < n_output_features_; outf++) {
      tp->addTask(nonlin_thread_cbs_[outf]);
    } 

    // Wait for all threads to finish
    std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
    while (threads_finished_ != n_output_features_) {
      not_finished_.wait(ul);
    }
    ul.unlock();  // Release lock
  }

  void ConvStage::performNonlinearityFeat(const int32_t outf) {
    const int32_t dim = cur_inw_ * cur_inh_;
    switch (nonlin_type_) {
    case TanhNonlin:
      for (int32_t outv = 0; outv < cur_inw_; outv++) {
        for (int32_t outu = 0; outu < cur_inh_; outu++) {
          cur_out_[outf * dim + outv * cur_inw_ + outu] = 
            tanh(cur_out_[outf * dim + outv * cur_inw_ + outu]);
        }
      }
      break;
    default:
      throw std::wruntime_error("ConvStage::performNonlinearity() - ERROR: "
        "Only TanhNonlin supported for now.");
    }
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  void ConvStage::performPooling(float*&in, const int32_t inw, 
    const int32_t inh, float*& out) const {
    const int32_t out_w = inw / pool_size_;
    const int32_t out_h = inh / pool_size_;
    const int32_t out_dim = out_w * out_h;
    const int32_t in_dim = inw * inh;
    switch (pool_type_) {
    case L2Pool:
      for (int32_t outf = 0; outf < n_output_features_; outf++) {
        for (int32_t outv = 0; outv < out_h; outv++) {
          for (int32_t outu = 0; outu < out_w; outu++) {
            out[outf * out_dim + outv * out_w + outu] = 0.0f;
            // Now perform L2 pooling:
            for (int32_t inv = outv * pool_size_; 
              inv < (outv + 1) * pool_size_; inv++) {
              for (int32_t inu = outu * pool_size_; 
                inu < (outu + 1) * pool_size_; inu++) {
                float val = in[outf * in_dim + inv * inw + inu];
                out[outf * out_dim + outv * out_w + outu] += (val * val);
              }
            }
            out[outf * out_dim + outv * out_w + outu] = 
              sqrtf(out[outf * out_dim + outv * out_w + outu]);
          }
        }
      }
      break;
    default:
      throw std::wruntime_error("ConvStage::performPooling() - ERROR: "
        "Only L2Pool supported for now.");
    }
  }

  void ConvStage::initNormCoef(const int32_t inw, const int32_t inh) {
    // Only reinit if the input image size has changed
    if (norm_coef_w != inw || norm_coef_h != inh) {
      if (norm_kernel_size_ % 2 != 1) {
        throw std::wruntime_error("ConvStage::initNormCoef() - ERROR: "
          "norm_kernel_size_ must be an odd number!");
      }
      SAFE_DELETE(norm_coef_);
      SAFE_DELETE(norm_accum_);
      SAFE_DELETE(norm_filt_tmp_);
      norm_coef_ = new float[inw * inh];
      norm_accum_ = new float[inw * inh];
      norm_filt_tmp_ = new float[inw * inh];
      norm_coef_w = inw;
      norm_coef_h = inh;

      int32_t filt_rad = (norm_kernel_size_ - 1) / 2;

      // Filter an image of all 1 values to create the normalization constants
      // See norm_test.lua for proof that this works as well as:
      // https://github.com/andresy/torch/blob/master/extra/nn/SpatialSubtractiveNormalization.lua
      // The filter is seperable, but we'll just do the dumb 2D version since
      // we only do this once on startup.  --> O(n * m), rather than O(n * 2)
      for (int32_t v = 0; v < inh; v++) {
        for (int32_t u = 0; u < inw; u++) {
          norm_coef_[v * inw + u] = 0.0f;
          for (int32_t v_filt = -filt_rad; v_filt <= filt_rad; v_filt++) {
            for (int32_t u_filt = -filt_rad; u_filt <= filt_rad; u_filt++) {
              int32_t u_in = u + u_filt;
              int32_t v_in = v + v_filt;
              if (u_in >= 0 && u_in < inw && v_in >= 0 && v_in < inh) {
                // Pixel is inside --> We'll effectively clamp zeros elsewhere.
                norm_coef_[v * inw + u] += (norm_1dkernel_[v_filt + filt_rad] *
                  norm_1dkernel_[u_filt + filt_rad]);
              }
            }
          }
          norm_coef_[v * inw + u] /= n_output_features_;
        }
      }
    }
  }

  void ConvStage::performNormalization(float*&in, const int32_t inw, 
    const int32_t inh, float*& out) const {
    const int32_t im_dim = inw * inh;
    int32_t filt_rad = (norm_kernel_size_ - 1) / 2;

    switch (norm_type_) {
    case SubtractiveNorm:
      // Zero out the accumulator
      for (int32_t i = 0; i < im_dim; i++) {
        norm_accum_[i] = 0.0f;
      }
      for (int32_t outf = 0; outf < n_output_features_; outf++) {
        // The filter is seperable --> Filter HORIZONTALLY first
        for (int32_t v = 0; v < inh; v++) {
          for (int32_t u = 0; u < inw; u++) {
            norm_filt_tmp_[v * inw + u] = 0.0f;
            int32_t i = 0;
            for (int32_t u_filt = -filt_rad; u_filt <= filt_rad; u_filt++) {
              int32_t u_in = u + u_filt;
              if (u_in >= 0 && u_in < inw) {
                norm_filt_tmp_[v * inw + u] += norm_1dkernel_[i] * 
                  in[outf * im_dim + v * inw + u_in];
              }
              i++;
            }
          }
        }
        // The filter is seperable --> Filter VERTICALLY second
        for (int32_t v = 0; v < inh; v++) {
          for (int32_t u = 0; u < inw; u++) {
            out[v * inw + u] = 0.0f;
            int32_t i = 0;
            for (int32_t v_filt = -filt_rad; v_filt <= filt_rad; v_filt++) {
              int32_t v_in = v + v_filt;
              if (v_in >= 0 && v_in < inh) {
                out[v * inw + u] += norm_1dkernel_[i] * 
                  norm_filt_tmp_[v_in * inw + u];
              }
              i++;
            }

            // Add the filtered pixel to the output accumulator
            norm_accum_[v * inw + u] += out[v * inw + u];
          }
        }
      }

      // The accumulator now needs to be normalized
      for (int32_t i = 0; i < im_dim; i++) {
        norm_accum_[i] /= (n_output_features_ * n_output_features_);
        norm_accum_[i] /= norm_coef_[i];
      }

      // At this point norm_accum should be the same as <substage>.adjustedsums
      // in torch

      // Finally perform the 
      for (int32_t outf = 0; outf < n_output_features_; outf++) {
        // The filter is seperable --> Filter HORIZONTALLY first
        for (int32_t v = 0; v < inh; v++) {
          for (int32_t u = 0; u < inw; u++) {
            out[outf * im_dim + v * inw + u] = in[outf * im_dim + v * inw + u]-
              norm_accum_[v * inw + u];
          }
        }
      }
      break;
    default:
      throw std::wruntime_error("ConvStage::performNormalization() - ERROR: "
        "Only SubtractiveNorm supported for now.");
    }
  }

  void ConvStage::printToStdOut() const {
    std::cout << "  filt_width_ = " << filt_width_ << std::endl;
    std::cout << "  filt_height_ = " << filt_height_ << std::endl;
    std::cout << "  n_input_features_ = " << n_input_features_ << std::endl;
    std::cout << "  n_output_features_ = " << n_output_features_ << std::endl;
    std::cout << "  filt_fan_in_ = " << filt_fan_in_ << std::endl;
    std::cout << "  norm_type_ = " << (int32_t)norm_type_ << std::endl;
    std::cout << "  pool_type_ = " << (int32_t)pool_type_ << std::endl;
    std::cout << "  pool_size_ = " << pool_size_ << std::endl;
    std::cout << "  nonlin_type_ = " << (int32_t)nonlin_type_ << std::endl;

    std::cout << std::setprecision(6);
    std::cout << std::fixed;

    std::cout << "  weights_:" << std::endl;
    TorchStage::print3DTensorToStdCout<float>(weights_, 
      std::min<int32_t>(n_output_features_ * filt_fan_in_, MAX_PRINT_LENGTH), 
      filt_height_, filt_width_);

    std::cout << "  conn_:" << std::endl;
    TorchStage::print3DTensorToStdCout<int16_t>(conn_table_,  
      std::min<int32_t>(n_output_features_, MAX_PRINT_LENGTH), filt_fan_in_, 
      2);

    std::cout << "  biases_[] =" << std::endl;
    std::cout << "      (0) ";
    std::cout.setf(std::ios::showpos);
    for (int32_t i = 0; i < std::min<int32_t>(n_output_features_, 
      MAX_PRINT_LENGTH); i++) {
      std::cout << biases_[i];
      if (i != n_output_features_ - 1) {
        std::cout << ", ";
      } else {
        std::cout << std::endl;
      }
    }

    std::cout << std::endl;
    std::cout << std::resetiosflags(std::ios_base::showpos);
  }

  const int32_t ConvStage::dataSizeReq(const int32_t inw, const int32_t inh) 
    const {
    // input size requirement
    int32_t in_req = inw * inh * n_input_features_;
    // calculate intermediate image size
    int32_t out_req = calcIntermWidth(inw) * calcIntermHeight(inh) * 
      n_output_features_;
    // The downsampled image is only less than this, so we don't need to worry
    // about it.
    return std::max<int32_t>(in_req, out_req);
  }

  const int32_t ConvStage::calcOutWidth(const int32_t inw) const {
    return (inw - filt_width_ + 1) / pool_size_;
  }

  const int32_t ConvStage::calcOutHeight(const int32_t inh) const {
    return (inh - filt_height_ + 1) / pool_size_;
  }

  const int32_t ConvStage::calcIntermWidth(const int32_t inw) const {
    return (inw - filt_width_ + 1);
  }

  const int32_t ConvStage::calcIntermHeight(const int32_t inh) const {
    return (inh - filt_height_ + 1);
  }

}  // namespace hand_net
}  // namespace kinect_interface
