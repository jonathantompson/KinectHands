#include "jtorch/spatial_divisive_normalization.h"
#include "jtorch/tensor.h"
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
  SpatialDivisiveNormalization::SpatialDivisiveNormalization(
    const Tensor<float>& kernel1d, const float threshold) : TorchStage() {
    if (kernel1d.dataSize() % 2 == 0 || kernel1d.dim()[1] != 1 ||
      kernel1d.dim()[2] != 1) {
      throw std::wruntime_error("SpatialDivisiveNormalization() - ERROR: "
        "Averaging kernel must be 1D and have odd size!");
    }

    kernel1d_ = kernel1d.copy();  // Normalization is input size dependant
    kernel1d_norm_ = NULL;

    output = NULL;
    std_coef_ = NULL;
    std_pass1_ = NULL;
    std_pass2_ = NULL;
    std_ = NULL;

    threshold_ = threshold;
  }

  SpatialDivisiveNormalization::~SpatialDivisiveNormalization() {
    SAFE_DELETE(output);
    SAFE_DELETE(kernel1d_);
    SAFE_DELETE(kernel1d_norm_);
    SAFE_DELETE(std_coef_);
    SAFE_DELETE(std_pass1_);
    SAFE_DELETE(std_pass2_);
    SAFE_DELETE(std_);
  }

  void SpatialDivisiveNormalization::init(TorchData& input)  {
    if (input.type() != TorchDataType::TENSOR_DATA) {
      throw std::wruntime_error("SpatialDivisiveNormalization::init() - "
        "FloatTensor expected!");
    }
    Tensor<float>& in = (Tensor<float>&)input;
    if (output != NULL) {
      if (!Int3::equal(in.dim(), ((Tensor<float>*)output)->dim())) {
        // Input dimension has changed!
        SAFE_DELETE(output);
        SAFE_DELETE(kernel1d_);
        SAFE_DELETE(kernel1d_norm_);
        SAFE_DELETE(std_coef_);
        SAFE_DELETE(std_pass1_);
        SAFE_DELETE(std_pass2_);
        SAFE_DELETE(std_);
      }
    }
    if (output == NULL) {
      output = new Tensor<float>(in.dim());
      std_pass1_ = new Tensor<float>(in.dim());
      std_pass2_ = new Tensor<float>(in.dim());

      // Find the maximum local_work_group size that is divisible by the output
      // dimension.
      for (uint32_t i = 0; i < 3; i++) {
        local_worgroup_size_3d[i] = std::min<int>(jtorch::max_local_workgroup_size,
          ((Tensor<float>*)output)->dim()[i]);
        while (((Tensor<float>*)output)->dim()[i] % local_worgroup_size_3d[i] != 0) {
          local_worgroup_size_3d[i]--;
        }
      }
    }
    if (kernel1d_norm_ == NULL) {
      kernel1d_norm_ = kernel1d_->copy();
      float* kernel1d_norm_cpu = new float[kernel1d_norm_->dataSize()];
      kernel1d_norm_->getData(kernel1d_norm_cpu);
      // Now normalize the kernel!
      const float n_feats = (float)in.dim()[2];
      float sum = 0.0f;
      for (int32_t i = 0; i < kernel1d_norm_->dim()[0]; i++) {
        sum += kernel1d_norm_cpu[i];
      }
      for (int32_t i = 0; i < kernel1d_norm_->dim()[0]; i++) {
        kernel1d_norm_cpu[i] /= (sum * sqrtf(n_feats));
      }
      kernel1d_norm_->setData(kernel1d_norm_cpu);
      delete[] kernel1d_norm_cpu;
    }
    if (std_coef_ == NULL) {
      Int3 std_coeff_dim(((Tensor<float>*)output)->dim());
      std_coeff_dim[2] = 1;  // This tensor is only 2D
      std_coef_ = new Tensor<float>(std_coeff_dim);

      float* std_coef_cpu = new float[std_coef_->dataSize()];
      float* kernel1d_norm_cpu = new float[kernel1d_norm_->dataSize()];
      kernel1d_norm_->getData(kernel1d_norm_cpu);

      // Filter an image of all 1 values to create the normalization constants
      // See norm_test.lua for proof that this works as well as:
      // https://github.com/andresy/torch/blob/master/extra/nn/SpatialSubtractiveNormalization.lua
      // The filter is seperable, but we'll just do the dumb 2D version since
      // we only do this once on startup.  --> O(n * m)
      int32_t kernel1d_size = kernel1d_->dim()[0];
      int32_t filt_rad = (kernel1d_size - 1) / 2;
      int32_t n_feats = ((Tensor<float>*)output)->dim()[2];
      int32_t height = ((Tensor<float>*)output)->dim()[1];
      int32_t width = ((Tensor<float>*)output)->dim()[0];
      for (int32_t v = 0; v < height; v++) {
        for (int32_t u = 0; u < width; u++) {
          std_coef_cpu[v * width + u] = 0.0f;
          for (int32_t v_filt = -filt_rad; v_filt <= filt_rad; v_filt++) {
            for (int32_t u_filt = -filt_rad; u_filt <= filt_rad; u_filt++) {
              int32_t u_in = u + u_filt;
              int32_t v_in = v + v_filt;
              if (u_in >= 0 && u_in < width && v_in >= 0 && v_in < height) {
                // Pixel is inside --> We'll effectively clamp zeros elsewhere.
                std_coef_cpu[v * width + u] += 
                  (kernel1d_norm_cpu[v_filt + filt_rad] * 
                   kernel1d_norm_cpu[u_filt + filt_rad]);
              }
            }
          }
          std_coef_cpu[v * width + u] /= n_feats;
        }
      }
      std_coef_->setData(std_coef_cpu);
      delete[] std_coef_cpu;
      delete[] kernel1d_norm_cpu;
    }
    if (std_ == NULL) {
      Int3 std_coeff_dim(((Tensor<float>*)output)->dim());
      std_coeff_dim[2] = 1;  // This tensor is only 2D
      std_ = new Tensor<float>(std_coeff_dim);
      // Find the maximum local_work_group size that is divisible by the output
      // dimension.
      for (uint32_t i = 0; i < 3; i++) {
        local_worgroup_size_2d[i] = std::min<int>(jtorch::max_local_workgroup_size,
          std_->dim()[i]);
        while (std_->dim()[i] % local_worgroup_size_2d[i] != 0) {
          local_worgroup_size_2d[i]--;
        }
      }
    }
  }

  void SpatialDivisiveNormalization::forwardProp(TorchData& input) { 
    init(input);
    int32_t filt_rad = (kernel1d_norm_->dim()[0] - 1) / 2;
    
    // Perform horizontal filter pass
    Tensor<float>& in = (Tensor<float>&)input;
    Tensor<float>* out = (Tensor<float>*)output;
    std::string kernel = jtorch::jtorch_path + "kernels/spatial_divisive_normalization.cl";
    cl_context->useKernel(kernel.c_str(), "SpatialDivisiveNormalizationHoriz");
    cl_context->setArg(0, in.data());
    cl_context->setArg(1, std_pass1_->data());
    cl_context->setArg(2, kernel1d_norm_->data());
    cl_context->setArg(3, filt_rad);
    cl_context->runKernel3D(jtorch::deviceid, std_pass1_->dim(), 
      local_worgroup_size_3d, false);

    // Perform vertical filter pass
    cl_context->useKernel(kernel.c_str(), "SpatialDivisiveNormalizationVert");
    cl_context->setArg(0, std_pass1_->data());
    cl_context->setArg(1, std_pass2_->data());
    cl_context->setArg(2, kernel1d_norm_->data());
    cl_context->setArg(3, filt_rad);
    cl_context->runKernel3D(jtorch::deviceid, std_pass2_->dim(), 
      local_worgroup_size_3d, false);

    // Perform accumulation and division pass
    cl_context->useKernel(kernel.c_str(), "SpatialDivisiveNormalizationAccumDiv");
    cl_context->setArg(0, std_pass2_->data());
    cl_context->setArg(1, std_->data());
    cl_context->setArg(2, std_coef_->data());
    cl_context->setArg(3, out->dim()[2]);
    cl_context->setArg(4, threshold_);
    cl_context->runKernel3D(jtorch::deviceid, std_->dim(), 
      local_worgroup_size_2d, false);

    // Perform normalization pass
    cl_context->useKernel(kernel.c_str(), "SpatialDivisiveNormalization");
    cl_context->setArg(0, in.data());
    cl_context->setArg(1, out->data());
    cl_context->setArg(2, std_->data());
    cl_context->runKernel3D(jtorch::deviceid, out->dim(), 
      local_worgroup_size_3d, false);

    /*
    FloatTensor& in = (FloatTensor&)input;
    const int32_t width = in.dim()[0];
    const int32_t height = in.dim()[1];
    const int32_t n_feats = in.dim()[2];
    const int32_t n_banks = in.dim()[3];
    const int32_t im_dim = in.dim()[0] * in.dim()[1];
    const float* kernel = kernel1d_norm_->data();
    int32_t filt_rad = (kernel1d_norm_->dim()[0] - 1) / 2;

    for (int32_t outb = 0; outb < n_banks; outb++) {
      // Replicate the self.stdestimator in the torch module:
      for (int32_t i = 0; i < im_dim; i++) {
        std_accum_[i] = 0.0f;
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
    */
  }

  TorchStage* SpatialDivisiveNormalization::loadFromFile(std::ifstream& file) {
    // This whole thing is a little wasteful.  I copy to GPU here, and then
    // I copy it back down in the constructor anyway...  But it's good enough
    // for now.
    int32_t kernel_size;
    file.read((char*)(&kernel_size), sizeof(kernel_size));
    Tensor<float>* kernel = new Tensor<float>(kernel_size);
    float* kernel_cpu = new float[kernel_size];
    file.read((char*)(kernel_cpu), kernel_size * sizeof(*kernel_cpu));
    float threshold;
    file.read((char*)(&threshold), sizeof(threshold));
    kernel->setData(kernel_cpu);
    TorchStage* ret = new SpatialDivisiveNormalization(*kernel, threshold);
    delete kernel;
    delete[] kernel_cpu;
    return ret;
  }

}  // namespace jtorch