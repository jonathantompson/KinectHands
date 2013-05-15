#include "jtorch/spatial_convolution_map.h"
#include "jtorch/tensor.h"
#include "jtorch/jtorch.h"
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
using namespace jcl;

namespace jtorch {

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

    weights_ = new Tensor<float>(Int3(feats_out_ * fan_in_, filt_height_,
      filt_width_));
    biases_ = new Tensor<float>(feats_out_);
    conn_table_ = new Tensor<int>(Int3(feats_out_, fan_in_, 2));
  }

  SpatialConvolutionMap::~SpatialConvolutionMap() {
    SAFE_DELETE(output);
    SAFE_DELETE(weights_);
    SAFE_DELETE(biases_);
    SAFE_DELETE(conn_table_);
  }

  void SpatialConvolutionMap::setWeights(const float* weights) {
    weights_->setData(weights);
  }

  void SpatialConvolutionMap::setBiases(const float* biases) {
    biases_->setData(biases);
  }

  void SpatialConvolutionMap::setConnTable(const int* conn_table) {
    conn_table_->setData(conn_table);
  }

  void SpatialConvolutionMap::init(TorchData& input)  {
    if (input.type() != TorchDataType::TENSOR_DATA) {
      throw std::wruntime_error("SpatialConvolutionMap::init() - "
        "FloatTensor expected!");
    }
    Tensor<float>& in = (Tensor<float>&)input;
    if (in.dim()[2] != feats_in_) {
      throw std::wruntime_error("SpatialConvolutionMap::init() - ERROR: "
        "incorrect number of input features!");
    }
    if (output != NULL) {
      Int3 out_dim(in.dim());
      out_dim[0] = out_dim[0] - filt_width_ + 1;
      out_dim[1] = out_dim[1] - filt_height_ + 1;
      out_dim[2] = feats_out_;
      if (!Int3::equal(out_dim, ((Tensor<float>*)output)->dim())) {
        // Input dimension has changed!
        SAFE_DELETE(output);
      }
    }
    if (output == NULL) {
      Int3 out_dim(in.dim());
      out_dim[0] = out_dim[0] - filt_width_ + 1;
      out_dim[1] = out_dim[1] - filt_height_ + 1;
      out_dim[2] = feats_out_;
      output = new Tensor<float>(out_dim);
      // Find the maximum local_work_group size that is divisible by the output
      // dimension.
      for (uint32_t i = 0; i < 3; i++) {
        local_worgroup_size[i] = std::min<int>(jtorch::max_local_workgroup_size,
          ((Tensor<float>*)output)->dim()[i]);
        while (((Tensor<float>*)output)->dim()[i] % local_worgroup_size[i] != 0) {
          local_worgroup_size[i]--;
        }
      }
    }
  }

  void SpatialConvolutionMap::forwardProp(TorchData& input) { 
    init(input);
    Tensor<float>& in = (Tensor<float>&)input;
    const int32_t n_banks = in.dim()[3];
    std::string kernel = jtorch::jtorch_path + 
      "kernels/spatial_convolution_map.cl";
    cl_context->useKernel(kernel.c_str(), "SpatialConvolutionMap");
    cl_context->setArg(0, ((Tensor<float>&)input).data());
    cl_context->setArg(1, ((Tensor<float>*)output)->data());
    cl_context->runKernel3D(jtorch::deviceid, ((Tensor<float>*)output)->dim(),
      local_worgroup_size, false);
  }

  /*
  void SpatialConvolutionMap::forwardPropThread(const int32_t outf) {
    const int32_t out_w = ((FloatTensor*)output)->dim()[0];
    const int32_t out_h = ((FloatTensor*)output)->dim()[1];
    const int32_t out_dim = out_w * out_h;
    const int32_t in_dim = cur_input_width_ * cur_input_height_;

    // Initialize the output array to the convolution bias:
    // http://www.torch.ch/manual/nn/index#spatialconvolution
    // Set the output layer to the current bias
    for (int32_t uv = outf * out_dim; uv < ((outf+1) * out_dim); uv++) {
      cur_output_[uv] = biases[outf];
    }

    // Now iterate through the connection table:
    for (int32_t inf = 0; inf < fan_in_; inf++) {
      int32_t inf_index = (int32_t)conn_table[outf][inf * 2];
      int32_t weight_index = (int32_t)conn_table[outf][inf * 2 + 1];
      float* cur_filt = weights[weight_index];

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
  */

  TorchStage* SpatialConvolutionMap::loadFromFile(std::ifstream& file) {
    int32_t filt_width, filt_height, n_input_features, n_output_features;
    int32_t filt_fan_in;
    file.read((char*)(&filt_width), sizeof(filt_width));
    file.read((char*)(&filt_height), sizeof(filt_height));
    file.read((char*)(&n_input_features), sizeof(n_input_features));
    file.read((char*)(&n_output_features), sizeof(n_output_features));
    file.read((char*)(&filt_fan_in), sizeof(filt_fan_in));

    SpatialConvolutionMap* ret = new SpatialConvolutionMap(n_input_features,
      n_output_features, filt_fan_in, filt_height, filt_width);

    int32_t filt_dim = filt_width * filt_height;
    float* weights = new float[n_output_features * filt_fan_in * filt_dim];
    for (int32_t i = 0; i < n_output_features * filt_fan_in; i++) {
      float* bank = &weights[i * filt_dim];
      file.read((char*)(bank), sizeof(bank[0]) * filt_dim);
    }
    ret->setWeights(weights);
    delete[] weights;

    uint16_t* cur_conn_table = new uint16_t[filt_fan_in * 2];
    int* conn_table = new int[n_output_features * filt_fan_in * 2];
    for (int32_t i = 0; i < n_output_features; i++) {
      file.read((char*)cur_conn_table, 
        sizeof(cur_conn_table[0]) * filt_fan_in * 2);
      for (int32_t j = 0; j < filt_fan_in * 2; j++) {
        conn_table[i * filt_fan_in * 2 + j] = (int)cur_conn_table[j];
      }
    }
    ret->setConnTable(conn_table);
    delete[] cur_conn_table;
    delete[] conn_table;

    float* biases = new float[n_output_features];
    file.read((char*)(biases), sizeof(biases[0]) * n_output_features);
    ret->setBiases(biases);
    delete[] biases;

    return ret;
  }

}  // namespace jtorch