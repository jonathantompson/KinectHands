#include "jtorch/linear.h"
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

  Linear::Linear(const int32_t n_inputs, const int32_t n_outputs) 
    : TorchStage() {
    n_inputs_ = n_inputs;
    n_outputs_ = n_outputs;

    output = new Tensor<float>(Int3(n_outputs_, 1, 1));

    for (uint32_t i = 0; i < 3; i++) {
      local_worgroup_size[i] = std::min<int>(jtorch::max_local_workgroup_size,
        ((Tensor<float>*)output)->dim()[i]);
      while (((Tensor<float>*)output)->dim()[i] % local_worgroup_size[i] != 0) {
        local_worgroup_size[i]--;
      }
    }

    weights_ = new Tensor<float>(Int3(n_inputs_, n_outputs_, 1));
    biases_ = new Tensor<float>(n_outputs_);
  }

  Linear::~Linear() {
    SAFE_DELETE(output);
    SAFE_DELETE(weights_);
    SAFE_DELETE(biases_);
  }

  void Linear::setWeights(const float* weights) {
    weights_->setData(weights);
  }

  void Linear::setBiases(const float* biases) {
    biases_->setData(biases);
  }

  void Linear::init(TorchData& input)  {
    if (input.type() != TorchDataType::TENSOR_DATA) {
      throw std::wruntime_error("Linear::init() - "
        "FloatTensor expected!");
    }
    Tensor<float>& in = (Tensor<float>&)input;
    if (in.dataSize() != n_inputs_) {
      throw std::wruntime_error("Linear::init() - ERROR: input size mismatch!");
    }
  }

  void Linear::forwardProp(TorchData& input) { 
    init(input);
    Tensor<float>& in = (Tensor<float>&)input;

    std::string kernel = jtorch::jtorch_path + "kernels/linear.cl";
    cl_context->useKernel(kernel.c_str(), "Linear");
    cl_context->setArg(0, ((Tensor<float>&)input).data());
    cl_context->setArg(1, ((Tensor<float>*)output)->data());
    cl_context->setArg(2, weights_->data());
    cl_context->setArg(3, biases_->data());
    cl_context->setArg(4, n_inputs_);
    cl_context->runKernel3D(jtorch::deviceid, ((Tensor<float>*)output)->dim(),
      local_worgroup_size, false);
  }

  TorchStage* Linear::loadFromFile(std::ifstream& file) {
    int32_t n_outputs;
    int32_t n_inputs;
    file.read((char*)(&n_outputs), sizeof(n_outputs));
    file.read((char*)(&n_inputs), sizeof(n_inputs));
    Linear* ret = new Linear(n_inputs, n_outputs);

    int32_t n_weights = n_outputs * n_inputs;
    float* weights_cpu = new float[n_weights];
    file.read((char*)(weights_cpu), sizeof(weights_cpu[0]) * n_weights);
    ret->setWeights(weights_cpu);
    delete[] weights_cpu;

    float* bias_cpu = new float[n_outputs];
    file.read((char*)(bias_cpu), sizeof(bias_cpu[0]) * n_outputs);
    ret->setBiases(bias_cpu);
    delete[] bias_cpu;

    return ret;
  }

}  // namespace jtorch