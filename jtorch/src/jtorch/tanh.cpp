#include "jtorch/tanh.h"
#include "jtorch/jtorch.h"
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

  Tanh::Tanh() : TorchStage() {
    output = NULL;
  }

  Tanh::~Tanh() {
    SAFE_DELETE(output);
  }

  void Tanh::init(TorchData& input)  {
    if (input.type() != TorchDataType::FLOAT_TENSOR_DATA) {
      throw std::wruntime_error("Tanh::init() - FloatTensor expected!");
    }
    FloatTensor& in = (FloatTensor&)input;
    if (output != NULL) {
      if (!Int3::equal(in.dim(), ((FloatTensor*)output)->dim())) {
        // Input dimension has changed!
        SAFE_DELETE(output);
      }
    }
    if (output == NULL) {
      output = new FloatTensor(in.dim());
      // Find the maximum local_work_group size that is divisible by the output
      // dimension.
      for (uint32_t i = 0; i < 3; i++) {
        local_worgroup_size[i] = std::min<int>(jtorch::max_local_workgroup_size,
          ((FloatTensor*)output)->dim()[i]);
        while (((FloatTensor*)output)->dim()[i] % local_worgroup_size[i] != 0) {
          local_worgroup_size[i]--;
        }
      }
    }
  }

  void Tanh::forwardProp(TorchData& input) { 
    init(input);
    std::string kernel = jtorch::jtorch_path + "kernels/tanh.cl";
    cl_context->useKernel(kernel.c_str(), "TanH");
    cl_context->setArg(0, ((FloatTensor&)input).data());
    cl_context->setArg(1, ((FloatTensor*)output)->data());
    cl_context->runKernel3D(jtorch::deviceid, ((FloatTensor*)output)->dim(),
      local_worgroup_size, false);
  }

  TorchStage* Tanh::loadFromFile(std::ifstream& file) {
    // Nothing to do for Tanh
    return new Tanh();
  }

}  // namespace jtorch