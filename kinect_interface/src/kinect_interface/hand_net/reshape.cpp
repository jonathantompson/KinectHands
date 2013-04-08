#include "kinect_interface/hand_net/reshape.h"
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

  Reshape::Reshape() : TorchStage() {
    output = NULL;
  }

  Reshape::~Reshape() {
    SAFE_DELETE(output);
  }

  void Reshape::init(FloatTensor& input, ThreadPool& tp)  {
    if (output != NULL) {
      if (!Int4::equal(input.dim(), output->dim())) {
        // Input dimension has changed!
        SAFE_DELETE(output);
      }
    }
    if (output == NULL) {
      Int4 out_dim(input.dataSize(), 1, 1, 1);
      output = new FloatTensor(out_dim);
    }
  }

  void Reshape::forwardProp(FloatTensor& input, ThreadPool& tp) { 
    init(input, tp);
    memcpy(output->data(), input.data(), 
      output->dim()[0] * sizeof(output->data()[0]));
  }

}  // namespace hand_net
}  // namespace kinect_interface
