#include "kinect_interface/hand_net/reshape.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread_pool.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::threading;

namespace kinect_interface {
namespace hand_net {

  Reshape::Reshape(const int32_t feats_in, const int32_t height, 
    const int32_t width) : TorchStage() {
    feats_in_ = feats_in;
    width_ = width;
    height_ = height;

    output = new float[feats_in_ * width_ * height_];
  }

  Reshape::~Reshape() {
    SAFE_DELETE_ARR(output);
  }

  void Reshape::forwardProp(float* input, jtil::threading::ThreadPool* tp) { 
    memcpy(output, input, feats_in_ * width_ * height_ * sizeof(output[0]));
  }

}  // namespace hand_net
}  // namespace kinect_interface
