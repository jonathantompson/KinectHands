#include "kinect_interface/hand_net/spatial_contrastive_normalization.h"
#include "kinect_interface/hand_net/spatial_subtractive_normalization.h"
#include "kinect_interface/hand_net/spatial_divisive_normalization.h"
#include "kinect_interface/hand_net/sequential.h"
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

  // kernel1d default is either TorchStage::gaussian1D<float>(n) or just a
  // vector of 1 values.
  SpatialContrastiveNormalization::SpatialContrastiveNormalization(
    const FloatTensor* kernel1d, const float threshold) : TorchStage() {
    const FloatTensor* kernel;
    if (kernel1d) {
      if (kernel1d->dataSize() % 2 == 0 || kernel1d->dim()[1] != 1 ||
        kernel1d->dim()[2] != 1 || kernel1d->dim()[3] != 1) {
        throw std::wruntime_error("SpatialSubtractiveNormalization() - ERROR: "
          "Averaging kernel must be 1D and have odd size!");
      }
      kernel = kernel1d;
    } else {
      kernel = FloatTensor::ones1D(7);
    }

    network_ = new Sequential();
    network_->add(new SpatialSubtractiveNormalization(*kernel));
    network_->add(new SpatialDivisiveNormalization(*kernel, threshold));

    if (kernel1d == NULL) {
      // remove temporarily allocated kernel (since sub-modules will store
      // their own copy).
      delete kernel;
    }
  }

  SpatialContrastiveNormalization::~SpatialContrastiveNormalization() {
    SAFE_DELETE(network_);
  }

  void SpatialContrastiveNormalization::forwardProp(TorchData& input, 
    jtil::threading::ThreadPool& tp) {
    network_->forwardProp(input, tp);
    output = network_->output;
  }

  TorchStage* SpatialContrastiveNormalization::loadFromFile(std::ifstream& file) {
    throw std::wruntime_error("Not yet implemented");
  }

}  // namespace hand_net
}  // namespace kinect_interface
