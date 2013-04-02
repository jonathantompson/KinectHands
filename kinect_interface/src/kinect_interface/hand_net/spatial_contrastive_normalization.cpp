#include "kinect_interface/hand_net/spatial_contrastive_normalization.h"
#include "kinect_interface/hand_net/spatial_subtractive_normalization.h"
#include "kinect_interface/hand_net/spatial_divisive_normalization.h"
#include "kinect_interface/hand_net/sequential.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread_pool.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::threading;

namespace kinect_interface {
namespace hand_net {

  // kernel1d default is either TorchStage::gaussian1D<float>(n) or just a
  // vector of 1 values.
  SpatialContrastiveNormalization::SpatialContrastiveNormalization(
    const int32_t n_feats, const int32_t height, const int32_t width,
    const uint32_t kernel1d_size, const float* kernel1d, 
    const float threshold) : TorchStage() {
    if (kernel1d_size % 2 == 0) {
      throw std::wruntime_error("SpatialContrastiveNormalization() - ERROR: "
        "Averaging kernel must have odd dimensions!");
    }

    float* kernel = const_cast<float*>(kernel1d);
    if (kernel == NULL) {
      kernel = TorchStage::ones1D<float>(kernel1d_size);
    }

    network_ = new Sequential();
    network_->add(new SpatialSubtractiveNormalization(n_feats, kernel1d_size,
      kernel, height, width));
    network_->add(new SpatialDivisiveNormalization(n_feats, kernel1d_size,
      kernel, height, width, threshold));

    if (kernel1d == NULL) {
      // remove temporarily allocated kernel (since sub-modules will store
      // their own copy).
      delete[] kernel;
    }
  }

  SpatialContrastiveNormalization::~SpatialContrastiveNormalization() {
    SAFE_DELETE(network_);
  }

  void SpatialContrastiveNormalization::forwardProp(float* input, 
    jtil::threading::ThreadPool* tp) { 
    network_->forwardProp(input, tp);
  }

  TorchStage* SpatialContrastiveNormalization::loadFromFile(std::ifstream& file) {
    throw std::wruntime_error("Not yet implemented");
  }

  int32_t SpatialContrastiveNormalization::outWidth() const {
    return network_->outWidth();
  }

  int32_t SpatialContrastiveNormalization::outHeight() const {
    return network_->outHeight();
  }

  int32_t SpatialContrastiveNormalization::outNFeats() const {
    return network_->outNFeats();
  }

  float* SpatialContrastiveNormalization::output() {
    return network_->output();
  }

}  // namespace hand_net
}  // namespace kinect_interface
