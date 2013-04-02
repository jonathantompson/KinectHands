#include "kinect_interface/hand_net/sequential.h"
#include "jtil/data_str/vector_managed.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::data_str;

namespace kinect_interface {
namespace hand_net {

  Sequential::Sequential() {
    // Create an empty container
    network_ = new VectorManaged<TorchStage*>(1);
  }

  Sequential::~Sequential() {
    SAFE_DELETE(network_);
  }

  void Sequential::add(TorchStage* stage) {
    network_->pushBack(stage);
  }

  TorchStage* Sequential::loadFromFile(std::ifstream& file) {
    throw std::wruntime_error("Not yet implemented");
  }

  int32_t Sequential::outWidth() const {
    if (network_ == NULL) {
      throw std::wruntime_error("Sequential::outWidth() - ERROR: "
        "Network is empty!");
    }
    return (*network_)[network_->size()-1]->outWidth();
  }

  int32_t Sequential::outHeight() const {
    if (network_ == NULL) {
      throw std::wruntime_error("Sequential::outHeight() - ERROR: "
        "Network is empty!");
    }
    return (*network_)[network_->size()-1]->outHeight();
  }

  int32_t Sequential::outNFeats() const {
    if (network_ == NULL) {
      throw std::wruntime_error("Sequential::outNFeats() - ERROR: "
        "Network is empty!");
    }
    return (*network_)[network_->size()-1]->outNFeats();
  }

  void Sequential::forwardProp(float* input, jtil::threading::ThreadPool* tp) {
    if (network_ == NULL) {
      throw std::wruntime_error("Sequential::forwardProp() - ERROR: "
        "Network is empty!");
    }
    (*network_)[0]->forwardProp(input, tp);
    for (uint32_t i = 1; i < network_->size(); i++) {
      (*network_)[i]->forwardProp((*network_)[i-1]->output, tp);
    }
  }

  float* Sequential::output() {
    if (network_ == NULL) {
      throw std::wruntime_error("Sequential::output() - ERROR: "
        "Network is empty!");
    }
    return (*network_)[network_->size()-1]->output;
  }

}  // namespace hand_net
}  // namespace kinect_interface
