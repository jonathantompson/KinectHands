#include "kinect_interface/hand_net/parallel.h"
#include "jtil/data_str/vector_managed.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::data_str;

namespace kinect_interface {
namespace hand_net {

  Parallel::Parallel() {
    // Create an empty container
    network_ = new VectorManaged<TorchStage*>(1);
  }

  Parallel::~Parallel() {
    SAFE_DELETE(network_);
  }

  void Parallel::add(TorchStage* stage) {
    network_->pushBack(stage);
  }

  TorchStage* Parallel::loadFromFile(std::ifstream& file) {
    int n_nodes;
    file.read(reinterpret_cast<char*>(&n_nodes), sizeof(n_nodes));
    Parallel* ret = new Parallel();
    ret->network_->capacity(n_nodes);
    for (uint32_t i = 0; i < n_nodes; i++) {
      ret->network_->pushBack(TorchStage::loadFromFile(file));
    }
    return ret;
  }

  int32_t Parallel::outWidth(const uint32_t index) const {
    if (network_ == NULL) {
      throw std::wruntime_error("Parallel::outWidth() - ERROR: "
        "Network is empty!");
    }
    return (*network_)[index]->outWidth();
  }

  int32_t Parallel::outHeight(const uint32_t index) const {
    if (network_ == NULL) {
      throw std::wruntime_error("Parallel::outHeight() - ERROR: "
        "Network is empty!");
    }
    return (*network_)[index]->outHeight();
  }

  int32_t Parallel::outNFeats(const uint32_t index) const {
    if (network_ == NULL) {
      throw std::wruntime_error("Parallel::outNFeats() - ERROR: "
        "Network is empty!");
    }
    return (*network_)[index]->outNFeats();
  }

  void Parallel::forwardProp(float* input, jtil::threading::ThreadPool* tp) {
    throw std::wruntime_error("Not yet implemented");
  }

  float* Parallel::output(const uint32_t index) {
    if (network_ == NULL) {
      throw std::wruntime_error("Parallel::output() - ERROR: "
        "Network is empty!");
    }
    return (*network_)[index]->output;
  }

  uint32_t Parallel::numBanks() const {
    if (network_ == NULL) {
      throw std::wruntime_error("Parallel::output() - ERROR: "
        "Network is empty!");
    }
    return (*network_).size();
  }

}  // namespace hand_net
}  // namespace kinect_interface
