#include "kinect_interface/hand_net/parallel.h"
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

  void Parallel::forwardProp(FloatTensor& input, 
    ThreadPool& tp) {
    throw std::wruntime_error("Not yet implemented");
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
