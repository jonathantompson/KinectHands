#include "kinect_interface/hand_net/transpose.h"
#include "kinect_interface/hand_net/float_tensor.h"
#include "kinect_interface/hand_net/table.h"
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

  Transpose::Transpose() {
    output = NULL;
  }

  Transpose::~Transpose() {
  }


  TorchStage* Transpose::loadFromFile(std::ifstream& file) {
    int32_t num_permutations;
    file.read((char*)(&num_permutations), sizeof(num_permutations));
    int32_t* perms = new int32_t[num_permutations * 2];
    file.read((char*)(perms), sizeof(perms[0]) * num_permutations * 2);
    // But we don't really use it...
    TorchStage* ret_val = new Transpose();
    delete[] perms;
    return ret_val;
  }

  void Transpose::forwardProp(TorchData& input, ThreadPool& tp) {
    output = &input;
  }
}  // namespace hand_net
}  // namespace kinect_interface
