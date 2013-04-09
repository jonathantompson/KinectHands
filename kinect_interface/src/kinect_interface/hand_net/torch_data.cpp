#include <sstream>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include "kinect_interface/hand_net/torch_data.h"
#include "jtil/exceptions/wruntime_error.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

namespace kinect_interface {
namespace hand_net {

  TorchData::TorchData() {
  }

  TorchData::~TorchData() {
  }

}  // namespace hand_net
}  // namespace kinect_interface
