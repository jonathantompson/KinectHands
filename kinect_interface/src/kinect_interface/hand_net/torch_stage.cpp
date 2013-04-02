#include "kinect_interface/hand_net/torch_stage.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

namespace kinect_interface {
namespace hand_net {

  TorchStage::TorchStage() {
    output = NULL; 
  }

  TorchStage::~TorchStage() {
    
  }

}  // namespace hand_net
}  // namespace kinect_interface
