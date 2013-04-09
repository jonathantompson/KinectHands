#include <sstream>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include "kinect_interface/hand_net/torch_stage.h"
#include "kinect_interface/hand_net/linear.h"
#include "kinect_interface/hand_net/parallel.h"
#include "kinect_interface/hand_net/reshape.h"
#include "kinect_interface/hand_net/sequential.h"
#include "kinect_interface/hand_net/spatial_contrastive_normalization.h"
#include "kinect_interface/hand_net/spatial_convolution_map.h"
#include "kinect_interface/hand_net/spatial_divisive_normalization.h"
#include "kinect_interface/hand_net/spatial_lp_pooling.h"
#include "kinect_interface/hand_net/spatial_max_pooling.h"
#include "kinect_interface/hand_net/spatial_subtractive_normalization.h"
#include "kinect_interface/hand_net/tanh.h"
#include "kinect_interface/hand_net/threshold.h"
#include "kinect_interface/hand_net/join_table.h"
#include "jtil/exceptions/wruntime_error.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

namespace kinect_interface {
namespace hand_net {

  TorchStage::TorchStage() {
    output = NULL; 
  }

  TorchStage::~TorchStage() {
    
  }

  TorchStage* TorchStage::loadFromFile(const std::string& file) {
    TorchStage* ret = NULL;
    std::ifstream ifile(file.c_str(), std::ios::in|std::ios::binary);
    if (ifile.is_open()) {
      ifile.seekg(0, std::ios::beg);
      // Now recursively load the network
      std::cout << "Loading torch model..." << std::endl;
      ret = TorchStage::loadFromFile(ifile);
    } else {
      std::stringstream ss;
      ss << "HandNet::loadFromFile() - ERROR: Could not open convnet";
      ss << " file " << file << std::endl;
      throw std::wruntime_error(ss.str());
    }
    return ret;
  }

  TorchStage* TorchStage::loadFromFile(std::ifstream& ifile) { 
    // Read in the enum type:
    int type;
    ifile.read(reinterpret_cast<char*>(&type), sizeof(type));
    switch (type) {
    case SEQUENTIAL_STAGE:
      std::cout << "  Loading Sequential..." << std::endl;
      return Sequential::loadFromFile(ifile);
    case PARALLEL_STAGE:
      std::cout << "  Loading Parallel..." << std::endl;
      return Parallel::loadFromFile(ifile);
    case TANH_STAGE:
      std::cout << "  Loading Tanh..." << std::endl;
      return Tanh::loadFromFile(ifile);
    case THRESHOLD_STAGE:
      std::cout << "  Loading Threshold..." << std::endl;
      return Threshold::loadFromFile(ifile);
    case LINEAR_STAGE:
      std::cout << "  Loading Linear..." << std::endl;
      return Linear::loadFromFile(ifile);
    case RESHAPE_STAGE:
      std::cout << "  Loading Reshape..." << std::endl;
      return Reshape::loadFromFile(ifile);
    case SPATIAL_CONVOLUTION_MAP_STAGE:
      std::cout << "  Loading SpatialConvolutionMap..." << std::endl;
      return SpatialConvolutionMap::loadFromFile(ifile);
    case SPATIAL_LP_POOLING_STAGE:
      std::cout << "  Loading SpatialLPPooling..." << std::endl;
      return SpatialLPPooling::loadFromFile(ifile);
    case SPATIAL_MAX_POOLING_STAGE:
      std::cout << "  Loading SpatialMaxPooling..." << std::endl;
      return SpatialMaxPooling::loadFromFile(ifile);
    case SPATIAL_SUBTRACTIVE_NORMALIZATION_STAGE:
      std::cout << "  Loading SpatialSubtractiveNormalization..." << std::endl;
      return SpatialSubtractiveNormalization::loadFromFile(ifile);
    case SPATIAL_DIVISIVE_NORMALIZATION_STAGE:
      std::cout << "  Loading SpatialDivisiveNormalization..." << std::endl;
      return SpatialDivisiveNormalization::loadFromFile(ifile);
    case SPATIAL_CONTRASTIVE_NORMALIZATION_STAGE:
      std::cout << "  Loading SpatialContrastiveNormalization..." << std::endl;
      return SpatialContrastiveNormalization::loadFromFile(ifile);
    case JOIN_TABLE_STAGE:
      std::cout << "  Loading JoinTable..." << std::endl;
      return JoinTable::loadFromFile(ifile);
    case TRANSPOSE_STAGE:
      std::cout << "  Loading Transpose..." << std::endl;
    default:
      throw std::wruntime_error("TorchStage::loadFromFile() - ERROR: "
        "Node type not recognized!");
    }
  }

}  // namespace hand_net
}  // namespace kinect_interface
