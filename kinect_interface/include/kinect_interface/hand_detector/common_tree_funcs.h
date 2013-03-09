//
//  common_tree_funcs.h
//
//  Created by Jonathan Tompson on 7/20/12.
//
//  For internal use only
//

#ifndef KINECT_INTERFACE_HAND_DETECTOR_COMMON_TREE_FUNCS_HEADER
#define KINECT_INTERFACE_HAND_DETECTOR_COMMON_TREE_FUNCS_HEADER

#include "jtil/math/math_types.h"

#if defined(_WIN32) || defined(WIN32)
float log2(float val);
#endif

namespace kinect_interface {
namespace hand_detector {

  uint32_t calcTreeSize(const uint32_t height);
  float calcEntropy(const float* prob);

};
};

#endif  // KINECT_INTERFACE_HAND_DETECTOR_COMMON_TREE_FUNCS_HEADER