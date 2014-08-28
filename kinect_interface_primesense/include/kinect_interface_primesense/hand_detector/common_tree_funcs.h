//
//  common_tree_funcs.h
//
//  Created by Jonathan Tompson on 7/20/12.
//
//  For internal use only
//

#pragma once

#include "jtil/math/math_types.h"

#if defined(_WIN32) || defined(WIN32)
float log2(float val);
#endif

namespace kinect_interface_primesense {
namespace hand_detector {

  uint32_t calcTreeSize(const uint32_t height);
  float calcEntropy(const float* prob);

};
};
