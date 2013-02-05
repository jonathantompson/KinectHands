//
//  common_tree_funcs.h
//
//  Created by Jonathan Tompson on 7/20/12.
//
//  For internal use only
//

#ifndef DECISION_FOREST_COMMON_TREE_FUNCS_HEADER
#define DECISION_FOREST_COMMON_TREE_FUNCS_HEADER

#include "math/math_types.h"

#if defined(_WIN32) || defined(WIN32)
float log2(float val);
#endif

namespace decision_forest {

  uint32_t calcTreeSize(uint32_t height);
  float calcEntropy(float* prob);

}

#endif  // DECISION_FOREST_COMMON_TREE_FUNCS_HEADER