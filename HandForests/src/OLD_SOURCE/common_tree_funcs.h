//
//  common_tree_funcs.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#ifndef UNNAMED_COMMON_TREE_FUNCS_HEADER
#define UNNAMED_COMMON_TREE_FUNCS_HEADER

#include "math/math_types.h"

#if defined(_WIN32) || defined(WIN32)
float log2(float val);
#endif

uint32_t calcTreeSize(uint32_t height);

/*
int32_t getLChild(int32_t i);

int32_t getRChild(int32_t i);

int32_t getParent(int32_t i);
*/

float calcEntropy(float* prob);

#endif  // UNNAMED_EVALUATE_DECISION_TREE_HEADER