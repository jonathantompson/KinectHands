//
//  common_tree_funcs.cpp
//
//  Created by Jonathan Tompson on 7/20/12.
//

#include <cmath>
#include "common_tree_funcs.h"
#include "generate_decision_tree.h"

// Microsoft doesn't include log2 in math.h (not part of C90 standard, but
// part of C99 standard).
#if defined(_WIN32) || defined(WIN32)
float log2(float val) {
  return (log(val) / log(2.0f));
}
#endif

uint32_t calcTreeSize(uint32_t height) { return (1<<height) - 1; }

int32_t getLChild(int32_t i) {
  return 2*i + 1;
}

int32_t getRChild(int32_t i) {
  return 2*i + 2;
}

int32_t getParent(int32_t i) {
  return (i-1) / 2;
}

float calcEntropy(float* prob) {
  float ret = 0;
  for (uint32_t i = 0; i < NUM_LABELS; i++) {
    ret += prob[i]*log2(prob[i]+EPSILON);
  }
  return -ret;
}