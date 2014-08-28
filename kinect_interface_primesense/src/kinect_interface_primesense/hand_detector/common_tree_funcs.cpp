#include <cmath>
#include "kinect_interface_primesense/hand_detector/common_tree_funcs.h"
#include "kinect_interface_primesense/hand_detector/decision_tree_structs.h"

// Microsoft doesn't include log2 in math.h (not part of C90 standard, but
// part of C99 standard).
#if defined(_WIN32) || defined(WIN32)
float log2(float val) {
  return (log(val) / log(2.0f));
}
#endif

namespace kinect_interface_primesense {
namespace hand_detector {

  uint32_t calcTreeSize(const uint32_t height) { 
    return (1<<height) - 1; 
  }

  float calcEntropy(const float* prob) {
    float ret = 0;
    for (uint32_t i = 0; i < NUM_LABELS; i++) {
      ret += prob[i]*log2(prob[i]+EPSILON);
    }
    return -ret;
  }

};  // namespace hand_detector
};  // namespace kinect_interface_primesense