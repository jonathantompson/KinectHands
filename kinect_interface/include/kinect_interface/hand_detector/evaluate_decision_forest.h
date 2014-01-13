//
//  evaluate_decision_forest.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#pragma once

// #define VERBOSE_EVALUATE  // Print perbose messages during tree generation
// #define MULTIPLY_LEAVES  // If not defined leaves will be added

#if defined(WIN32) || defined(_WIN32)
  #define force_inline __forceinline 
#else
//  #define force_inline inline
  #define force_inline
#endif

#include "jtil/math/math_types.h"

namespace kinect_interface {

struct DepthImageData;

namespace hand_detector {
  struct DecisionTree;

  void evaluateDecisionForest(uint8_t* label_data,
    const DecisionTree* forest, const uint32_t max_height,
    const uint32_t num_trees, const int16_t* image_data,
    const int32_t width, const int32_t height);

  force_inline void evaluateDecisionForestPixel(uint8_t* label_data,
    const DecisionTree* forest, const uint32_t max_height,
    const uint32_t num_trees, const int16_t* image_data,
    const int32_t width, const int32_t height, const int32_t index);

};  // namespace hand_detector
};  // namespace kinect_interface
