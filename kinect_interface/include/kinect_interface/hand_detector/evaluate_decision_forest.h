//
//  evaluate_decision_forest.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#ifndef KINECT_INTERFACE_HAND_DETECTOR_EVALUATE_DECISION_FOREST_HEADER
#define KINECT_INTERFACE_HAND_DETECTOR_EVALUATE_DECISION_FOREST_HEADER

// #define VERBOSE_EVALUATE  // Print perbose messages during tree generation
// #define MULTIPLY_LEAVES  // If not defined leaves will be added

#include "jtil/math/math_types.h"

namespace kinect_interface {

struct DepthImageData;

namespace hand_detector {
  struct DecisionTree;

  void evaluateDecisionForest(uint8_t* label_data,  // Output
    const DecisionTree* forest, const uint32_t max_height,
    const uint32_t num_trees, const int16_t* image_data,
    const int32_t width, const int32_t height);

  float evaluateDecisionForestError(const DepthImageData* data,
    const DecisionTree* forest, const uint32_t num_trees,
    const bool run_median_filter, const int32_t median_filter_rad);

};  // namespace hand_detector
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_DETECTOR_EVALUATE_DECISION_FOREST_HEADER