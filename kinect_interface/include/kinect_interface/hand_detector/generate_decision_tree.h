//
//  generate_decision_tree.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#ifndef KINECT_INTERFACE_HAND_DETECTOR_GENERATE_DECISION_TREE_HEADER
#define KINECT_INTERFACE_HAND_DETECTOR_GENERATE_DECISION_TREE_HEADER

#include "jtil/math/math_types.h"
#include "kinect_interface/depth_image_data.h"

namespace kinect_interface {
namespace hand_detector {

  struct WLSet;
  struct DecisionTreeNode;
  struct DecisionTree;
  struct TrainingSettings;

  // The main and only computational routine
  class GenerateDecisionTree {
  public:
    void generateDecisionTree(
      DecisionTree* dt,                 // output --> Must be pre-allocated!
      DepthImageData* training_data,    // image input
      WLSet* wl_set,                    // WL input
      TrainingSettings* settings,       // settings input
      DecisionTree* bootstrap_dt,       // input
      uint32_t bootstrap_num_trees);    // input     
  };

};  // namespace hand_detector
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_DETECTOR_GENERATE_DECISION_TREE_HEADER
