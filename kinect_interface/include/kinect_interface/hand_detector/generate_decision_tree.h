//
//  generate_decision_tree.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#ifndef KINECT_INTERFACE_HAND_DETECTOR_GENERATE_DECISION_TREE_HEADER
#define KINECT_INTERFACE_HAND_DETECTOR_GENERATE_DECISION_TREE_HEADER

#include "jtil/math/math_types.h"

namespace kinect_interface {

struct DepthImageData;

namespace hand_detector {
  struct WLSet;
  struct DecisionTreeNode;
  struct DecisionTree;
  struct TrainingSettings;

  class GenerateDecisionTree {
  public:
    GenerateDecisionTree() {}
    ~GenerateDecisionTree() {}

    // The main and only computational routine:
    static void generateDecisionTree(
      DecisionTree& dt,                     // output --> Must be pre-allocated
      const DepthImageData& train_data,     // image input
      const WLSet& wl_set,                  // WL input
      const TrainingSettings& settings);    // settings input

  private:
    static int32_t populateOccupancyList(const DepthImageData& data, 
      const int32_t max_pix_per_image, const unsigned int& seed,
      int32_t*& cur_occ_list, int32_t*& next_occ_list);
  };

};  // namespace hand_detector
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_DETECTOR_GENERATE_DECISION_TREE_HEADER
