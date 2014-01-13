//
//  generate_decision_tree.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#pragma once

#include "jtil/math/math_types.h"

namespace kinect_interface {

namespace hand_detector {
  struct WLSet;
  struct DecisionTreeNode;
  struct DecisionTree;
  struct TrainingSettings;
  struct DepthImageData;

  class GenerateDecisionTree {
  public:
    GenerateDecisionTree() {}
    ~GenerateDecisionTree() {}

    // The main and only computational routine:
    // Note: For some unknown reason, std::thread gets the wrong reference if
    // inputs are not passed in as pointers.
    static void generateDecisionTree(
      DecisionTree* dt,                     // output --> Must be pre-allocated
      const DepthImageData* train_data,     // image input
      const WLSet* wl_set,                  // WL input
      TrainingSettings *settings);    // settings input

  private:
    static int32_t populateOccupancyList(const DepthImageData& data, 
      const int32_t max_pix_per_image, unsigned int& seed,
      int32_t*& cur_occ_list, int32_t*& next_occ_list);
  };

};  // namespace hand_detector
};  // namespace kinect_interface
