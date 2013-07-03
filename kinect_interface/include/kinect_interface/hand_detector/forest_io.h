//
//  forest_io.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#pragma once

#include <string>
#include "jtil/math/math_types.h"

namespace kinect_interface {
namespace hand_detector {

  struct DecisionTree;

  void saveForest(DecisionTree*& forest, const int32_t num_trees, 
    const std::string& filename);

  void loadForest(DecisionTree*& forest, int32_t& num_trees, 
    const std::string& filename);

  void releaseForest(DecisionTree*& forest, const int32_t num_trees);

};  // namespace hand_detector
};  // namespace kinect_interface
