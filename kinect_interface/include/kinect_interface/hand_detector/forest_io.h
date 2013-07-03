//
//  forest_io.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#ifndef KINECT_INTERFACE_HAND_DETECTOR_FOREST_IO_HEADER
#define KINECT_INTERFACE_HAND_DETECTOR_FOREST_IO_HEADER

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

#endif  // KINECT_INTERFACE_HAND_DETECTOR_FOREST_IO_HEADER
