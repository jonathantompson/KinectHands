#include <cmath>
#include <stdexcept>
#include <string>
#include "kinect_interface/hand_detector/evaluate_decision_forest.h"
#include "kinect_interface/hand_detector/common_tree_funcs.h"
#include "kinect_interface/hand_detector/decision_tree_func.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"
#include "kinect_interface/depth_image_data.h"
#include "jtil/image_util/image_util.h"

using namespace jtil::image_util;

namespace kinect_interface {
namespace hand_detector {
  void evaluateDecisionForest(uint8_t* label_data,
    const DecisionTree* forest, const uint32_t max_height,
    const uint32_t num_trees, const int16_t* image_data,
    const int32_t width, const int32_t height) {
  #ifdef VERBOSE_EVALUATE
    std::cout << "evaluateDecisionTree INPUTS:";
    std::cout << "   tree_height = " << tree_height ", tree_size = " << tree_size << std::endl;
    std::cout << "   width = " << width << std::endl;
    std::cout << "   height = " << height << std::endl;
  #endif
    float hist[NUM_LABELS];

    for (int32_t v = 0; v < height; v++) {
      for (int32_t u = 0; u < width; u++) {
        int32_t index = v * width + u;
        evaluateDecisionForestPixel(label_data, forest, max_height, num_trees,
          image_data, width, height, index);
      }  // for (int32_t u = 0; u < static_cast<int32_t>(width); u++)
    }  // for (int32_t v = 0; v < static_cast<int32_t>(height); v++)
  }

  void evaluateDecisionForestPixel(uint8_t* label_data,
    const DecisionTree* forest, const uint32_t max_height,
    const uint32_t num_trees, const int16_t* image_data,
    const int32_t width, const int32_t height, const int32_t index) {
    float hist[NUM_LABELS];

    if (image_data[index] == 0 || image_data[index] >= GDT_MAX_DIST) {
      label_data[index] = 0;
    } else {
      // For each tree, evaluate the pixel adding to the accumulated histogram
      for (uint32_t i = 0; i < NUM_LABELS; i++) {
#ifndef MULTIPLY_LEAVES
        hist[i] = 0;
#else
        hist[i] = 1;
#endif
      }
      for (int32_t cur_tree = 0; cur_tree < static_cast<int32_t>(num_trees); cur_tree++) {
        DecisionTreeNode* cur_node = &forest[cur_tree].tree[0];
        uint32_t cur_height = 1;
        while (true) {       
          bool isLeaf = (cur_height == max_height || 
            cur_height == forest[cur_tree].tree_height || 
            cur_node->left_child == -1);
          if (isLeaf) {
#ifndef MULTIPLY_LEAVES
            for (uint32_t i = 0; i < NUM_LABELS; i++) {
              hist[i] += cur_node->prob[i];
            }
#else
            for (uint32_t i = 0; i < NUM_LABELS; i++) {
              hist[i] *= cur_node->prob[i];
            }
#endif
            break;
          }

          bool result;
          WL_FUNC(index, cur_node->coeff0, cur_node->coeff1, cur_node->coeff2, 
            cur_node->wl_func, width, height, image_data, result);

          if (result) {
            // Go left
            cur_node = &forest[cur_tree].tree[cur_node->left_child];
          } else {
            // Go right
            cur_node = &forest[cur_tree].tree[cur_node->right_child];
          }
          cur_height++;
        }
      }  // for (int32_t i = 0; i < num_trees; i++)
      uint8_t pixel_label = 0;
      float max_hist = -1;
      for (uint8_t i = 0; i < NUM_LABELS; i++) {
        if (hist[i] > max_hist) {
          max_hist = hist[i];
          pixel_label = i;
        }
      }
      label_data[index] = pixel_label;
    }  // if (image_data[index] == 0 || image_data[index] >= GDT_MAX_DIST)
  }

  float evaluateDecisionForestError(const DepthImageData* data,
    const DecisionTree* forest, const uint32_t num_trees,
    const bool run_median_filter, const int32_t median_filter_rad) {
    // Iterate through the images and aggregate the pixel error
    uint8_t* label_data_evaluated = new uint8_t[data->im_width * data->im_height];
    uint8_t* label_data_filtered = NULL;

    uint32_t num_correct = 0;
    uint32_t num_incorrect = 0;
    for (int32_t i = 0; i < data->num_images; i++) {
      uint8_t* cur_label_data = &data->label_data[data->im_width * data->im_height * i];
      int16_t* cur_image_data = &data->image_data[data->im_width * data->im_height * i];
      evaluateDecisionForest(label_data_evaluated, forest, forest->tree_height, 
        num_trees, cur_image_data, data->im_width, data->im_height);
      if (run_median_filter) {
        label_data_filtered = new uint8_t[data->im_width * data->im_height];
        MedianLabelFilter<uint8_t, int16_t>(label_data_filtered, 
          label_data_evaluated, cur_image_data,
          data->im_width, data->im_height, median_filter_rad);
        // swap the buffers
        uint8_t* temp;
        temp = label_data_filtered;
        label_data_filtered = label_data_evaluated;
        label_data_evaluated = temp;
      }
      for (int32_t i = 0; i < data->im_width * data->im_height; i++) {
        if(cur_image_data[i] != 0 && cur_image_data[i] < GDT_MAX_DIST) {
          if (cur_label_data[i] == label_data_evaluated[i]) {
            num_correct++;
          } else {
            num_incorrect++;
          }
        }
      }
    }
    delete[] label_data_evaluated;
    if (label_data_filtered) {
      delete[] label_data_filtered;
    }
    return static_cast<float>(num_incorrect) / 
           static_cast<float>(num_correct + num_incorrect);
  }

};  // namespace hand_detector
};  // namespace kinect_interface

