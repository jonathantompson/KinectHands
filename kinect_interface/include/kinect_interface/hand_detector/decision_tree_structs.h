//
//  decision_tree_structs.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#pragma once

#include "jtil/math/math_types.h"

#ifndef NULL
  #define NULL 0
#endif

#define NUM_LABELS 2  // The labels must be indexed (per pixel) from 0 --> NUM_LABELS - 1
#define DT_DOWNSAMPLE 4  // 1 --> no downsample, 4 --> 16th origional size (default)
#define NUM_WL_FUNCS 2  // My depth test and the Kinect paper's depth test
// #define VERBOSE_GENERATION  // Print verbose messages during tree generation

namespace kinect_interface {
namespace hand_detector {

  struct WLSet {
    int32_t* wl_coeffs0;
    int32_t* wl_coeffs1;
    int16_t* wl_coeffs2;
    uint8_t* wl_funcs;
    uint32_t* wl_coeffs_sizes;
    uint32_t num_samples_per_node;
  };

  struct DecisionTreeNode {
    int32_t coeff0;
    int32_t coeff1;
    int16_t coeff2;
    uint8_t wl_func;
    float prob[NUM_LABELS];
    int32_t left_child;
    int32_t right_child;
  };

  // DecisionTree --> Output coefficients and histogram
  struct DecisionTree {
    DecisionTree() : tree(NULL), num_nodes(0) { }
    DecisionTreeNode* tree;
    uint32_t num_nodes;
    uint32_t tree_height;
  };

  struct TrainingSettings {
    int32_t num_im_to_consider;
    uint32_t tree_height;
    float min_info_gain;
    uint32_t max_pix_per_im_per_label;
    uint32_t dt_index;
    unsigned int seed;
  };

};  // namespace hand_detector
};  // namespace kinect_interface
