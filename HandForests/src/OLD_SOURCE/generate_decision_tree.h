//
//  generate_decision_tree.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#ifndef UNNAMED_GENERATE_DECISION_TREE_HEADER
#define UNNAMED_GENERATE_DECISION_TREE_HEADER

#include "math/math_types.h"
#include "image_data.h"

#ifndef NULL
  #define NULL 0
#endif

// #define VERBOSE_GENERATION  // Print verbose messages during tree generation

// DTCoeffs --> Input coefficients and settings
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
  int32_t num_images_to_consider;
  uint32_t tree_height;
  float min_info_gain;
  uint32_t max_pixels_per_image_per_label;
  uint32_t dt_index;
  unsigned int seed;
};

// The computational routine
class GenerateDecisionTree {
public:
  void generateDecisionTree(
    DecisionTree* dt,                 // output --> Must be pre-allocated!
    ImageData* training_data,        // image input
    WLSet* wl_set,                    // WL input
    TrainingSettings* settings,       // settings input
    DecisionTree* bootstrap_dt,       // input
    uint32_t bootstrap_num_trees);    // input     
};

#endif
