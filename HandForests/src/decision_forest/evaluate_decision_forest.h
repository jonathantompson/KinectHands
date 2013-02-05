//
//  evaluate_decision_forest.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#ifndef UNNAMED_EVALUATE_DECISION_FOREST_HEADER
#define UNNAMED_EVALUATE_DECISION_FOREST_HEADER

// #define VERBOSE_EVALUATE  // Print perbose messages during tree generation
// #define MULTIPLY_LEAVES  // If not defined leaves will be added

#include "math/math_types.h"

struct ImageData;
struct DecisionTree;

void evaluateDecisionForest(uint8_t* label_data,       // output  --> Must be preallocated
                            DecisionTree* forest,      // input
                            uint32_t max_height,       // input
                            uint32_t num_trees,        // input
                            int16_t* image_data,       // input
                            int32_t width,             // input
                            int32_t height);           // input

float evaluateDecisionForestError(ImageData* data,            // input
                                  DecisionTree* forest,        // input
                                  uint32_t num_trees,          // input
                                  bool run_median_filter,      // input    
                                  int32_t median_filter_rad);  // input

#endif  // UNNAMED_EVALUATE_DECISION_TREE_HEADER