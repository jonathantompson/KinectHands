//
//  forest_io.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#ifndef UNNAMED_FOREST_IO_HEADER
#define UNNAMED_FOREST_IO_HEADER

#include <string>
#include "math/math_types.h"

struct DecisionTree;

void saveForest(DecisionTree* forest, uint32_t num_trees, std::string filename);

void loadForest(DecisionTree** forest, uint32_t* num_trees, std::string filename);

void releaseForest(DecisionTree** forest, uint32_t num_trees);

#endif  // UNNAMED_FOREST_IO_HEADER
