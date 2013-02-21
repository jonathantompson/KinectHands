//
//  forest_io.cpp
//
//  Created by Jonathan Tompson on 7/20/12.
//

#if defined(WIN32) || defined(_WIN32)
#include <windows.h>
#include <tchar.h> 
#include <stdio.h>
#include <strsafe.h>
#pragma comment(lib, "User32.lib")
#endif
#include <fstream>
#include <string>
#include <stdexcept>
#include "forest_io.h"
#include "common_tree_funcs.h"
#include "generate_decision_tree.h"

using std::wstring;
using std::string;
using std::runtime_error;

#define im_width 640
#define im_height 480
// Note (width / downsample) && (height / downsample) must be integer values!

void saveForest(DecisionTree* forest, uint32_t num_trees, 
                std::string filename) {
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("saveForest() error opening file.");
  }
  file.write(reinterpret_cast<const char*>(&num_trees), 
    1 * sizeof(num_trees));
  for (uint32_t i = 0; i < num_trees; i++) {
    file.write(reinterpret_cast<const char*>(&forest[i].tree_height), 
      1 * sizeof(forest->tree_height));
    file.write(reinterpret_cast<const char*>(&forest[i].num_nodes), 
      1 * sizeof(forest->tree_height));
    file.write(reinterpret_cast<const char*>(forest[i].tree), 
      forest[i].num_nodes * sizeof(forest[i].tree[0]));
  }
  file.flush();
  file.close();
}

void loadForest(DecisionTree** forest, uint32_t* num_trees, 
                std::string filename) {
  std::ifstream in_file(filename.c_str(), 
    std::ios::in | std::ios::binary);
  if (!in_file.is_open()) {
    std::cout << "couldn't load forest from: " << filename << std::endl;
    throw std::runtime_error("loadForest() error opening file (did you forget"
      " to unzip the .bin.zip file in the root directory?).");
  }

  in_file.read(reinterpret_cast<char*>(num_trees), 
    1 * sizeof(num_trees[0]));

  (*forest) = new DecisionTree[*num_trees];

  for (uint32_t i = 0; i < (*num_trees); i++) {
    uint32_t height;
    in_file.read(reinterpret_cast<char*>(&height), 1 * sizeof(height));
    (*forest)[i].tree_height = height;

    uint32_t num_nodes;
    in_file.read(reinterpret_cast<char*>(&num_nodes), 1 * sizeof(num_nodes));
    (*forest)[i].num_nodes = num_nodes;

    DecisionTreeNode* tree = new DecisionTreeNode[num_nodes];
    in_file.read(reinterpret_cast<char*>(tree), num_nodes * sizeof(tree[0]));
    (*forest)[i].tree = tree;
  }
  in_file.close();
}

void releaseForest(DecisionTree** forest, uint32_t num_trees) {
  for (uint32_t i = 0; i < num_trees; i++) {
    delete[] (*forest)[i].tree;
    (*forest)[i].tree = NULL;
  }
  delete[] (*forest);
  (*forest) = NULL;
}