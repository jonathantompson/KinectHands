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
#include <iostream>
#include "kinect_interface/hand_detector/forest_io.h"
#include "kinect_interface/hand_detector/common_tree_funcs.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"

using std::wstring;
using std::string;
using std::runtime_error;

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }
// Note (width / downsample) && (height / downsample) must be integer values!

namespace kinect_interface {
namespace hand_detector {
  void saveForest(DecisionTree*& forest, const int32_t num_trees, 
    const std::string& filename) {
    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error("saveForest() error opening file.");
    }
    file.write(reinterpret_cast<const char*>(&num_trees), 
      1 * sizeof(num_trees));
    for (int32_t i = 0; i < num_trees; i++) {
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

  void loadForest(DecisionTree*& forest, int32_t& num_trees, 
    const std::string& filename) {
    std::ifstream in_file(filename.c_str(), 
      std::ios::in | std::ios::binary);
    if (!in_file.is_open()) {
      std::cout << "couldn't load forest from: " << filename << std::endl;
      throw std::runtime_error("loadForest() error opening file (did you "
        "forget to unzip the .bin.zip file in the root directory?).");
    }

    in_file.read(reinterpret_cast<char*>(&num_trees), 
      1 * sizeof(num_trees));

    forest = new DecisionTree[num_trees];

    for (int32_t i = 0; i < num_trees; i++) {
      uint32_t height;
      in_file.read(reinterpret_cast<char*>(&height), 1 * sizeof(height));
      forest[i].tree_height = height;

      uint32_t num_nodes;
      in_file.read(reinterpret_cast<char*>(&num_nodes), 1 * sizeof(num_nodes));
      forest[i].num_nodes = num_nodes;

      DecisionTreeNode* tree = new DecisionTreeNode[num_nodes];
      in_file.read(reinterpret_cast<char*>(tree), num_nodes * sizeof(tree[0]));
      forest[i].tree = tree;
    }
    in_file.close();
  }

  void releaseForest(DecisionTree*& forest, const int32_t num_trees) {
    for (int32_t i = 0; i < num_trees; i++) {
      SAFE_DELETE_ARR(forest[i].tree);
    }
    SAFE_DELETE_ARR(forest);
}

};  // namespace hand_detector
};  // namespace kinect_interface
