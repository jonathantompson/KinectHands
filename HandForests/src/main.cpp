//
//  main.cpp
//
//  Created by Jonathan Tompson on 7/20/12.
// 

#if defined(WIN32) || defined(_WIN32)
  #include <Windows.h>
#endif
#include <signal.h>
#include <stdexcept>
#include <string>
#include <iostream>
#include <thread>
#include "jtil/math/math_types.h"
#include "kinect_interface/depth_images_io.h"
#include "kinect_interface/hand_detector/evaluate_decision_forest.h"
#include "kinect_interface/hand_detector/generate_decision_tree.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"
#include "kinect_interface/hand_detector/common_tree_funcs.h"
#include "kinect_interface/hand_detector/forest_io.h"
#include "jtil/string_util/string_util.h"
#include "jtil/image_util/image_util.h"
#include "jtil/clk/clk.h"
#include "jtil/threading/callback.h"
#include "jtil/threading/thread.h"
#include "load_settings_from_file.h"
#include "jtil/debug_util/debug_util.h"  // Must come last in .cpp that includes main

using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using namespace jtil::threading;
using namespace kinect_interface;
using namespace kinect_interface::hand_detector;

typedef enum {
  CorrectLabeling = 0,
  ForrestLabeling = 1,
  NumResults = 2
} CurResult;

#if defined(__APPLE__)
  #define IMAGE_DIRECTORY string("../../../../../../hand_depth_data/")
  #define FOREST_DATA_FILENAME string("../../../../../../forest_data.bin")
  #define PROGRAM_SETTINGS_FILENAME string("../../../../../../program_settings.csv")
#else
  #define IMAGE_DIRECTORY string("./data/hand_depth_data_processed_for_DF/")
  #define FOREST_DATA_FILENAME string("./forest_data.bin")
  #define PROGRAM_SETTINGS_FILENAME string("./hand_forests_settings.csv")
#endif

#define SAFE_DELETE(x) do { if (x != NULL) { delete x; x = NULL; } } while (0); 
#define SAFE_DELETE_ARR(x) do { if (x != NULL) { delete[] x; x = NULL; } } while (0); 

// DATA VARIABLES
DepthImagesIO* images_io = NULL;
const bool load_processed_images = true;  // Don't change this!
const int32_t file_stride = 200;  // Better to use ALL the training data (it's already been decimated)
const float frac_test_data = 0.05f;  // 5% of data files will be test data, def > 0.05
DepthImageData* training_data = NULL;
DepthImageData* test_data = NULL;
uint8_t* label_data_evaluated = NULL;
uint8_t* label_data_filtered = NULL;
bool filter_results = false;
uint32_t filter_results_radius = 2;
int32_t total_num_images;

// DECISION TREE VARIABLES
ProgramSettings prog_settings;
DecisionTree* forest = NULL;
TrainingSettings* settings = NULL;
WLSet wl_set;
const uint32_t num_threshold_vals = 41;
int16_t threshold_vals[num_threshold_vals] = 
{ 0,
  1,  2,  3,  4,  5,  6,  8,  10,  12,  15,  25,  50,  75,  100,  250,  500, 
  750,  1000,  1250,  1500,
 -1, -2, -3, -4, -5, -6, -8, -10, -12, -15, -25, -50, -75, -100, -250, -500, 
  750, -1000, -1250, -1500 };
const int32_t num_uv_offset_vals = 35;
// offset is divided by depth!, so 1000 is 1 pixel offset at 1m
int32_t uv_offset_vals[num_uv_offset_vals] = 
{ 0, 
  398,  631,  1000,  1585,  2512,  3981,  6310,  10000,  15850,  25120,  39810, 
  63100,  100000,  158500,  251200,  398100,  631000, 
 -398, -631, -1000, -1585, -2512, -3981, -6310, -10000, -15850, -25120, -39810,
 -63100, -100000, -158500, -251200, -398100, -631000};
uint32_t num_trees_to_evaluate;
uint32_t max_eval_height = 30;

void shutdown() {
  SAFE_DELETE(images_io);
  SAFE_DELETE_ARR(label_data_evaluated);
  SAFE_DELETE_ARR(label_data_filtered);
  if (wl_set.wl_coeffs0 != NULL) {
    SAFE_DELETE_ARR(wl_set.wl_coeffs0);
    SAFE_DELETE_ARR(wl_set.wl_coeffs1);
    SAFE_DELETE_ARR(wl_set.wl_coeffs2);
    SAFE_DELETE_ARR(wl_set.wl_coeffs_sizes);
  }
  if (test_data) {
    DepthImagesIO::releaseImages(test_data);
  }
  if (training_data) {
    DepthImagesIO::releaseImages(training_data);
  }
  releaseForest(forest, prog_settings.num_trees);
}

int main(int argc, char *argv[]) { 
  std::cout << "image directory is " << IMAGE_DIRECTORY << std::endl;
  static_cast<void>(argc);  // Get rid of unused variable warning
  static_cast<void>(argv);
#ifdef _DEBUG
  jtil::debug::EnableMemoryLeakChecks();
  //jtil::debug::SetBreakPointOnAlocation(587);
#endif
  try {
    if (DT_DOWNSAMPLE < 1) {
      cout << "ERROR: DT_DOWNSAMPLE < 1!" << endl;
      return -1;
    }
    if (argc <= 1) {
      loadSettingsFromFile(&prog_settings, PROGRAM_SETTINGS_FILENAME);
    } else if (argc == 2) {
      loadSettingsFromFile(&prog_settings, string(argv[1]));
    } else {
      std::cout << "Usage: KinectHands <filename>    <-- filename is optional" << endl;
    }
    images_io = new DepthImagesIO();
    cout << "loading image data from file..." << endl;
    images_io->LoadDepthImagesFromDirectoryForDT(IMAGE_DIRECTORY, training_data, 
      test_data, frac_test_data, file_stride);
    total_num_images = test_data->num_images + training_data->num_images; 
    
    if (!prog_settings.load_forest_from_file) {
      srand(0);
      
      // Allocate space for our decision trees
      uint32_t total_num_trees = (prog_settings.num_bootstrap_passes + 1) * 
        prog_settings.num_trees;
      forest = new DecisionTree[total_num_trees];
      
      // Allocate space for the WLInput data struct
      wl_set.wl_coeffs0 = new int32_t[num_uv_offset_vals];
      wl_set.wl_coeffs1 = new int32_t[num_uv_offset_vals];
      switch (prog_settings.wl_func_type) {
      case 0:
        wl_set.wl_funcs = new uint8_t[NUM_WL_FUNCS];
        break;
      case 1:
        wl_set.wl_funcs = new uint8_t[1];
        break;
      case 2:
        wl_set.wl_funcs = new uint8_t[2];
        break;
      }

      // Now fill the coefficient arrays
      cout << endl << "wl_coeffs0 and 1 coeffs = [";
      for (int32_t i = 0; i < num_uv_offset_vals; i++) {
        wl_set.wl_coeffs0[i] = uv_offset_vals[i];
        wl_set.wl_coeffs1[i] = uv_offset_vals[i];
        cout << wl_set.wl_coeffs1[i];
        if (i != num_uv_offset_vals - 1) {
          cout << ", ";
        }
      }
      cout << "]" << endl << endl;
      wl_set.wl_coeffs2 = new int16_t[num_threshold_vals];
      cout << endl << "wl_coeffs2 = [";
      for (uint32_t i = 0; i < num_threshold_vals; i++) {
        wl_set.wl_coeffs2[i] = threshold_vals[i];
        cout << wl_set.wl_coeffs2[i];
        if (i != num_threshold_vals - 1) {
          cout << ", ";
        }
      }
      cout << "]" << endl << endl << "wl_funcs = [";
      switch (prog_settings.wl_func_type) {
      case 0:
        for (uint8_t i = 0; i < NUM_WL_FUNCS; i++) {
          wl_set.wl_funcs[i] = i;
          cout << static_cast<int>(wl_set.wl_funcs[i]);
          if (i != NUM_WL_FUNCS - 1) {
            cout << ", ";
          }
        }
        break;
      case 1:
        wl_set.wl_funcs[0] = 0;
        cout << static_cast<int>(wl_set.wl_funcs[0]);
        break;
      case 2:
        wl_set.wl_funcs[0] = 1;
        cout << static_cast<int>(wl_set.wl_funcs[0]);
        break;
      }
      cout << "]" << endl << endl;
      
      wl_set.wl_coeffs_sizes = new uint32_t[4];
      wl_set.wl_coeffs_sizes[0] = num_uv_offset_vals;
      wl_set.wl_coeffs_sizes[1] = num_uv_offset_vals;
      wl_set.wl_coeffs_sizes[2] = num_threshold_vals;
      switch (prog_settings.wl_func_type) {
      case 0:
        wl_set.wl_coeffs_sizes[3] = NUM_WL_FUNCS;
        break;
      case 1:
      case 2:
        wl_set.wl_coeffs_sizes[3] = 1;
        break;
      }
      wl_set.num_samples_per_node = prog_settings.num_wl_samples_per_node;
      
      settings = new TrainingSettings[total_num_trees];
      for (uint32_t i = 0; i < total_num_trees; i++) {
        if (prog_settings.max_num_images > static_cast<uint32_t>(training_data->num_images)) {
          settings[i].num_im_to_consider = training_data->num_images;
        } else {
          settings[i].num_im_to_consider = prog_settings.max_num_images;
        }
        settings[i].min_info_gain = prog_settings.min_info_gain;
        settings[i].tree_height = prog_settings.tree_height;
        settings[i].max_pix_per_im_per_label = prog_settings.max_pixels_per_image_per_label;
        settings[i].seed = static_cast<unsigned int>(rand());
        settings[i].dt_index = i;
      }
      for (uint32_t i = prog_settings.num_trees; i < total_num_trees; i++) {
        settings[i].tree_height = prog_settings.bootstrap_tree_height;
      }
     
      std::thread* threads = new std::thread[prog_settings.num_workers];
      
      kinect_interface::hand_detector::GenerateDecisionTree genTree;
      for (uint32_t i = 0; i < prog_settings.num_trees; i += prog_settings.num_workers) {
        for (uint32_t j = 0; j < prog_settings.num_workers && (i + j) < prog_settings.num_trees; j++) {
          uint32_t cur_tree_index = i + j;
          // Hack to fix linux version when statically linking --> it throws operation not permitted
          threads[j] = std::thread(GenerateDecisionTree::generateDecisionTree,
            &forest[cur_tree_index], training_data, &wl_set, &settings[cur_tree_index]);
        }
        for (uint32_t j = 0; j < prog_settings.num_workers && (i + j) < prog_settings.num_trees; j++) {
          threads[j].join();
        }
      }
      
      cout << endl << "Saving Forest to file..." << endl << endl;
      prog_settings.num_trees = total_num_trees;  // Just pretend we have this many trees
      saveForest(forest, total_num_trees, FOREST_DATA_FILENAME);
    } else {
      wl_set.wl_coeffs0 = NULL;
      wl_set.wl_coeffs1 = NULL;
      wl_set.wl_coeffs2 = NULL;
      wl_set.wl_funcs = NULL;
      wl_set.wl_coeffs_sizes = NULL;
      cout << "Loading Forest from file..." << endl;
      int32_t ntrees;
      loadForest(forest, ntrees, FOREST_DATA_FILENAME);
      prog_settings.num_trees = (uint32_t)ntrees;
    }
    
    const int32_t evaluate_median_radius = 2;
    if (test_data->num_images > 0) {
      cout << "Evaluating results..." << endl;
      for (uint32_t i = 0; i < prog_settings.num_trees; i++) {
        float error = evaluateDecisionForestError(test_data, forest, i+1, false, 
          evaluate_median_radius);
        cout << i+1 << " trees --> error on test set = " << error*100 << endl;
        error = evaluateDecisionForestError(test_data, forest, i+1, true, 
          evaluate_median_radius);
        cout << i+1 << " trees --> error on test set using " << evaluate_median_radius;
        cout << " rad median filter = " << error*100 << endl;
      }
    }
  } catch(runtime_error e) {
    printf("std::runtime_error caught!:\n");
    printf("  %s\n", e.what());
#if defined(WIN32) || defined(_WIN32)
    system("pause");
#endif
  } catch(std::bad_alloc e) {
    printf("std::bad_alloc caught! --> Likely not enough memory!:\n");
#if defined(WIN32) || defined(_WIN32)
    system("pause");
#endif
  }
  shutdown();
#if defined(WIN32) || defined(_WIN32)
  system("pause");
#endif
  return 0;
}
