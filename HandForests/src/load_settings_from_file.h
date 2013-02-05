//
//  load_settings_from_file.h
//
//  Created by Jonathan Tompson on 7/31/12.
//

#ifndef UNNAMED_LOAD_SETTINGS_FROM_FILE_HEADER
#define UNNAMED_LOAD_SETTINGS_FROM_FILE_HEADER

#include <string>
#include "math/math_types.h"

struct ProgramSettings {
  bool load_forest_from_file;
  uint32_t num_trees;
  uint32_t num_workers;
  uint32_t tree_height;
  float min_info_gain;
  uint32_t max_pixels_per_image_per_label;
  uint32_t num_wl_samples_per_node;  // Def ~ 500-5000
  uint32_t max_num_images;  // There may not be this many images
  uint32_t num_bootstrap_passes;
  uint32_t bootstrap_tree_height;
  uint32_t wl_func_type;  // 0 - both, 1 - my WL only, 2 - Kinect WL only
};

void loadSettingsFromFile(ProgramSettings* settings, std::string filename);

#endif  // UNNAMED_LOAD_SETTINGS_FROM_FILE_HEADER
