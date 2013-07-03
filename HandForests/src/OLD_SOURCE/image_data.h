//
//  image_data
//
//  Created by Jonathan Tompson on 10/01/12.
//

#ifndef UNNAMED_IMAGE_DATA_HEADER
#define UNNAMED_IMAGE_DATA_HEADER

#include "math/math_types.h"

#define GDT_MAX_DIST 2000  // Early out for points 2m away from the kinect
#define NUM_LABELS 2  // The labels must be indexed (per pixel) from 0 --> NUM_LABELS - 1
#define DT_DOWNSAMPLE 4  // 1 --> no downsample, 4 --> 16th origional size (default)
#define NUM_WL_FUNCS 2  // My depth test and the Kinect paper's depth test

struct ImageData {
  int16_t* image_data;
  uint8_t* label_data;
  uint8_t* rgb_data;
  int32_t num_images;
  int32_t im_width;
  int32_t im_height;
  char** filenames;
};

#endif
