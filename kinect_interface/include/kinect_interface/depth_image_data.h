//
//  depth_image_data.h
//
//  Created by Jonathan Tompson on 3/1/13.
//

#ifndef KINECT_INTERFACE_LOAD_DEPTH_IMAGE_HEADER
#define KINECT_INTERFACE_LOAD_DEPTH_IMAGE_HEADER

#include <string>
#include <iostream>
#include <fstream>
#include "jtil/math/math_types.h"

namespace kinect_interface {

  struct DepthImageData {
    int16_t* image_data;
    uint8_t* label_data;
    uint8_t* rgb_data;
    int32_t num_images;
    int32_t im_width;
    int32_t im_height;
    char** filenames;
  };

};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_LOAD_DEPTH_IMAGE_HEADER
