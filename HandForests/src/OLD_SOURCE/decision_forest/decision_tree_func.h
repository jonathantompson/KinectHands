//
//  decision_tree_func.h
//
//  Created by Jonathan Tompson on 7/24/12.
//

#ifndef UNNAMED_DECISION_TREE_FUNC_HEADER
#define UNNAMED_DECISION_TREE_FUNC_HEADER

// The core weak learner function:
//   For speed use a pre-processor define to avoid inline function calls.  This
//    way the compiler can make easy optimization decisions
//   - index - the current image index (could be part of a large array, ie num_images * width * height)
//   - coeff0 - int32_t U offset (to be scaled by depth in mm)
//   - coeff1 - int32_t V offset (to be scaled by depth in mm)
//   - coeff2 - int16_t threshold
//   - coeff3 - uint8_t threshold
//   - width - int32_t image width
//   - height - int32_t image height
//   - image_data - int16_t image depth data
//   - result - the boolean output result

// **************** BOTH MINE AND MICROSOFT'S WL ****************
/*
#define WL_FUNC(index, coeff0, coeff1, coeff2, coeff3, width, height, image_data, result) \
  int32_t im_index = index % (width * height); \
  int32_t u = im_index % width; \
  int32_t v = im_index / width; \
  int32_t cur_u_offset = coeff0 / image_data[index]; \
  int32_t cur_v_offset = coeff1 / image_data[index]; \
  int32_t u_offset = u + cur_u_offset; \
  int32_t v_offset = v + cur_v_offset; \
  if (u_offset < 0 || u_offset >= width || v_offset < 0 || v_offset >= height) { \
    result = false; \
  } else { \
    if (coeff3 == 0) { \
      int32_t index_offset = index + (width * cur_v_offset) + cur_u_offset; \
      result = (image_data[index_offset] - image_data[index]) >= coeff2; \
    } else { \
      int32_t index_offset1 = index + (width * cur_v_offset); \
      int32_t index_offset2 = index + cur_u_offset; \
      result = (image_data[index_offset1] - image_data[index_offset2]) >= coeff2; \
    } \
  } 
*/

// ************************ ONLY MY WL ************************
#define WL_FUNC(index, coeff0, coeff1, coeff2, coeff3, width, height, image_data, result) \
  int32_t im_index = index % (width * height); \
  int32_t u = im_index % width; \
  int32_t v = im_index / width; \
  int32_t cur_u_offset = coeff0 / image_data[index]; \
  int32_t cur_v_offset = coeff1 / image_data[index]; \
  int32_t u_offset = u + cur_u_offset; \
  int32_t v_offset = v + cur_v_offset; \
  if (u_offset < 0 || u_offset >= width || v_offset < 0 || v_offset >= height) { \
    result = false; \
  } else { \
    int32_t index_offset = index + (width * cur_v_offset) + cur_u_offset; \
    result = (image_data[index_offset] - image_data[index]) >= coeff2;\
  }

#endif  // UNNAMED_DECISION_TREE_FUNC_HEADER