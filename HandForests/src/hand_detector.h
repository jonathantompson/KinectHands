//
//  hand_detector.h
//
//  Created by Jonathan Tompson on 7/22/12.
//

#ifndef UNNAMED_HAND_DETECTOR_HEADER
#define UNNAMED_HAND_DETECTOR_HEADER

#include <string>
#include "XnVNite.h"
#include "math/math_types.h"
#include "data_str/vector.h"

#define FOREST_DATA_FILENAME std::string("./forest_data.bin")
#define MIN_PTS_PER_HAND_BLOB 100
#define STARTING_NUM_TREES_TO_EVALUATE 4
#define STARTING_MAX_TREE_HEIGHT_TO_EVALUATE 22

struct DecisionTree;

class HandDetector {
public:
  HandDetector(uint32_t im_width, uint32_t im_height);
  ~HandDetector();

  void findHands(int16_t* depth_data, bool* rhand_found, bool* lhand_found, 
    XnPoint3D* rhand_uvd, XnPoint3D* lhand_uvd);
  
  void reset();
  
  inline int16_t* getDepthImDownsampled() { return depth_downsampled_; }
  inline uint8_t* getLabelIm() { return labels_filtered_; }
  inline uint32_t width() { return width_; }
  inline uint32_t height() { return height_; }
  inline int32_t stage2_med_filter_radius() { return stage2_med_filter_radius_; }
  inline int32_t stage1_shrink_filter_radius() { return stage1_shrink_filter_radius_; }
  inline void stage2_med_filter_radius(int32_t val) { stage2_med_filter_radius_ = val; }
  inline void stage1_shrink_filter_radius(int32_t val) { stage1_shrink_filter_radius_ = val; }
  inline uint32_t num_trees_to_evaluate() { return num_trees_to_evaluate_; }
  inline void num_trees_to_evaluate(uint32_t val) { num_trees_to_evaluate_ = val; }
  inline uint32_t num_trees() { return num_trees_; }
  inline uint32_t max_height_to_evaluate() { return max_height_to_evaluate_; }
  inline void max_height_to_evaluate(uint32_t val) { max_height_to_evaluate_ = val; }
  inline uint32_t max_height() { return max_height_; }

  void drawOBB();  // draw a bounding rectangle where the hands are

private:
  DecisionTree* forest_;
  uint32_t num_trees_;
  uint32_t max_height_;
  uint8_t* labels_evaluated_;
  uint8_t* labels_filtered_;
  int16_t* depth_;  // Not owned here!
  int16_t* depth_downsampled_;
  uint32_t width_;
  uint32_t height_;
  int32_t stage2_med_filter_radius_;  // Final median filter after shrink filter
  int32_t stage1_shrink_filter_radius_;
  uint32_t num_trees_to_evaluate_;
  uint32_t max_height_to_evaluate_;
  uint32_t* pixel_queue_;
  data_str::Vector<math::Int2> hands_uv_min_;  // In downsampled UV space
  data_str::Vector<math::Int2> hands_uv_max_;
  data_str::Vector<math::Float3> hands_uvd_;  // In 640 x 480 space
  data_str::Vector<uint32_t> hands_n_pts_;
  math::Int2 cur_uv_min_;
  math::Int2 cur_uv_max_;
  uint32_t queue_head_;
  uint32_t queue_tail_;
  uint8_t* pixel_on_queue_;  // Not allocated but shared with labels_evaluated_

  void processNeighbour(int u, int v);

  void floodFillLabelData(bool* rhand_found, bool* lhand_found, 
    XnPoint3D* rhand_uvd, XnPoint3D* lhand_uvd);
};

#endif