//
//  hand_detector.h
//
//  Created by Jonathan Tompson on 7/22/12.
//

#ifndef UNNAMED_HAND_DETECTOR_HEADER
#define UNNAMED_HAND_DETECTOR_HEADER

#include <string>
#include <mutex>
#include <condition_variable>
#include "math/math_types.h"
#include "data_str/vector.h"
#include "threading/callback.h"

#define FOREST_DATA_FILENAME std::string("./forest_data.bin")
#define HD_MIN_PTS_PER_HAND_BLOB 50  // EDIT: 2/13 (prev 100)
#define HD_STARTING_NUM_TREES_TO_EVALUATE 8
#define HD_STARTING_MAX_TREE_HEIGHT_TO_EVALUATE 30
#define HD_STARTING_SHRINK_FILT_RAD 0
#define HD_STARTING_MED_FILT_RAD 3  // EDIT: 2/13 (prev 1)
#define HD_STARTING_GROW_FILT_RAD 2
#define HD_NUM_WORKER_THREADS 7

// Post processing variables
#define HD_SMALL_HAND_RADIUS 10.0f
#define HD_DISCONT_FILT_RAD 3
#define HD_DISCONT_FILT_DEPTH_THRESH 25
#define HD_SMALL_HAND_RADIUS_MIN_UV 1
#define HD_N_PTS_FILL_KERNEL 16
#define HD_HAND_RADIUS 150.0f
#define HD_BACKGROUND_THRESH 100.0f  // For hand flood fill
#define HD_BACKGROUND_THRESH_SQ (HD_BACKGROUND_THRESH * HD_BACKGROUND_THRESH)
#define HD_FILL_COARSE_RADIUS 5000  // This value is divided by depth in mm!
#define HD_FILL_FINE_RADIUS 1 

struct DecisionTree;
class Clock;

typedef enum {
  HDUpconvert,
  HDFloodfill
} HDLabelMethod;

namespace threading { class ThreadPool; }

class HandDetector {
public:
  HandDetector(const uint32_t im_width, const uint32_t im_height,
    const std::string filename = FOREST_DATA_FILENAME);
  ~HandDetector();

  void findHands(const int16_t* depth_data, bool& rhand_found, bool& lhand_found, 
    float* rhand_uvd, float* lhand_uvd);
  bool findHandLabels(const int16_t* depth_in, const float* xyz, 
    const HDLabelMethod method, uint8_t* label_out);
  
  void reset();
  
  inline int16_t* getDepthImDownsampled() { return depth_downsampled_; }
  inline uint8_t* getLabelIm() { return labels_filtered_; }
  inline uint32_t down_width() { return down_width_; }
  inline uint32_t down_height() { return down_height_; }
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

  inline data_str::Vector<math::Float3>& hands_uvd() { return hands_uvd_; }
  inline data_str::Vector<math::Int2>& hands_uv_min() { return hands_uv_min_; }
  inline data_str::Vector<math::Int2>& hands_uv_max() { return hands_uv_max_; }

private:
  DecisionTree* forest_;
  int32_t num_trees_;
  int32_t max_height_;
  uint8_t* labels_evaluated_;
  uint8_t* labels_filtered_;
  const int16_t* depth_;  // Not owned here!
  int16_t* depth_downsampled_;
  int32_t down_width_;
  int32_t down_height_;
  int32_t src_width_;
  int32_t src_height_;
  int32_t stage2_med_filter_radius_;  // Final median filter after shrink filter
  int32_t stage3_grow_filter_radius_;
  int32_t stage1_shrink_filter_radius_;
  int32_t num_trees_to_evaluate_;
  int32_t max_height_to_evaluate_;
  uint32_t* pixel_queue_;
  data_str::Vector<math::Int2> hands_uv_min_;  // In downsampled UV space
  data_str::Vector<math::Int2> hands_uv_max_;
  data_str::Vector<math::Float3> hands_uvd_;  // In 640 x 480 space
  data_str::Vector<uint32_t> hands_n_pts_;
  math::Int2 cur_uv_min_;
  math::Int2 cur_uv_max_;
  uint32_t queue_head_;
  uint32_t queue_tail_;
  uint8_t* pixel_on_queue_;

  // Multithreading
  threading::ThreadPool* tp_;
  uint32_t threads_finished_;
  std::mutex thread_update_lock_;
  std::condition_variable not_finished_;
  threading::Callback<void>** thread_cbs_;

  static const float floodFillKernel_[HD_N_PTS_FILL_KERNEL][2]; 

  void processNeighbour(int u, int v);

  // if lhand is NULL then it'll just find one hand
  // floodFillLabelData --> Find's hand points by doing bloob detection
  void floodFillLabelData(bool* rhand_found, float* rhand_uvd, 
    bool* lhand_found = NULL, float* lhand_uvd = NULL);

  void createLabels();

  void evaluateForest();  // Sets up the pixel work queue and fire's off threads
  void evaluateForestPixelRange(const uint32_t istart, const uint32_t iend);
  void evaluateForestPixel(const uint32_t index);

  // findHandLabelsFloodFill --> Performs a floodfill from the hand point
  // which is sensitive to depth discontinuities.
  void findHandLabelsFloodFill(const float* pt_hand_uvd, const float* xyz, 
    uint8_t* label);
  bool searchUVForMinHandPoint(float* pt_hand_uvd, float* min_pt, 
    const int radius, const float* uvd);
  void clipOnScreen(int& uMin, int& uMax, int& vMin, int& vMax, 
    float* pt_hand_uvd, int radius);
  void processNeighbour(const int* nieghbourPtUV, const int curPtIndex, 
    const float* ptHand, const float* xyz);

  // findHand --> Just find A hand (will find the biggest of the hand blobs)
  void findHand(const int16_t* depth_data, bool& hand_found, float* hand_uvd);
};

#endif