//
//  hand_detector.cpp
//
//  Created by Jonathan Tompson on 7/22/12.
// 

#include <string>
#include <stdexcept>
#include "hand_detector.h"
#include "generate_decision_tree.h"
#include "evaluate_decision_forest.h"
#include "forest_io.h"
#include "image_util.h"

using std::string;
using std::runtime_error;
using math::Float3;
using math::Int3;
using data_str::Vector;

HandDetector::HandDetector(uint32_t im_width, uint32_t im_height) {
  if (im_width % DT_DOWNSAMPLE != 0 ||
      im_height % DT_DOWNSAMPLE != 0) {
    throw runtime_error(string("HandDetector() - ERROR: downsample factor is") +
      string(" not an integer multiple!"));
  }
  
  stage2_med_filter_radius_ = 1;
  stage1_shrink_filter_radius_ = 0;
  num_trees_to_evaluate_ = STARTING_NUM_TREES_TO_EVALUATE;
  max_height_to_evaluate_ = STARTING_MAX_TREE_HEIGHT_TO_EVALUATE;
  
  width_ = im_width / DT_DOWNSAMPLE;
  height_ = im_height / DT_DOWNSAMPLE;

  labels_evaluated_ = new uint8_t[width_ * height_];
  labels_filtered_ = new uint8_t[width_ * height_];
  depth_downsampled_ = new int16_t[width_ * height_];
  pixel_queue_ = new uint32_t[width_ * height_];

  loadForest(&forest_, &num_trees_, FOREST_DATA_FILENAME);
  printf("HandDetector() - Decision Forest: %s loaded\n", 
    FOREST_DATA_FILENAME.c_str());

  num_trees_to_evaluate_ = num_trees_ < num_trees_to_evaluate_ ? num_trees_ 
                           : num_trees_to_evaluate_;

  max_height_ = 0;
  for (uint32_t i = 0; i < num_trees_; i++) {
    if (forest_[i].tree_height > max_height_) {
      max_height_ = forest_[i].tree_height;
    }
  }
  num_trees_to_evaluate_ = max_height_ < num_trees_to_evaluate_ ? max_height_ 
                           : num_trees_to_evaluate_;
}

HandDetector::~HandDetector() {
  releaseForest(&forest_, num_trees_);
  if (labels_evaluated_) { delete[] labels_evaluated_; }
  if (labels_filtered_) { delete[] labels_filtered_; }
  if (depth_downsampled_) { delete[] depth_downsampled_; }
}

void HandDetector::findHands(int16_t* depth_data, bool& rhand_found, 
    bool& lhand_found, float* rhand_uvd, float* lhand_uvd) {
  depth_ = depth_data;
  // Downsample the input image
  if (DT_DOWNSAMPLE > 1) {
    DownsampleImageWithoutNonZeroPixelsAndBackground<int16_t>(depth_downsampled_, 
      depth_, width_ * DT_DOWNSAMPLE, height_* DT_DOWNSAMPLE, DT_DOWNSAMPLE);
  } else {
    memcpy(depth_downsampled_, depth_, width_ * height_ * sizeof(depth_downsampled_[0]));
  }

  for (uint32_t i = 0; i < width_ * height_; i++) {
    if (depth_downsampled_[i] > GDT_MAX_DIST) {
      depth_downsampled_[i] = GDT_MAX_DIST + 1;
    }
  }  

  // Evaluate the decision forest
  evaluateDecisionForest(labels_evaluated_, forest_, max_height_to_evaluate_, 
    num_trees_to_evaluate_, depth_downsampled_, width_, height_);

  // Filter the results
  ShrinkFilter<uint8_t>(labels_filtered_, labels_evaluated_,
    width_, height_, stage1_shrink_filter_radius_);
  MedianLabelFilter<uint8_t, int16_t>(labels_evaluated_, labels_filtered_, 
    depth_downsampled_, width_, height_, stage2_med_filter_radius_);
  
  // Now swap the buffers
  uint8_t* tmp = labels_filtered_;
  labels_filtered_ = labels_evaluated_;
  labels_evaluated_ = tmp;
  
  // Find hands using flood fill
  floodFillLabelData(&rhand_found, &lhand_found, rhand_uvd, lhand_uvd);
}

void HandDetector::reset() {
  hands_uv_min_.resize(0);
  hands_uv_max_.resize(0);
  hands_n_pts_.resize(0);
  hands_uvd_.resize(0);
}

void HandDetector::floodFillLabelData(bool* rhand_found, bool* lhand_found, 
  float* rhand_uvd, float* lhand_uvd) {
  // re-use the labels_evaluated data instead of allocating more
  hands_uv_min_.resize(0);
  hands_uv_max_.resize(0);
  hands_n_pts_.resize(0);
  hands_uvd_.resize(0);

  pixel_on_queue_ = labels_evaluated_;
  memset(pixel_on_queue_, 0, width_ * height_ * sizeof(pixel_on_queue_[0]));
  queue_head_ = 0;
  queue_tail_ = 0; // When queue_head_ == queue_tail_ the queue is empty
  for (int v = 0; v < static_cast<int>(height_); v++) {
    for (int u = 0; u < static_cast<int>(width_); u++) {
      int cur_index = v * static_cast<int>(width_) + u;
      if (labels_filtered_[cur_index] == 1 && !pixel_on_queue_[cur_index]) {
        // We haven't already visited this pixel perform a floodfill 
        // starting from this pixel --> Potentially a new blob

        // Add the current pixel to the end of the queue
        pixel_queue_[queue_tail_] = cur_index;
        pixel_on_queue_[cur_index] = true;
        queue_tail_++;
        uint32_t n_blob_pts = 0;

        cur_uv_min_.set(width_, height_);
        cur_uv_max_.set(-1, -1);

        while (queue_head_ != queue_tail_) {
          // Take the pixel off the head
          int cur_pixel = pixel_queue_[queue_head_];
          int cur_u = cur_pixel % width_;
          int cur_v = cur_pixel / width_;
          queue_head_++;
          n_blob_pts++;

          if (cur_u < cur_uv_min_[0]) {
            cur_uv_min_[0] = cur_u;
          }
          if (cur_u > cur_uv_max_[0]) {
            cur_uv_max_[0] = cur_u;
          }
          if (cur_v < cur_uv_min_[1]) {
            cur_uv_min_[1] = cur_v;
          }
          if (cur_v > cur_uv_max_[1]) {
            cur_uv_max_[1] = cur_v;
          }

          // Process the 8 surrounding neighbours
          processNeighbour(cur_u-1, cur_v-1);
          processNeighbour(cur_u-1, cur_v);
          processNeighbour(cur_u-1, cur_v+1);
          processNeighbour(cur_u, cur_v-1);
          processNeighbour(cur_u, cur_v+1);
          processNeighbour(cur_u+1, cur_v-1);
          processNeighbour(cur_u+1, cur_v);
          processNeighbour(cur_u+1, cur_v+1);
        }
        // Finished processing the entire blob
        if (n_blob_pts >= MIN_PTS_PER_HAND_BLOB) {
          hands_uv_min_.pushBack(cur_uv_min_);
          hands_uv_max_.pushBack(cur_uv_max_);
          hands_n_pts_.pushBack(n_blob_pts);
          int center_u = ((cur_uv_max_[0] + cur_uv_min_[0]) / 2) * DT_DOWNSAMPLE +
                         DT_DOWNSAMPLE / 2;
          int center_v = ((cur_uv_max_[1] + cur_uv_min_[1]) / 2) * DT_DOWNSAMPLE +
                         DT_DOWNSAMPLE / 2;
          uint32_t index_depth_ = center_v * (width_ * DT_DOWNSAMPLE) + center_u;
          Float3 center(static_cast<float>(center_u), static_cast<float>(center_v), 
            static_cast<float>(depth_[index_depth_]));
          hands_uvd_.pushBack(center);
        }
      }
    }
  }

  // Now choose which hands should be the left and right
  // 1. Pick the two largest blobs
  uint32_t max_hand_n_pts = 0;
  uint32_t index_max_hand_n_pts = MAX_UINT32;
  uint32_t sec_max_hand_n_pts = 0;
  uint32_t index_sec_max_hand_n_pts = MAX_UINT32;
  for (uint32_t i = 0; i < hands_n_pts_.size(); i++) {
    if (hands_n_pts_[i] > max_hand_n_pts) {
      index_sec_max_hand_n_pts = index_max_hand_n_pts;
      sec_max_hand_n_pts = max_hand_n_pts;
      index_max_hand_n_pts = i;
      max_hand_n_pts = hands_n_pts_[i];
    } else if (hands_n_pts_[i] > sec_max_hand_n_pts) {
      sec_max_hand_n_pts = hands_n_pts_[i];
      index_sec_max_hand_n_pts = i;
    }
  }

  if (index_max_hand_n_pts == MAX_UINT32) {
    *lhand_found = false;
    *rhand_found = false;
    return;
  }

  // One hand found
  if (index_sec_max_hand_n_pts == MAX_UINT32) {
    if (hands_uvd_[index_max_hand_n_pts][0] < ((DT_DOWNSAMPLE * width_) / 2)) {
      *lhand_found = true;
      lhand_uvd[0] = hands_uvd_[index_max_hand_n_pts][0];
      lhand_uvd[1] = hands_uvd_[index_max_hand_n_pts][1];
      lhand_uvd[2] = hands_uvd_[index_max_hand_n_pts][2];
      *rhand_found = false;
    } else {
      *lhand_found = false;
      *rhand_found = true;
      rhand_uvd[0] = hands_uvd_[index_max_hand_n_pts][0];
      rhand_uvd[1] = hands_uvd_[index_max_hand_n_pts][1];
      rhand_uvd[2] = hands_uvd_[index_max_hand_n_pts][2];
    }
    return;
  }

  // Two hands found
  if (hands_uvd_[index_max_hand_n_pts][0] > hands_uvd_[index_sec_max_hand_n_pts][0]) {
    // Max is the right hand
    *lhand_found = true;
    lhand_uvd[0] = hands_uvd_[index_sec_max_hand_n_pts][0];
    lhand_uvd[1] = hands_uvd_[index_sec_max_hand_n_pts][1];
    lhand_uvd[2] = hands_uvd_[index_sec_max_hand_n_pts][2];
    *rhand_found = true;
    rhand_uvd[0] = hands_uvd_[index_max_hand_n_pts][0];
    rhand_uvd[1] = hands_uvd_[index_max_hand_n_pts][1];
    rhand_uvd[2] = hands_uvd_[index_max_hand_n_pts][2];
  } else {
    *lhand_found = true;
    lhand_uvd[0] = hands_uvd_[index_max_hand_n_pts][0];
    lhand_uvd[1] = hands_uvd_[index_max_hand_n_pts][1];
    lhand_uvd[2] = hands_uvd_[index_max_hand_n_pts][2];
    *rhand_found = true;
    rhand_uvd[0] = hands_uvd_[index_sec_max_hand_n_pts][0];
    rhand_uvd[1] = hands_uvd_[index_sec_max_hand_n_pts][1];
    rhand_uvd[2] = hands_uvd_[index_sec_max_hand_n_pts][2];
  }

}

inline int mod(int x, int m) {  // Handle negative modulus
  int r = x%m;
  return r<0 ? r+m : r;
}

void HandDetector::processNeighbour(int u, int v) {
  if (u < static_cast<int>(width_) && u >=0 && 
      v < static_cast<int>(height_) && v >= 0) {
    int cur_index = u + v * width_;
    if ((labels_filtered_[cur_index] == 1) && (pixel_on_queue_[cur_index] == 0)) {
      // Add the pixel to the back of the queue
      pixel_queue_[queue_tail_] = cur_index;
      pixel_on_queue_[cur_index] = 1;
      queue_tail_++;
    }
  }
}
