#include <string>
#include <stdexcept>
#include <iostream>
#include "kinect_interface/kinect_interface.h"
#include "kinect_interface/hand_detector/hand_detector.h"
#include "kinect_interface/hand_detector/forest_io.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"
#include "kinect_interface/hand_detector/evaluate_decision_forest.h"
#include "jtil/image_util/image_util.h"
#include "jtil/file_io/file_io.h"
#include "jtil/threading/thread_pool.h"
#include "jtorch/jtorch.h"
#include "jtil/data_str/vector_managed.h"

using std::string;
using std::runtime_error;
using jtil::data_str::Vector;
using jtil::data_str::VectorManaged;
using jtil::threading::Callback;
using namespace jtil::math;
using namespace jtil::threading;
using namespace jtil::image_util;

#define SAFE_FREE(x) do { if (x != NULL) { free(x); x = NULL; } } while (0); 
#define SAFE_DELETE(x) do { if (x != NULL) { delete x; x = NULL; } } while (0); 
#define SAFE_DELETE_ARR(x) do { if (x != NULL) { delete[] x; x = NULL; } } while (0); 

namespace kinect_interface {
namespace hand_detector {
  const float c_r = HD_FILL_COARSE_RADIUS;
  const float o_r = HD_FILL_OUSIDE_RADIUS;
  const float f_r = HD_FILL_FINE_RADIUS;
  const float HandDetector::floodFillKernel_[HD_N_PTS_FILL_KERNEL][2] = 
    {{-f_r, -f_r}, {-f_r, 0}, {-f_r, +f_r}, {0, +f_r}, {+f_r, +f_r}, {+f_r, 0}, 
     {+f_r, -f_r}, {0, -f_r}, 
     {-c_r, -c_r}, {-c_r, 0}, {-c_r, +c_r}, {0, +c_r}, {+c_r, +c_r}, {+c_r, 0},
     {+c_r, -c_r}, {0, -c_r},
     {-o_r, -o_r}, {-o_r, 0}, {-o_r, +o_r}, {0, +o_r}, {+o_r, +o_r}, {+o_r, 0},
     {+o_r, -o_r}, {0, -o_r}};

  HandDetector::HandDetector(ThreadPool* tp) {
    stage2_med_filter_radius_ = HD_STARTING_MED_FILT_RAD;
    stage3_grow_filter_radius_ = HD_STARTING_GROW_FILT_RAD;
    stage1_shrink_filter_radius_ = HD_STARTING_SHRINK_FILT_RAD;
    num_trees_to_evaluate_ = HD_STARTING_NUM_TREES_TO_EVALUATE;
    max_height_to_evaluate_ = HD_STARTING_MAX_TREE_HEIGHT_TO_EVALUATE;

    src_width_ = 0;
    src_height_ = 0;
    down_width_ = 0;
    down_height_ = 0;

    labels_evaluated_ = NULL;
    labels_filtered_ = NULL;
    labels_temp_ = NULL;
    labels_temp2_ = NULL;
    depth_downsampled_ = NULL;
    pixel_queue_ = NULL;
    pixel_on_queue_ = NULL;

    forest_ = NULL;
    num_trees_ = 0;

    tp_ = tp;
    thread_cbs_ = NULL;
  }

  HandDetector::~HandDetector() {
    releaseForest(forest_, num_trees_);
    SAFE_DELETE_ARR(labels_evaluated_);
    SAFE_DELETE_ARR(labels_filtered_);
    SAFE_DELETE_ARR(labels_temp_);
    SAFE_DELETE_ARR(labels_temp2_);
    SAFE_DELETE_ARR(depth_downsampled_);
    SAFE_DELETE(thread_cbs_);
    SAFE_DELETE_ARR(thread_cbs_);
    SAFE_DELETE_ARR(pixel_on_queue_);
    SAFE_DELETE_ARR(pixel_queue_);
  }

  void HandDetector::init(const uint32_t im_width, const uint32_t im_height,
      const std::string filename) {
    if (im_width % DT_DOWNSAMPLE != 0 || im_height % DT_DOWNSAMPLE != 0) {
      throw runtime_error(string("HandDetector() - ERROR: downsample factor") +
        string(" is not an integer multiple!"));
    }

    src_width_ = im_width;
    src_height_ = im_height;
    down_width_ = im_width / DT_DOWNSAMPLE;
    down_height_ = im_height / DT_DOWNSAMPLE;

    labels_evaluated_ = new uint8_t[src_width_ * src_height_];
    labels_filtered_ = new uint8_t[src_width_ * src_height_];
    labels_temp_ = new uint8_t[src_width_ * src_height_];
    labels_temp2_ = new uint8_t[src_width_ * src_height_];
    depth_downsampled_ = new int16_t[down_width_ * down_height_];
    pixel_queue_ = new uint32_t[src_width_ * src_height_];
    pixel_on_queue_ = new uint8_t[src_width_ * src_height_];

    loadForest(forest_, num_trees_, filename);
    std::cout << "HandDetector() - Decision Forest: " << filename << " loaded";
    std::cout << std::endl;

    num_trees_to_evaluate_ = num_trees_ < num_trees_to_evaluate_ ? num_trees_ 
                             : num_trees_to_evaluate_;

    max_height_ = 0;
    for (int32_t i = 0; i < num_trees_; i++) {
      if (forest_[i].tree_height > (uint32_t)max_height_) {
        max_height_ = forest_[i].tree_height;
      }
    }
    max_height_to_evaluate_ = max_height_ < max_height_to_evaluate_ ? max_height_ 
                             : max_height_to_evaluate_;

    const int num_threads = tp_->num_workers();
    thread_cbs_ = new VectorManaged<Callback<void>*>(num_threads);

    // Figure out the load balance accross worker threads.  This thread will act
    // as a worker thread as well.
    uint32_t n_pixels = down_width_*down_height_;
    uint32_t n_pixels_per_thread = 1 + n_pixels / num_threads;
    for (int32_t i = 0; i < num_threads; i++) {
      uint32_t start = i * n_pixels_per_thread;
      uint32_t end = std::min<uint32_t>(((i + 1) * n_pixels_per_thread) - 1,
        n_pixels - 1);
      thread_cbs_->pushBack(MakeCallableMany(&HandDetector::evaluateForestPixelRange, 
        this, start, end));
    }
  }

  bool HandDetector::findHandLabels(const int16_t* depth_in, const float* xyz, 
    const HDLabelMethod method, uint8_t* label_out) {
    const int32_t disc_filt_rad = 2;
    int16_t cur_depth_min;
    int16_t cur_depth_max;
    uint32_t index = 0;
    switch (method) {
    case HDUpconvert:
      createLabels(depth_in);
      UpsampleNoFiltering<uint8_t>(labels_temp_, labels_evaluated_, 
        down_width_, down_height_, DT_DOWNSAMPLE);
      ShrinkFilter<uint8_t>(labels_filtered_, labels_temp_,
        src_width_, src_height_, 2);
      for (int32_t v = 0; v < (int32_t)src_height_; v++) {
        for (int32_t u = 0; u < (int32_t)src_width_; u++) {
          cur_depth_min = max_depth;
          cur_depth_max = 0;
          if (labels_filtered_[index] == 1 && depth_in[index] > 0 || 
            depth_in[index] < max_depth) {
            for (int32_t v_off = v - disc_filt_rad; v_off <= v + disc_filt_rad; v_off++) {
              for (int32_t u_off = u - disc_filt_rad; u_off <= u + disc_filt_rad; u_off++) {   
                int32_t ioff = v_off * src_width_ + u_off;
                if (depth_in[ioff] <= 0 || depth_in[ioff] >= max_depth) {
                    cur_depth_max = max_depth;
                    cur_depth_min = 0;
                } else {
                  cur_depth_max = std::max<int16_t>(cur_depth_max, depth_in[ioff]);
                  cur_depth_min = std::min<int16_t>(cur_depth_min, depth_in[ioff]);        
                }
              }
            }
            if ((cur_depth_max - cur_depth_min) > 2 * HD_DISCONT_FILT_DEPTH_THRESH) {
              labels_filtered_[index] = 0;
            }
          } else {
            labels_filtered_[index] = 0;
          }
          index++;
        }
      }


      GrowFilterDepthThreshold(label_out, labels_filtered_, depth_in,
        src_width_, src_height_, 4);
      return true;

    case HDFloodfill:
      // Find the center hand point
      bool hand_found;
      float hand_uvd[3];
      findHand(depth_in, hand_found, hand_uvd);
      if (!hand_found) { 
        return false;
      }

      // Now start a flood fill from this point
      memset(label_out, 0, sizeof(label_out[0]) * src_width_ * src_height_);
      findHandLabelsFloodFill(hand_uvd, xyz, label_out);

      return true;
    default:
      throw std::wruntime_error("HandDetector::findHandLabels() - ERROR: "
        "Unsupported labeling method!");
    }
  }

  void HandDetector::findHands(const int16_t* depth_data, bool& rhand_found, 
    bool& lhand_found, float* rhand_uvd, float* lhand_uvd) {
    depth_ = depth_data;
    createLabels(depth_data);
    // Copy labels labels_evaluated_ since the filter methods may destroy
    // the source image.
    memcpy(labels_temp_, labels_evaluated_, down_width_*down_height_*
      sizeof(labels_temp_[0]));
    filterLabels(labels_filtered_, labels_temp_, labels_temp2_,
      depth_downsampled_, down_width_, down_height_);
    // Result is now in labels_filtered_

    // Find hands using flood fill
    floodFillLabelData(&rhand_found, rhand_uvd, &lhand_found, lhand_uvd);
  }

  void HandDetector::findHand(const int16_t* depth_data, bool& hand_found,
    float* hand_uvd) {
    createLabels(depth_data);
    // Copy labels labels_evaluated_ since the filter methods may destroy
    // the source image.
    memcpy(labels_temp_, labels_evaluated_, down_width_*down_height_*
      sizeof(labels_temp_[0]));
    filterLabels(labels_filtered_, labels_temp_, labels_temp2_,
      depth_downsampled_, down_width_, down_height_);
    // Result is now in labels_filtered_

    // Find hands using flood fill
    floodFillLabelData(&hand_found, hand_uvd, NULL, NULL);
  }

  void HandDetector::evaluateForest(const int16_t* depth_data) {
    depth_ = depth_data;
    // Downsample the input image
    if (DT_DOWNSAMPLE > 1) {
      DownsampleImageWithoutNonZeroPixelsAndBackground<int16_t>(
        depth_downsampled_, depth_, src_width_, src_height_, DT_DOWNSAMPLE,
        max_depth);
      for (int32_t i = 0; i < down_height_*down_width_; i++) {
        if (depth_downsampled_[i] > max_depth) {
          depth_downsampled_[i] = max_depth+1;
        }
      }
    } else {
      memcpy(depth_downsampled_, depth_, 
        down_width_ * down_height_ * sizeof(depth_downsampled_[0]));
      for (int32_t i = 0; i < down_width_ * down_height_; i++) {
        if (depth_downsampled_[i] > max_depth ||
          depth_downsampled_[i] == 0) {
            depth_downsampled_[i] = max_depth + 1;
        }
      }  
    }

    // Evaluate the decision forest
    // Single threaded
    //evaluateDecisionForest(labels_evaluated_, forest_, max_height_to_evaluate_, 
    //  num_trees_to_evaluate_, depth_downsampled_, down_width_, down_height_);
    // Multi threaded
    evaluateForestMultithreaded();
  }

  void HandDetector::createLabels(const int16_t* depth_data) {
    evaluateForest(depth_data);
  }

  void HandDetector::filterLabels(uint8_t*& dst, uint8_t*& src, uint8_t*& tmp,
    int16_t* depth, const int32_t w, const int32_t h) {
    ShrinkFilter<uint8_t>(tmp, src, w, h, 
      stage1_shrink_filter_radius_);
    MedianLabelFilter<uint8_t, int16_t>(dst, tmp, depth, w, h, 
      stage2_med_filter_radius_, NUM_LABELS, max_depth);
    GrowFilter<uint8_t>(tmp, dst, w, h, stage3_grow_filter_radius_);
    // Now swap the buffers
    uint8_t* tmp_local = tmp;
    tmp = dst;
    dst = tmp_local;
  }

  void HandDetector::filterLabelsThreshold(uint8_t*& dst, uint8_t*& src, 
    uint8_t*& tmp, int16_t* depth, const int32_t w, const int32_t h) {
    memcpy(tmp, src, w * h * sizeof(tmp[0]));
    GrowFilterDepthThreshold(dst, tmp, depth, w, h, 
      stage3_grow_filter_radius_);
  }


  void HandDetector::GrowFilterDepthThreshold(uint8_t* labels_dst, 
    uint8_t* labels_src, const int16_t* depth, const int32_t w, 
    const int32_t h, const int32_t rad) {
    memcpy(labels_dst, labels_src, w * h * sizeof(labels_dst[0]));
    // Grow horizontally
    int32_t index = 0;
    for (int32_t v = 0; v < h; v++) {
      for (int32_t u = 0; u < w; u++) {
        if (labels_dst[index] == 1) {
          int16_t cur_depth = depth[index];
          for (int32_t u_offset = std::max<int32_t>(u - rad, 0); 
            u_offset <= u + rad && u_offset < w; u_offset++) {
            int32_t offset = v * w + u_offset;
            int16_t delta_depth = cur_depth - depth[offset];
            if (delta_depth < 0) {
              delta_depth = -delta_depth; 
            }
            if (delta_depth < HD_BACKGROUND_THRESH_GROW) {
              labels_src[offset] = 1;
            }
          }
        }
        index++;
      }
    }
    // Grow vertically
    index = 0;
    for (int32_t v = 0; v < h; v++) {
      for (int32_t u = 0; u < w; u++) {
        if (labels_src[index] == 1) {
          int16_t cur_depth = depth[index];
          for (int32_t v_offset = std::max<int32_t>(v - rad, 0); 
            v_offset <= v + rad && v_offset < h; v_offset++) {
            int32_t offset = v_offset * w + u;
            int16_t delta_depth = cur_depth - depth[offset];
            if (delta_depth < 0) {
              delta_depth = -delta_depth; 
            }
            if (delta_depth < HD_BACKGROUND_THRESH_GROW) {
              labels_dst[offset] = 1;
            }
          }
        }
        index++;
      }
    }
  };

  void HandDetector::evaluateForestMultithreaded() {
    threads_finished_ = 0;
    for (uint32_t i = 0; i < thread_cbs_->size(); i++) {
      tp_->addTask((*thread_cbs_)[i]);
    }

    // Wait until the other threads are done
    std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
    while (threads_finished_ != thread_cbs_->size()) {
      not_finished_.wait(ul);
    }
    ul.unlock();  // Release lock
  }

  void HandDetector::evaluateForestPixelRange(const uint32_t istart, 
    const uint32_t iend) {
    for (uint32_t i = istart; i <= iend; i++) {
      evaluateForestPixel(i);
    }

    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  void HandDetector::evaluateForestPixel(const uint32_t index) {
    hand_detector::evaluateDecisionForestPixel(labels_evaluated_, forest_,
      max_height_to_evaluate_, num_trees_to_evaluate_, depth_downsampled_, 
      down_width_, down_height_, index);
  }

  void HandDetector::reset() {
    //hands_uv_min_.resize(0);
    //hands_uv_max_.resize(0);
    hands_n_pts_.resize(0);
    hands_uvd_.resize(0);
  }

  void HandDetector::floodFillLabelData(bool* rhand_found, float* rhand_uvd, 
    bool* lhand_found, float* lhand_uvd) {
    // re-use the labels_evaluated data instead of allocating more
    //hands_uv_min_.resize(0);
    //hands_uv_max_.resize(0);
    hands_n_pts_.resize(0);
    hands_uvd_.resize(0);

    memset(pixel_on_queue_, 0, down_width_ * down_height_ * 
      sizeof(pixel_on_queue_[0]));
    queue_head_ = 0;
    queue_tail_ = 0; // When queue_head_ == queue_tail_ the queue is empty
    for (int v = 0; v < static_cast<int>(down_height_); v++) {
      for (int u = 0; u < static_cast<int>(down_width_); u++) {
        int cur_index = v * static_cast<int>(down_width_) + u;
        if (labels_filtered_[cur_index] == 1 && !pixel_on_queue_[cur_index]) {
          // We haven't already visited this pixel perform a floodfill 
          // starting from this pixel --> Potentially a new blob

          // Add the current pixel to the end of the queue
          pixel_queue_[queue_tail_] = cur_index;
          pixel_on_queue_[cur_index] = true;
          queue_tail_++;
          uint32_t n_blob_pts = 0;
          uint32_t n_hand_pts = 0;

          int u_ave = 0;
          int v_ave = 0;
          int depth_ave = 0;
          //cur_uv_min_.set(down_width_, down_height_);
          //cur_uv_max_.set(-1, -1);

          while (queue_head_ != queue_tail_) {
            // Take the pixel off the head
            int cur_pixel = pixel_queue_[queue_head_];

            int cur_u = cur_pixel % down_width_;
            int cur_v = cur_pixel / down_width_;
            queue_head_++;
            n_blob_pts++;

            if (labels_evaluated_[cur_pixel] == 1) {
              depth_ave += (int)depth_downsampled_[cur_pixel];
              n_hand_pts += 1;
            }

            u_ave += cur_u;
            v_ave += cur_v;

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
          if (n_blob_pts >= HD_MIN_PTS_PER_HAND_BLOB) {
            hands_n_pts_.pushBack(n_blob_pts);

            Float3 center(DT_DOWNSAMPLE * (float)u_ave / (float)n_blob_pts, 
              DT_DOWNSAMPLE * (float)v_ave / (float)n_blob_pts, 
              (float)depth_ave / (float)n_hand_pts);
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
      if (lhand_found) { *lhand_found = false; }
      *rhand_found = false;
      return;
    }

    // One hand found
    if (index_sec_max_hand_n_pts == MAX_UINT32 || lhand_found == NULL) {
      if (lhand_found != NULL &&
        hands_uvd_[index_max_hand_n_pts][0] < (src_width_ / 2)) {
        *lhand_found = true;
        lhand_uvd[0] = hands_uvd_[index_max_hand_n_pts][0];
        lhand_uvd[1] = hands_uvd_[index_max_hand_n_pts][1];
        lhand_uvd[2] = hands_uvd_[index_max_hand_n_pts][2];
        *rhand_found = false;
      } else {
        if (lhand_found) { *lhand_found = false; }
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
    if (u < static_cast<int>(down_width_) && u >=0 && 
        v < static_cast<int>(down_height_) && v >= 0) {
      int cur_index = u + v * down_width_;
      if ((labels_filtered_[cur_index] == 1) && (pixel_on_queue_[cur_index] == 0)) {
        // Add the pixel to the back of the queue
        pixel_queue_[queue_tail_] = cur_index;
        pixel_on_queue_[cur_index] = 1;
        queue_tail_++;
      }
    }
  }

  float floorf_sym(float value) {
    if (value < 0.0f) {
      return ceilf(value);
    } else {
      return floorf(value);
    }
  }

  float ceilf_sym(float value) {
    if (value < 0.0f) {
      return floorf(value);
    } else {
      return ceilf(value);
    }
  }

  float round(float r) {
    return (r > 0.0f) ? floor(r + 0.5f) : ceil(r - 0.5f);
  }

  // Find the hand points from the kenect point cloud + hand point
  // THIS IS ESCENTIALLY THE SAME CODE (slightly modified) that is in
  // HandStatistics --> One day I will completely remove HandStatistics
  void HandDetector::findHandLabelsFloodFill(const float* pt_hand_uvd, 
    const float* xyz, uint8_t* label) {
    float pt_hand_xyz[3];
    KinectInterface::convertUVDToApproxXYZ(1, pt_hand_uvd, pt_hand_xyz);
    // Search in a small UV window for the minimum depth value around
    // the hand point.  This makes sure we can seed our connected components 
    // with a real hand point.

    // We need the radius to be depth dependant, so transform a point with X 
    // offset back into UV space (using openNI) to calculate radius in UV space
    Float3 uv_vec;
    float pt_off_xyz[3];
    float pt_off_uvd[3];
    pt_off_xyz[0] = pt_hand_xyz[0] - HD_SMALL_HAND_RADIUS;
    pt_off_xyz[1] = pt_hand_xyz[1];
    pt_off_xyz[2] = pt_hand_xyz[2];
    KinectInterface::convertXYZToApproxUVD(1, pt_off_xyz, pt_off_uvd);
    uv_vec.set(pt_off_uvd[0] - pt_hand_uvd[0], pt_off_uvd[1] - pt_hand_uvd[1], 
      0.0f); 
    int radius = static_cast<int>(uv_vec.length());
    if (radius < HD_SMALL_HAND_RADIUS_MIN_UV) {
      radius = HD_SMALL_HAND_RADIUS_MIN_UV;
    }

    float min_pt_uvd[3];
    // get rid of annoying uninitialized gcc compiler error
    for (uint32_t i = 0; i < 3; i++) {
      min_pt_uvd[i] = 0;
    }
    uint32_t num_iterations = 0;
    while (!searchUVForMinHandPoint(pt_off_uvd, min_pt_uvd, radius, xyz)) {
      radius *= 2;  // If we're unsuccessful then repeat with double the radius
      num_iterations++;
      if (num_iterations > 10) {
        return;  // Give up
      }
    }

    // Add the one point that we know about and start a walk checking each 
    // adjacent pixel and if it is within some small theshold then add it to the
    // queue of pixels
    int minPtIndex = static_cast<int>(round(min_pt_uvd[1])) * src_width_ + 
      static_cast<int>(round(min_pt_uvd[0]));

    memset(pixel_on_queue_, 0, src_width_ * src_height_ * 
      sizeof(pixel_on_queue_[0]));
    queue_head_ = 0;
    queue_tail_ = 1;  // When queue_head_ == queue_tail_ the queue is empty
    pixel_queue_[queue_head_] = minPtIndex;
    pixel_on_queue_[minPtIndex] = true;
    int neighbourPtIndexUV[2];
    while (queue_head_ != queue_tail_) {
      // Take the current pixel off the queue
      int curPtIndex = pixel_queue_[queue_head_];
      int curPtU = curPtIndex % src_width_;
      int curPtV = curPtIndex / src_width_;         
      queue_head_++;
      // Add the point to the set
      label[curPtIndex] = 1;

      for (int i = 0; i < HD_N_PTS_FILL_KERNEL; i ++) {
        int cur_rad_u = static_cast<int>(ceilf_sym(floodFillKernel_[i][0] /  
          static_cast<float>(xyz[curPtIndex*3+2])));
        int cur_rad_v = static_cast<int>(ceilf_sym(floodFillKernel_[i][1] /  
          static_cast<float>(xyz[curPtIndex*3+2])));
        neighbourPtIndexUV[0] = curPtU + cur_rad_u;
        neighbourPtIndexUV[1] = curPtV + cur_rad_v;
        processNeighbour(neighbourPtIndexUV, curPtIndex, pt_hand_xyz, xyz);
      }
    }
  }

  void HandDetector::processNeighbour(const int* nieghbourPtUV, 
    const int curPtIndex, const float* ptHand, const float* xyz) {
    Float3 vec;
    if (nieghbourPtUV[0] >= 0 && nieghbourPtUV[0] < (int)src_width_ && 
      nieghbourPtUV[1] >= 0 && nieghbourPtUV[1] < (int)src_height_) {
      int nieghbourPtIndex = nieghbourPtUV[1]*src_width_ + nieghbourPtUV[0];
      if (!pixel_on_queue_[nieghbourPtIndex]) {  // don't add it if we have
        // See if the pixel is a user pixel and that it is outside our big rad
        vec[0] = xyz[nieghbourPtIndex*3] - ptHand[0];
        vec[1] = xyz[nieghbourPtIndex*3+1] - ptHand[1];
        vec[2] = xyz[nieghbourPtIndex*3+2] - ptHand[2];
        float len_sq = (vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]); 
        if (xyz[nieghbourPtIndex*3+2] > 0 && len_sq < (HD_HAND_RADIUS*HD_HAND_RADIUS)) {
          // See if the pixel is close to the current point (otherwise it's the 
          // background)
          vec[0] = xyz[nieghbourPtIndex*3] - xyz[curPtIndex*3];
          vec[1] = xyz[nieghbourPtIndex*3+1] - xyz[curPtIndex*3+1];
          vec[2] = xyz[nieghbourPtIndex*3+2] - xyz[curPtIndex*3+2];     
          len_sq = (vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);         
          if (len_sq < HD_BACKGROUND_THRESH_SQ) {
            pixel_queue_[queue_tail_] = nieghbourPtIndex;
            pixel_on_queue_[nieghbourPtIndex] = true;
            queue_tail_++;
          }
        }
      }
    }
  }

  void HandDetector::clipOnScreen(int& uMin, int& uMax, int& vMin, int& vMax, 
    float* pt_hand_uvd, int radius) {
    // Now clamp the UV min and max coordinates so we don't go off screen
    uMin = static_cast<int>(round(pt_hand_uvd[0])) - radius;
    uMin = std::max<int>(uMin, 0);
    uMax = static_cast<int>(round(pt_hand_uvd[0])) + radius;
    uMax = std::min<int>(uMax, src_width_ - 1);

    vMin = static_cast<int>(round(pt_hand_uvd[1])) - radius;
    vMin = std::max<int>(vMin, 0);
    vMax = static_cast<int>(round(pt_hand_uvd[1])) + radius;
    vMax = std::min<int>(vMax, src_height_ - 1);
  }

  bool HandDetector::searchUVForMinHandPoint(float* pt_hand_uvd,
    float* min_pt, const int radius, const float* xyz) {
    int uMin, uMax, vMin, vMax;
    clipOnScreen(uMin, uMax, vMin, vMax, pt_hand_uvd, radius);

    Float3 vec;
    min_pt[2] = std::numeric_limits<float>::infinity();
    for (int u = uMin; u <= uMax;  u ++) {
      for (int v = vMin; v <= vMax;  v ++) {
        int i = v * src_width_ + u;
        if (xyz[i*3+2] > (1.0f / XYZ_UNIT)) {
          if (xyz[i*3+2] < min_pt[2]) {
            min_pt[0] = (float)u;
            min_pt[1] = (float)v;
            min_pt[2] = xyz[i*3+2];
          }
        }
      }
    }
    // Check if we were sucessful
    if (min_pt[2] < std::numeric_limits<float>::infinity()) {
      min_pt[2] *= XYZ_UNIT;
      return true;
    } else {
      return false;
    }
  }

  void HandDetector::num_trees_to_evaluate(const int32_t val) {
    if (val <= num_trees_) {
      num_trees_to_evaluate_ = val;
    } else {
      std::cout << "HandDetector::num_trees_to_evaluate() - WARNING: val is"
        " greater than the number of trees in this decision forest model";
      std::cout  << std::endl;
    }
  }

  void HandDetector::max_height_to_evaluate(const int32_t val) {
    if (val <= max_height_) {
      max_height_to_evaluate_ = val;
    } else {
      std::cout << "HandDetector::max_height_to_evaluate() - WARNING: val is"
        " greater than the maximum tree height in this decision forest model";
      std::cout  << std::endl;
    }
  }

};  // namespace hand_detector
};  // namespace kinect_interface
