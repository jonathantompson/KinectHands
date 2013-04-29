#include <stdio.h>  // for printf
#include <thread>
#include <string>
#include <iostream>  // for cout
#if defined(WIN32) || defined(_WIN32) 
  #include <direct.h>
#endif
#include "app/frame_data.h"
#include "jtil/ui/ui.h"
#include "kinect_interface/kinect_interface.h"
#include "kinect_interface/open_ni_funcs.h"
#include "kinect_interface/hand_detector/hand_detector.h"
#include "kinect_interface/hand_net/hand_image_generator.h"
#include "kinect_interface/hand_net/hand_model_coeff.h"  // for camera parameters
#include "kinect_interface/hand_detector/decision_tree_structs.h"
#include "jtil/image_util/image_util.h"
#include "jtil/fastlz/fastlz.h"

#ifndef NULL
#define NULL 0
#endif
#define SAFE_DELETE(x) if (x) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x) { delete[] x; x = NULL; }

#ifdef _WIN32
#ifndef snprintf
#define snprintf _snprintf_s
#endif
#endif

using std::wruntime_error;
using namespace jtil;
using namespace jtil::settings;
using namespace jtil::math;
using namespace jtil::image_util;

namespace app {
  double FrameData::time_sec = 0;

  FrameData::FrameData() {
    depth_frame_number = 0;
    rgb_frame_number = 0;
    ir_frame_number = 0;
    time_accum = 0;
    last_frame_time = 0;

    saved_min_frame_number = 0;

    depth = (int16_t*)depth_rgb_data_;
    registered_rgb = (uint8_t*)(&depth[src_dim]);

    memset(depth, 0, sizeof(depth[0]) * src_dim);
    memset(rgb_ir, 0, 3 * sizeof(rgb_ir[0]) * src_dim);
    memset(registered_rgb, 0, 3 *sizeof(registered_rgb[0]) * src_dim);

    GET_SETTING("sync_ir_stream", bool, sync_ir_stream_);
  }

  FrameData::~FrameData() {
  }

  bool FrameData::syncWithKinect(kinect_interface::KinectInterface* kinect,
    const bool sync_depth_hand_detector, uint8_t* render_labels) {
    time_accum += (time_sec - last_frame_time);
    last_frame_time = time_sec;
    if (time_accum > 0.5) {
      fps = (double)(depth_frame_number - last_depth_frame_number) / time_accum;
      time_accum = 0;
      last_depth_frame_number = depth_frame_number;
#if defined(WIN32) || defined(_WIN32)
#pragma warning(push)
#pragma warning(disable:4996)
#endif
      snprintf(kinect_fps_str, 255, "%.2f", fps);
#if defined(WIN32) || defined(_WIN32)
#pragma warning(pop)
#endif
    }

    bool ir_changed = ir_frame_number != kinect->ir_frame_number();
    bool rgb_changed = rgb_frame_number != kinect->rgb_frame_number();
    bool depth_changed = depth_frame_number != kinect->depth_frame_number();
    if (kinect != NULL && (ir_changed || rgb_changed || depth_changed)) {
      // Grab the kinect data
      kinect->lockData();
      
      if (!sync_ir_stream_) {
        if (rgb_changed ) {
          memcpy(rgb_ir, kinect->rgb(), sizeof(rgb_ir[0]) * src_dim * 3);
          rgb_frame_number = kinect->rgb_frame_number();
        }
      } else {
        if (ir_changed) {
          memcpy(rgb_ir, kinect->ir(), sizeof(rgb_ir[0]) * src_dim * 3);
          ir_frame_number = kinect->ir_frame_number();   
        }
      }
      if (depth_changed) {
        memcpy(registered_rgb, kinect->registered_rgb(), 
          sizeof(registered_rgb[0]) * src_dim * 3);
        memcpy(xyz, kinect->xyz(), sizeof(xyz[0]) * src_dim * 3);
        memcpy(depth, kinect->depth(), sizeof(depth[0]) * src_dim);
        if (sync_depth_hand_detector) {
          memcpy(depth_hand_detector, kinect->hand_detector()->depth_downsampled(), 
            sizeof(depth_hand_detector[0]) * src_dim / (DT_DOWNSAMPLE*DT_DOWNSAMPLE));
        }
        memcpy(labels, kinect->labels(), sizeof(labels[0]) * src_dim);
        depth_frame_number = kinect->depth_frame_number();

        if (render_labels != NULL) {
          int label_type_enum;
          GET_SETTING("label_type_enum", int, label_type_enum);
          switch ((LabelType)label_type_enum) {
          case OUTPUT_NO_LABELS:
            // Nothing to do
            break;
          case OUTPUT_UNFILTERED_LABELS:
            UpsampleNoFiltering<uint8_t>(render_labels, 
              kinect->hand_detector()->labels_evaluated(), 
              src_width / DT_DOWNSAMPLE, src_height / DT_DOWNSAMPLE, DT_DOWNSAMPLE);
            break;
          case OUTPUT_FILTERED_LABELS:
            image_util::UpsampleNoFiltering<uint8_t>(render_labels, 
              kinect->filteredDecisionForestLabels(), 
              src_width / DT_DOWNSAMPLE, src_height / DT_DOWNSAMPLE, DT_DOWNSAMPLE);
            break;
          case OUTPUT_FLOODFILL_LABELS:
            memcpy(render_labels, labels, sizeof(render_labels[0]) * src_dim);
            break;
          default:
            throw std::wruntime_error("App::run() - ERROR - label_type_enum "
              "invalid enumerant!");
          }
        }
      }  // if (depth_changed) {
      kinect->unlockData();
      return true;
    } else {
      return false;
    }
  }

  bool FrameData::saveSensorData(const bool calibration,
    const uint32_t kinect_index) {
    uint64_t min_frame = std::min<uint64_t>(depth_frame_number, 
      std::min<uint64_t>(rgb_frame_number, ir_frame_number));
    if (saved_min_frame_number == min_frame) {
      return false;
    } else {
      std::stringstream ss;
      uint64_t time_ns = (uint64_t)(time_sec * 1e9);
      if (calibration) {
        ss << "calb" << kinect_index << "_" << time << ".bin";
      } else {
        ss << "hands" << kinect_index << "_" << time << ".bin";
      }

      // Flag the user pixels (which for now are just pixels less than some 
      // threshold):
      uint16_t* depth_dst = (uint16_t*)depth_rgb_data_;
      for (int i = 0; i < src_width * src_height; i++) {
        if (depth[i] < GDT_INNER_DIST) {
          depth_dst[i] |= 0x8000;  // Mark the second MSB
        }
      }
      uint8_t* rgb_dst = (uint8_t*)&depth_dst[src_dim];

      // Compress the array
      static const int compression_level = 1;  // 1 fast, 2 better compression
      int compressed_length = fastlz_compress_level(compression_level,
        (void*)depth_rgb_data_, RGB_DEPTH_DATA_SIZE_BYTES, 
        (void*)depth_rgb_data_compressed_);

      //int compressed_length = jtil::ucl::UCLHelper::inPlaceCompress(
      //  depth_rgb_data_, RGB_DEPTH_DATA_SIZE_BYTES, 1);

      // Now save the array to file
#ifdef _WIN32
      _mkdir("./data/hand_depth_data/");  // Silently fails if dir exists
      std::string full_filename = "./data/hand_depth_data/" + ss.str();
#endif
#ifdef __APPLE__
      std::string full_path = file_io::GetHomePath() +
        std::string("Desktop/data/hand_depth_data/");
      struct stat st;
      if (stat(full_path.c_str(), &st) != 0) {
        if (mkdir(full_path.c_str(), S_IRWXU|S_IRWXG) != 0) {
          printf("Error creating directory %s: %s\n", full_path.c_str(),
            strerror(errno));
          return false;
        } else {
          printf("%s created\n", full_path.c_str());
        }
      }
      std::string full_filename = full_path + filename;
#endif
      // Save the file
      std::ofstream file(full_filename.c_str(), std::ios::out | std::ios::binary);
      if (!file.is_open()) {
        throw std::runtime_error(std::string("error opening file:") + ss.str());
      }
      // file.write((const char*)depth_rgb_data_, compressed_length);
      file.write((const char*)depth_rgb_data_compressed_, compressed_length);
      file.close();
      saved_min_frame_number = min_frame;
      return true;
    }
  }

}  // namespace app
