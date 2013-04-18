//
//  frame_data.h
//
//  Created by Jonathan Tompson on 5/1/12.
//
//  Since the Kinect Update thread sits in its local space, we need a buffer
//  to store all kinect data locally to the app.  This class is just a handy
//  container to store all the kinect data for each frame.
//

#ifndef APP_FRAME_DATA_HEADER
#define APP_FRAME_DATA_HEADER

#include <thread>
#include <string>
#include <random>
#include "jtil/jtil.h"
#include "app/app.h"  // for KinectOutput and LabelType
#include "kinect_interface/kinect_interface.h"
#include "kinect_interface/hand_net/hand_image_generator.h"  // for HN_IM_SIZE
#include "kinect_interface/hand_net/hand_net.h"  // for HandCoeffConvnet
#include "kinect_interface/hand_detector/decision_tree_structs.h"

#define RGB_DEPTH_DATA_SIZE_BYTES ((1 * src_dim * 3) + (2 * src_dim))

namespace app {

  struct FrameData {
  public:
    FrameData();
    ~FrameData();

    // syncWithKinect: Return true if a new frame was updated
    bool syncWithKinect(kinect_interface::KinectInterface* kinect,
      const bool sync_depth_hand_detector, uint8_t* render_labels = NULL);
    // saveSensorData: Return true if a new datafile was created (false usually
    // indicates that the frame has already been saved to disk).
    bool saveSensorData(const std::string& filename);

    uint64_t frame_number;
    uint64_t saved_frame_number;
    char kinect_fps_str[256];

    uint8_t* rgb;  // A ptr into rgb_depth_data_
    float xyz[src_dim * 3];
    uint8_t labels[src_dim];
    int16_t* depth;  // A ptr into rgb_depth_data_
    int16_t depth_hand_detector[src_dim / (DT_DOWNSAMPLE*DT_DOWNSAMPLE)];
    float normals_xyz[src_dim * 3];
    uint16_t hand_detector_depth[src_dim];
    float convnet_depth[HN_SRC_IM_SIZE * HN_SRC_IM_SIZE];

  private:
    uint8_t depth_rgb_data_[RGB_DEPTH_DATA_SIZE_BYTES]; 
    uint8_t depth_rgb_data_compressed_[RGB_DEPTH_DATA_SIZE_BYTES*2];

    // Non-copyable, non-assignable.
    FrameData(FrameData&);
    FrameData& operator=(const FrameData&);
  };

};  // namespace app

#endif  // APP_FRAME_DATA_HEADER
