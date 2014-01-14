//
//  kinect_interface.h
//
//  Jonathan Tompson
//
//  USAGE:
//
//  using namespace kinect_interface;
//  jtil::data_str::Vector<KinectInterface*> kinects_;
//  jtil::data_str::Vector<std::string> ids;
//
//  // Init the devices
//  KinectInterface::getDeviceIDs(ids);
//  for (uint32_t i = 0; i < ids.size(); i++) {
//    kinects_.push_back(new KinectInterface(ids[i]));
//  }
//
//  // Run the app
//  while (running) {
//    for (uint32_t i = 0; i < kinects_.size(); i++) {
//      kinect_[i]->lockData();
//      // <GET ALL THE DATA HERE>
//      kinect_[i]->unlockData();
//    }
//  }
//  
//  // Shutdown the devices
//  for (uint32_t i = 0; i < kinects_.size(); i++) {
//    kinect_[i]->shutdownKinect(); 
//    delete kinect_[i];
//  }
// 

#pragma once

#include <mutex>
#include <condition_variable>
#include <thread>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "jtil/data_str/vector_managed.h"
#include "jtil/data_str/vector.h"

namespace jtil { namespace clk { class Clk; } }

struct IKinectSensor;
struct IDepthFrameReader;
struct IColorFrameReader;
struct ICoordinateMapper;
struct IBodyFrameReader;
// To avoid exposing the structs of the windows SDK I escentially re-define
// them here.  Note that at runtime we check the sizes of the two types to make
// sure they're the same.
struct XYZPoint {
  float x;
  float y;
  float z;
};
struct XYPoint {
  float x;
  float y;
};
		
namespace kinect_interface {

  typedef enum {
    DEPTH_STREAM = 0,
    RGB_STREAM = 1,
    IR_STREAM = 2,
    NUM_STREAMS = 3
  } KinectStreamID;

  const uint32_t depth_w = 512;
  const uint32_t depth_h = 424;
  const uint32_t depth_dim = depth_w * depth_h;
  const float depth_hfov = 70.6f;  // (horizontal)
  const float depth_vfov = 60.0f;  // (vertical)

  const uint32_t rgb_w = 1920;
  const uint32_t rgb_h = 1080;
  const uint32_t rgb_dim = rgb_w * rgb_h;
  const float rgb_hfov = 84.1f;  // (horizontal)
  const float rgb_vfov = 53.8f;  // (vertical)

  // The array size is (depth_dim * 2_bytes + 3 * depth_dim * 1_byte)
  const uint32_t depth_arr_size_bytes = (depth_dim * 2) + (3 * depth_dim * 1);

  // Beyond this depth we treat everything as background.  It is a very rough
  // cutoff but it has a big impact on decision forest evaluation time and some
  // other image processing sections.
  const uint16_t max_depth = 2000;

  class KinectInterface{
  public:
    // KinectInterface() - Starts up the kinect thread.  Call getDeviceIDs to 
    // get the callable IDS
    KinectInterface(const char* device_id);  
    // Destroy the interface. Note: must not be called until thread is joined
    ~KinectInterface();
    // shutdownKinect is blocking until the kinect thread has shut down
    void shutdownKinect();  

    static void getDeviceIDs(jtil::data_str::VectorManaged<const char*>& ids);

    const uint8_t* rgb() const;  // NOT THREAD SAFE!  Use lockData()
    const XYZPoint* xyz() const;  // NOT THREAD SAFE!  Use lockData()
    const uint16_t* depth() const;  // NOT THREAD SAFE!  Use lockData()
    const uint8_t* depth_colored() const;  // NOT THREAD SAFE!  Use lockData()
    const int64_t depth_frame_time() const { return depth_frame_time_; }
    const uint64_t depth_frame_number() const { return depth_frame_number_; }
    const int64_t rgb_frame_time() const { return rgb_frame_time_; }
    const uint64_t rgb_frame_number() const { return rgb_frame_number_; }
    const char* kinect_fps_str() const { return kinect_fps_str_; }

    void setSyncRGB(const bool sync_rgb);
    void setSyncDepth(const bool sync_depth);
    void setSyncDepthColored(const bool sync_depth_colored);
    void setSyncXYZ(const bool sync_xyz);
    void setSyncBody(const bool sync_body);

    inline void lockData() { data_lock_.lock(); };
    inline void unlockData() { data_lock_.unlock(); };

    // Note, this requires an instance of the kinect actually running :-(
    // Hopefully this can be broken out like I did for OpenNI
    // NOTE: xyz units are in meters! (while depth is in mm)
    void convertDepthFrameToXYZ(const uint32_t n_pts, const uint16_t* depth, 
      XYZPoint* xyz);  
    void convertDepthFrameToXYZ(const uint32_t n_pts, const uint16_t* depth, 
      float* xyz);  // Requires O(n) copy internally
    void convertXYZToDepthSpace(const uint32_t n_pts, const XYZPoint* xyz,
      XYPoint* uv_pos);  // Note: z, in XYZPoint will be the depth
    void convertUVDToXYZ(const uint32_t n_pts, const float* uvd, 
      float* xyz);  // Requires O(n) copy internally
    void convertXYZToUVD(const uint32_t n_pts, const float* xyz,
      float* uvd);  // Requires O(n) copy internally

  private:
    bool sync_depth_;  // default true
    bool sync_rgb_;  // default true
    bool sync_depth_colored_;  // default true
    bool sync_xyz_;  // default true
    bool sync_body_;  // default true

    // Kinect Device
    std::string device_id_;
    IKinectSensor* kinect_sensor_;
    IDepthFrameReader* depth_frame_reader_;
    IColorFrameReader* rgb_frame_reader_;
    ICoordinateMapper* coord_mapper_;
    IBodyFrameReader* body_frame_reader_;
    uint16_t max_depth_;
    uint16_t min_depth_;
    float cur_depth_vfov_;
    float cur_depth_hfov_;
    float cur_rgb_vfov_;
    float cur_rgb_hfov_;

    static jtil::data_str::Vector<KinectInterface*> open_kinects_;
    static std::recursive_mutex sdk_static_lock_;
    static jtil::clk::Clk shared_clock_;
    bool device_initialized_;
   
    // Multi-threading
    std::recursive_mutex data_lock_;
    std::thread kinect_thread_;
    
    // Processed data
    // NOTE: depth_ and depth_colored_ are actually one contiguous array:
    // depth_colored_ just indexes into depth_
    uint16_t depth_[depth_arr_size_bytes/2];
    uint8_t* depth_colored_;  // Size of the depth image
    uint8_t rgb_[rgb_dim * 4];  // enough space for 4 channels are allocated, but we only use 3
    XYZPoint xyz_[depth_dim];
    XYPoint uv_depth_2_rgb_[depth_dim];
    uint64_t depth_frame_number_;
    int64_t depth_frame_time_;
    uint64_t rgb_frame_number_;
    int64_t rgb_frame_time_;
    uint32_t body_frame_number_;
    int64_t body_frame_time_;
    char kinect_fps_str_[256];

    // Some temporary data, this can likely be cleaned up (shared with
    // other arrays), but is kept separate for now to avoid conflicts.
    XYPoint uv_tmp_[depth_dim];
    uint16_t depth_tmp_[depth_dim];
    XYZPoint xyz_tmp_[depth_dim];
    
    bool kinect_running_;
    
    // MAIN UPDATE THREAD:
    void kinectUpdateThread();
    void init(const std::string& device_id);
    IKinectSensor* getDeviceByID(const std::string& id);
    void waitForDepthFrame(const uint64_t timeout_ms = -1);  // -1 = inf
    void waitForRGBFrame(const uint64_t timeout_ms = -1);  // -1 = inf
  };
  
#ifndef EPSILON
  #define EPSILON 0.000001f
#endif
  
};  // namespace kinect_interface
