//
//  kinect_interface.h
//
//  Jonathan Tompson
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
struct IMultiSourceFrameReader;
		
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

  class KinectInterface{
  public:
    // KinectInterface() - Starts up the kinect thread.  Call getDeviceIDs to 
    // get the callable IDS
    KinectInterface(const std::string& device_id);  
    // Destroy the interface. Note: must not be called until thread is joined
    ~KinectInterface();
    // shutdownKinect is blocking until the kinect thread has shut down
    void shutdownKinect();  

    static void getDeviceIDs(jtil::data_str::Vector<std::string>& ids);

    const uint8_t* rgb() const;  // NOT THREAD SAFE!  Use lockData()
    const uint8_t* ir() const;  // NOT THREAD SAFE!  Use lockData()
    const uint8_t* registered_rgb() const;  // NOT THREAD SAFE!  Use lockData()
    const float* xyz() const;  // NOT THREAD SAFE!  Use lockData()
    const uint16_t* depth() const;  // NOT THREAD SAFE!  Use lockData()
    double depth_frame_time() { return depth_frame_time_; }

    inline void lockData() { data_lock_.lock(); };
    inline void unlockData() { data_lock_.unlock(); };

    const uint64_t depth_frame_number() const { return depth_frame_number_; }

  private:
    // Kinect Device
    std::string device_id_;
    IKinectSensor* kinect_sensor_;
    // TODO: We need m_pMultiSourceFrameReader
    IMultiSourceFrameReader* frame_reader_;


    static jtil::data_str::Vector<KinectInterface*> open_kinects_;
    static std::recursive_mutex sdk_static_lock_;
    static jtil::clk::Clk shared_clock_;
    bool device_initialized_;
   
    // Multi-threading
    std::recursive_mutex data_lock_;
    std::thread kinect_thread_;
    
    // Processed data
    RGBQUAD* depth_;
    uint64_t depth_frame_number_;
    double depth_frame_time_;
    float max_depth_;
    bool flip_image_;
    
    bool kinect_running_;
    
    // MAIN UPDATE THREAD:
    void kinectUpdateThread();
    void init(const std::string& device_id);
    IKinectSensor* getDeviceByID(const std::string& id);
  };
  
#ifndef EPSILON
  #define EPSILON 0.000001f
#endif
  
};  // namespace kinect_interface
