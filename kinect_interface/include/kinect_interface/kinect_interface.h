//
//  kinect_interface.h
//
//  Source initially from OpenNI libraries, then heavily modified by 
//  Jonathan Tompson
// 

#ifndef KINECT_INTERFACE_KINECT_INTERFACE_HEADER
#define KINECT_INTERFACE_KINECT_INTERFACE_HEADER

#include <mutex>
#include <condition_variable>
#include <thread>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"

#define SKEL_NJOINTS 25
#define SKELETON_SMOOTHING 0.05f
#define MIRROR true  // true if you want OPEN to mirror all kinect data
#define KINECT_INTERFACE_NUM_WORKER_THREADS 4
#define OPENNI_WAIT_TIMEOUT 50  // Maybe ms?

namespace jtil { namespace clk { class Clk; } }
namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }
namespace jtil { namespace threading { class ThreadPool; } }

namespace openni { class Device; }
namespace openni { class VideoStream; }
namespace openni { class VideoFrameRef; }
namespace openni { class VideoMode; }
namespace openni { class SensorInfo; }
		
namespace kinect_interface {
  namespace hand_detector { class HandDetector; }
  namespace hand_net { class HandNet; }
  namespace hand_net { class HandModelCoeff; }
  class DepthImagesIO;

  typedef enum {
    DEPTH = 0,
    RGB = 1,
    NUM_STREAMS = 2
  } OpenNIStreamID;

  class OpenNIFuncs;

  class KinectInterface {
  public:
    // If device_id == NULL then it will open the first avalible device
    // Otherwise you can use findDevices() to get a list of connected devices
    KinectInterface(const char* device_uri);  // Starts up the kinect update thread
    ~KinectInterface();  // Must not be called until thread is joined
    void shutdownKinect();  // Blocking until the kinect thread has shut down

    static void findDevices(jtil::data_str::VectorManaged<char*>& devices);

    const uint8_t* rgb() const;  // NOT THREAD SAFE!  Use lockData()
    const float* xyz() const;  // NOT THREAD SAFE!  Use lockData()
    const uint16_t* depth() const;  // NOT THREAD SAFE!  Use lockData()
    const uint16_t* depth1mm() const;  // NOT THREAD SAFE!  Use lockData()
    const uint8_t* labels() const { return labels_; }  // NOT THREAD SAFE!  Use lockData()
    const uint8_t* filteredDecisionForestLabels() const;  // NOT THREAD SAFE!  Use lockData()
    const uint8_t* rawDecisionForestLabels() const;  // NOT THREAD SAFE!  Use lockData()
    hand_detector::HandDetector* hand_detector() { return hand_detector_; }
    OpenNIFuncs* openni_funcs() { return openni_funcs_; };

    inline void lockData() { data_lock_.lock(); };
    inline void unlockData() { data_lock_.unlock(); };
    
    char* getStatusMessage();

    const double fps() const { return fps_; }
    const uint64_t frame_number() const { return frame_number_; }
    const jtil::math::Int2& depth_dim() const { return depth_dim_; }
    const jtil::math::Int2& rgb_dim() const { return rgb_dim_; }

  private:
    // Kinect nodes
    openni::Device* device_;
    openni::VideoStream* streams_[NUM_STREAMS];
    openni::VideoFrameRef* frames_[NUM_STREAMS];
    static bool openni_init_;
    static std::mutex openni_static_lock_;
    static uint32_t openni_devices_open_;
    bool device_initialized_;
    bool crop_depth_to_rgb_;
    bool flip_image_;
    bool depth_color_sync_;
   
    // Multi-threading
    uint32_t threads_finished_;
    jtil::threading::ThreadPool* tp_;
    std::mutex thread_update_lock_;  // For workers to communicate with main thread
    std::condition_variable not_finished_;
    jtil::data_str::VectorManaged<jtil::threading::Callback<void>*>* pts_world_thread_cbs_;
    std::recursive_mutex data_lock_;
    std::thread kinect_thread_;
    jtil::math::Int2 depth_dim_;
    int depth_fps_setting_;
    jtil::math::Int2 rgb_dim_;  // RGB dimension always matches depth
    int rgb_fps_setting_;
    
    // Processed data
    bool depth_format_100um_;
    OpenNIFuncs* openni_funcs_;
    uint16_t* depth_1mm_;
    float* pts_uvd_;
    float* pts_world_;
    uint8_t* labels_;  // Generated by hand_detector
    uint64_t frame_number_;
    double fps_;
    float max_depth_;
    
    // Hand data and data collection for machine learning
    hand_detector::HandDetector* hand_detector_;

    // Depth image IO (mostly for loading the debug image)
    DepthImagesIO* image_io_;
    
    // Status structures
    bool kinect_running_;
    char status_str_[256];
    
    jtil::clk::Clk* clk_;
    
    // MAIN UPDATE THREAD:
    void kinectUpdateThread();
    
    void init(const char* device_uri);
    void initOpenNI(const char* device_uri);
    void initDepth();
    void initRGB();
    void convertDepthToWorld();
    void convertDepthToWorldThread(const uint32_t start, const uint32_t end);
    openni::VideoMode findMaxResYFPSMode(const openni::SensorInfo& sensor,
      const int required_format) const;
    openni::VideoMode findMatchingMode(const openni::SensorInfo& sensor,
      const jtil::math::Int2& dim, const int fps, const int format) const;
    static void initOpenNIStatic();
    static void shutdownOpenNIStatic();
    static void checkOpenNIRC(int rc, const char* error_msg);
    static std::string formatToString(const int mode);
    static void printMode(const openni::VideoMode& mode);
    inline bool kinect_running() const { return kinect_running_; }
    void setCropDepthToRGB(const bool crop_depth_to_rgb);  // internal use only
    void setFlipImage(const bool flip_image);  // internal use only
    void setDepthColorSync(const bool depth_color_sync);  // internal use only
  };
  
#ifndef EPSILON
  #define EPSILON 0.000001f
#endif
  
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_KINECT_INTERFACE_HEADER
