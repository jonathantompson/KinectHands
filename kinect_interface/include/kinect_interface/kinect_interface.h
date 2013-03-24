//
//  kinect_interface.h
//
//  Source initially from OpenNI libraries, then heavily modified by 
//  Jonathan Tompson
// 

#ifndef KINECT_INTERFACE_KINECT_INTERFACE_HEADER
#define KINECT_INTERFACE_KINECT_INTERFACE_HEADER

#include <mutex>
#include <thread>
#include "jtil/math/math_types.h"

#define SKEL_NJOINTS 25
#define SKELETON_SMOOTHING 0.05f
#define MIRROR true  // true if you want OPEN to mirror all kinect data
#define src_width 640
#define src_height 480
#define src_dim (src_width * src_height)

namespace jtil { namespace clk { class Clk; } }

namespace xn {
  class DepthGenerator;
  class ImageGenerator;
  class UserGenerator;
  class Context;
  class PoseDetectionCapability;
  class SkeletonCapability;
  class EnumerationErrors;
}

typedef enum XnCalibrationStatus XnCalibrationStatus;
		
namespace kinect_interface {
  namespace hand_detector { class HandDetector; }
  namespace hand_net { class HandNet; }
  namespace hand_net { class HandModel; }
  class DepthImagesIO;

  class KinectInterface {
  public:
    // Top level interface
    KinectInterface();  // Starts up the kinect update thread
    ~KinectInterface();  // Must not be called until thread is joined
    void shutdownKinect();  // Blocking until the kinect thread has shut down
    
    const uint8_t* rgb() const { return rgb_; }  // NOT THREAD SAFE!  Use lockData()
    const uint16_t* depth() const { return depth_; }  // NOT THREAD SAFE!  Use lockData()
    const uint8_t* labels() const { return labels_; }  // NOT THREAD SAFE!  Use lockData()
    const float* coeff_convnet() const;  // NOT THREAD SAFE!  Use lockData()
    const uint8_t* filteredDecisionForestLabels() const;  // NOT THREAD SAFE!  Use lockData()
    const uint8_t* rawDecisionForestLabels() const;  // NOT THREAD SAFE!  Use lockData()
    const jtil::math::Float3& uvd_com() const;
    hand_detector::HandDetector* hand_detector() { return hand_detector_; }
    hand_net::HandNet* hand_net() { return hand_net_; }

    inline void lockData() { data_lock_.lock(); };
    inline void unlockData() { data_lock_.unlock(); };
    
    void saveCalibration();
    bool loadCalibration();
    void resetTracking();
    
    char* getStatusMessage();
    int getNumUsersTracked();

    void detectPose(const int16_t* depth, const uint8_t* labels);

    const double fps() const { return fps_; }
    const uint64_t frame_number() const { return frame_number_; }

  private:
    static KinectInterface* g_kinect_;

    // Kinect nodes
    xn::DepthGenerator* dg_;  // depth generator
    xn::ImageGenerator* ig_;  // image generator
    xn::UserGenerator* ug_;  // user generator
   
    // Multi-threading
    std::recursive_mutex data_lock_;
    std::recursive_mutex context_lock_;
    std::thread kinect_thread_;
    
    // Data copied out of the kinect
    uint8_t rgb_[src_dim * 3];
    uint16_t depth_[src_dim];
    float pts_uvd_[src_dim * 3];
    float pts_world_[src_dim * 3];
    uint8_t labels_[src_dim];  // Generated by hand_detector
    uint64_t frame_number_;
    double fps_;
    bool joints_found_[SKEL_NJOINTS];
    float joints_world_[SKEL_NJOINTS * 3];  // X, Y, Z per joint
    float joints_uvd_[SKEL_NJOINTS * 3];
    float max_depth_;
    
    // Hand data and data collection for machine learning
    hand_detector::HandDetector* hand_detector_;
    int16_t depth_from_file_[src_dim];
    float* coeff_convnet_from_file_;
    uint8_t labels_from_file_[src_dim];
    uint8_t rgb_from_file_[src_dim * 3];
    float pts_world_from_file_[src_dim * 3];

    // Convolutional Neural Network
    hand_net::HandNet* hand_net_;
    hand_net::HandModel* hands_[2];

    // Depth image IO (mostly for loading the debug image)
    DepthImagesIO* image_io_;
    
    // Status structures
    bool kinect_running_;
    char status_str_[256];
    int num_users_found_;
    int num_users_tracked_;
    bool tracking_skeleton_;

    xn::Context* context_;  // type: xn::Context
    
    jtil::clk::Clk* clk_;
    
    // MAIN UPDATE THREAD:
    void kinectUpdateThread();

    // Kinect CBs
    static void __stdcall newUser(xn::UserGenerator& generator, 
      unsigned int nId, void* pCookie);
    static void __stdcall lostUser(xn::UserGenerator& generator, 
      unsigned int nId, void* pCookie);
    static void __stdcall poseDetected(xn::PoseDetectionCapability& capability, 
      const char* strPose, unsigned int nId, void* pCookie);
    static void __stdcall calStart(xn::SkeletonCapability& capability, 
      unsigned int nId, void* pCookie);
    static void __stdcall calFinish(xn::SkeletonCapability& capability, 
      unsigned int  nId, XnCalibrationStatus eStatus, void* pCookie);

    void logErrors(xn::EnumerationErrors& rErrors);
    
    bool init();
    bool initUserGenerator();
    void addLicense(const std::string& sVendor, const std::string& sKey);
    void enableLogging();
    void updateJoint(const unsigned int user_id, const int32_t joint);
    inline bool kinect_running() { return kinect_running_; }
  };
  
#ifndef EPSILON
  #define EPSILON 0.000001f
#endif
  
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_KINECT_INTERFACE_HEADER
