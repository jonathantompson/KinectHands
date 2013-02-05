//
//  kinect_interface.h
//
//  Source origionally from OpenNI libraries, then modified by Otavio B. for
//  Computer Vision class code, then adapted by Ken Perlin's lab for KinectHands

#ifndef KINECT_INTERFACE_HEADER
#define KINECT_INTERFACE_HEADER

#include <condition_variable>
#include <string>
#include <deque>
#include <vector>
#include "XnCppWrapper.h"
#include "XnVNite.h"

#define SKEL_NJOINTS (XN_SKEL_RIGHT_FOOT+1)
#define SKELETON_SMOOTHING 0.05f
#define NI_MIRROR true
#define NUM_WORKER_THREADS 2
#define DELAY_RGB  // This is necessary to better sync the frames
// #define MULTI_THREADED  // Uncheck this to enable left + right hand threads

#define XN_CB XN_CALLBACK_TYPE

class Clock;
class HandDetector;
namespace threading { class ThreadPool; }
		
namespace kinect {
  
  class HandStatistics;
  
  enum enumDepthColoring {
    COLORING_BLUES,
    COLORING_GREY,
    COLORING_INVERSE_RAW_DEPTH,
  };
  
  class KinectInterface {
  public:
    friend class HandStatistics;
    
    // Top level interface
    KinectInterface(bool tracking_on);  // Starts up the kinect update thread
    void requestShutdown();
    bool kinect_running();
    void join();  // Wait until kinect update thread has finished
    ~KinectInterface();  // Must not be called until thread is joined
    
    // Getters and Setters
    XnPoint3D* getJoints() { return joints_; }
    XnPoint3D* getProjectedJoints() { return joints_projected_; }
    
    xn::Context& getContext() { return context; }
    
    XnRGB24Pixel* getRGBImageUnsafe();
    XnUInt8* getDepthImageUnsafe();
    XnDepthPixel* getDepthRawUnsafe();
    inline void lockData() { data_lock_.lock(); };
    inline void unlockData() { data_lock_.unlock(); };
    
    void SaveCalibration();
    bool LoadCalibration();
    void ResetTracking(bool tracking_on);
    
    void drawSkeletonPts();
    void drawSkeletonLines();
    void drawHandStatistics(bool draw_pts, bool draw_obb, bool draw_finger);
    void drawHandDetectorOBB();
    
    inline XnUInt16 getWidth() { return width_; }
    inline XnUInt16 getHeight() { return height_; }
    
    char* getStatusMessage();
    int getNumUsersTracked();
    
    inline HandStatistics* getLHand() { return lHand_; }
    inline HandStatistics* getRHand() { return rHand_; }
    inline HandDetector* getHandDetector() { return hand_detector_; }

    void setCalculateHandStatistics(bool b);
    void setCalculateHandPoints(bool b);
    inline void setTrackingSkeleton(bool b) { tracking_skeleton_ = b; }
    
    inline void setDepthColoring(int val) { depth_coloring_ = val; }

    double fps() { return fps_; }
    uint64_t getFrameNumber() { return frame_number_; }

  private:
    bool SaveOnCalibration_;    
    
    // Kinect nodes
    xn::DepthGenerator depth_generator_;
    xn::ImageGenerator image_generator_;
    xn::UserGenerator user_generator_;
    
    // Temp data
    XnSkeletonJointPosition tmp_joint_uvd_;
    XnSkeletonJointPosition tmp_joint_world_;
    
    // Multi-threading
    std::recursive_mutex data_lock_;
    std::recursive_mutex context_lock_;
    std::mutex hand_finished_lock_;
    std::condition_variable hand_finished_cv_;
    uint32_t num_hands_finished_;
    std::thread kinect_thread_;
#ifdef MULTI_THREADED
    threading::ThreadPool* tpool_;
#endif
    
    // Data copied out of the kinect
    static const XnUInt16 width_ = 640;
    static const XnUInt16 height_ = 480;
    XnRGB24Pixel* rgb;
#ifdef DELAY_RGB
    XnRGB24Pixel* rgb_next_frame;
#endif
    XnDepthPixel* depth;
    XnPoint3D* cloud_pts_uvd_;
    XnPoint3D* cloud_pts_world_;
    XnUInt16 depth_md_yoffset_;
    XnUInt16 depth_md_yres_;
    XnUInt16 depth_md_xoffset_;
    XnUInt16 depth_md_xres_;
    uint64_t frame_number_;
    double fps_;
    bool* joints_found_;
    XnPoint3D* joints_;
    XnPoint3D* joints_projected_;
    
    unsigned char* depth_image_;
    float max_depth_;
    
    // Hand data and data collection for machine learning
    HandStatistics* lHand_;
    HandStatistics* rHand_;
    HandDetector* hand_detector_;
    // uint8_t* red_pixels_;
    // uint8_t* red_pixels_tmp_;
    // XnRGB24Pixel* hsv_;
    
    // Status structures
    bool kinect_running_;
    char status_str_[256];
    int num_users_found_;
    int num_users_tracked_;
    
    int depth_coloring_;
    
    xn::Context context;

    bool calculate_hand_statistics_;
    bool calculate_hand_points_;

    bool tracking_skeleton_;
    
    Clock* clk_;
    
    // MAIN UPDATE THREAD:
    void kinectUpdateThread();
    
    bool Init();
    bool InitUserGenerator();
    static void XN_CB User_NewUser(xn::UserGenerator& generator, 
                                   XnUserID nId, void* pCookie);
    static void XN_CB User_LostUser(xn::UserGenerator& generator, 
                                    XnUserID nId, void* pCookie);
    static void XN_CB User_PoseDetected(xn::PoseDetectionCapability& capability,
                                        const XnChar* strPose, XnUserID nId,
                                        void* pCookie);
    static void XN_CB User_CalStart(xn::SkeletonCapability& capability, 
                                    XnUserID nId, void* pCookie);
    static void XN_CB User_CalFinish(xn::SkeletonCapability& capability, 
                                     XnUserID nId, XnCalibrationStatus eStatus, 
                                     void* pCookie);
    
    void addLicense(const std::string& sVendor, const std::string& sKey);
    void logErrors(xn::EnumerationErrors& rErrors);
    void enableLogging();
    void updateJoint(XnUserID id, XnSkeletonJoint joint);
    void drawLimb(XnSkeletonJoint a, XnSkeletonJoint b);
    static void processHand(HandStatistics* hand, XnPoint3D* pts_world,
                            XnPoint3D* pts_cloud, XnPoint3D* ptHand_world, 
                            XnPoint3D* ptHand_uvd, bool calculate_hand_points_, 
                            bool calculate_hand_statistics_);
    // void getRedPixels();
  };
  
#ifndef EPSILON
  #define EPSILON 0.000001f
#endif
  
};  // namespace kinect

#endif  // KINECT_INTERFACE_H
