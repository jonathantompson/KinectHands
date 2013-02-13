//
//  kinect_interface.cpp
//
//  Source origionally from OpenNI libraries, then modified by Otavio B. for
//  Computer Vision class code, then adapted by Ken Perlin's lab for KinectHands

#include <mutex>
#include <thread>
#include <string>
#include <fstream>
#include <iostream>
#include "rendering/opengl_include.h"
#include "XnLog.h"
#include "kinect_interface.h"
#include "hand_statistics.h"
#include "threading/thread_pool.h"
#include "threading/callback.h"
#include "threading/thread.h"
#include "clock/clock.h"
#include "hand_detector.h"
#include "image_util.h"

#ifdef _WIN32
  #ifndef snprintf
    #define snprintf _snprintf_s
  #endif
#endif

// Some macro defines copied from ofxOpenNIMacros.h
#define SHOW_RC(rc, what)           \
printf("%s status: %s\n", what, xnGetStatusString(rc));

#define CHECK_RC(rc, what)           \
if (rc != XN_STATUS_OK)            \
{                 \
printf("%s failed: %s\n", what, xnGetStatusString(rc));   \
} else {               \
printf("%s succeed: %s\n", what, xnGetStatusString(rc));  \
}
#define SAMPLE_XML_PATH "SamplesConfig.xml"

using std::cout;
using std::endl;
using std::thread;
using math::Float3;
using threading::ThreadPool;
using threading::MakeCallableOnce;
using threading::MakeCallableMany;
using threading::Callback;
using threading::MakeThread;

#define CALIBRATION_POSE "Psi"

namespace kinect {
  
  // Mildly, hacky.  We want non-static functions (the retistered callbacks) to
  // reference our class.  OK, since KinectInterface is a singleton.
  KinectInterface* ptr_static;  
  bool g_bNeedPose = false;
  XnChar g_strPose[20] = "";
  
  KinectInterface::KinectInterface(bool tracking_on) {
    tracking_skeleton_ = tracking_on;
    cout << "Initializing KinectInterface...\n";
    if (!Init())
      throw std::runtime_error("Kinect Initialization failed!");
    cout << "Kinect initialization finished\n";
    ptr_static = this;
    
    // **************************************************
    // INIT SOME DATA STRUCTURES
    rgb = new XnRGB24Pixel[width_*height_];
#ifdef DELAY_RGB
    rgb_next_frame = new XnRGB24Pixel[width_*height_];
#endif
    depth = new XnDepthPixel[width_*height_];
    cloud_pts_uvd_ = new XnPoint3D[width_*height_];
    cloud_pts_world_ = new XnPoint3D[width_*height_];
    // cloud_colors_ = new unsigned char[width_*height_*4];  // r,g,b,a
    SaveOnCalibration_ = false;
    lHand_ = new HandStatistics(this, HandLHand);
    rHand_ = new HandStatistics(this, HandRHand);
    joints_ = new XnPoint3D[SKEL_NJOINTS];
    joints_projected_ = new XnPoint3D[SKEL_NJOINTS];
    joints_found_ = new bool[SKEL_NJOINTS];
    depth_image_ = new XnUInt8[width_*height_*4];
    for (int i = 0; i < SKEL_NJOINTS; i++) {
      joints_found_[i] = false;
    }
    num_users_found_ = 0;
    num_users_tracked_ = 0;
    depth_coloring_ = COLORING_GREY;
    calculate_hand_statistics_ = true;
    calculate_hand_points_ = true;
    frame_number_ = 0;
    fps_ = 0;
#ifdef MULTI_THREADED
    tpool_ = NULL;
#endif
    clk_ = new Clock();
    hand_detector_ = new HandDetector(width_, height_);
    
    HandStatistics::initHandStatistics();
    //  Now spawn the Kinect Update Thread
    Callback<void>* threadBody = MakeCallableOnce(&KinectInterface::kinectUpdateThread, this);
    kinect_thread_ = MakeThread(threadBody);
    kinect_running_ = true;
  }
  
  KinectInterface::~KinectInterface() {
    // Clean up
    image_generator_.Release();
    depth_generator_.Release();
    user_generator_.Release();
    context.Release();
    delete[] rgb;
#ifdef DELAY_RGB
    delete[] rgb_next_frame;
#endif
    delete[] depth;
    delete[] cloud_pts_uvd_;
    delete[] cloud_pts_world_;
    // delete[] cloud_colors_;
    delete rHand_;
    delete lHand_;
    delete[] joints_;
    delete[] joints_projected_;
    delete[] joints_found_;
    delete[] depth_image_;
#ifdef MULTI_THREADED
    if (tpool_) {
      delete tpool_;  // Assume: stop() has already been called
      tpool_ = NULL;
    }
#endif
    delete clk_;
    if (hand_detector_) {
      delete hand_detector_;
      hand_detector_ = NULL;
    }
  }
  
  
  // ************************************************************
  // XN Callbacks
  // ************************************************************
  // Callback: New user was detected
  void XN_CB KinectInterface::User_NewUser(xn::UserGenerator& generator, 
                                           XnUserID nId, void* pCookie) {
    printf("New User %d\n", nId);
    ptr_static->num_users_found_++;
    if (ptr_static->num_users_found_ <= 1) {  // ONLY TRACK ONE USER
      if (ptr_static->LoadCalibration() == false) {
        // New user found
        if (g_bNeedPose) {
          ptr_static->user_generator_.GetPoseDetectionCap().StartPoseDetection(
                                                                      g_strPose,
                                                                      nId);
        } else {
          ptr_static->user_generator_.GetSkeletonCap().RequestCalibration(nId, 
                                                                          TRUE);
        }
      } else {
        ptr_static->num_users_tracked_++;
      }
    }
  }
  // Callback: An existing user was lost
  void XN_CB KinectInterface::User_LostUser(xn::UserGenerator& generator, 
                                            XnUserID nId, void* pCookie) {
    printf("Lost user %d\n", nId); 
    ptr_static->user_generator_.GetSkeletonCap().Reset(nId);
    ptr_static->num_users_found_--;
//    if (ptr_static->num_users_found_ == 0) {
//      ptr_static->ResetTracking(ptr_static->tracking_skeleton_);
//    }
  }
  // Callback: Detected a pose
  void XN_CB KinectInterface::User_PoseDetected(xn::PoseDetectionCapability& 
                                                capability, 
                                                const XnChar* strPose, 
                                                XnUserID nId, void* pCookie) {
    printf("Pose %s detected for user %d\n", strPose, nId);
    ptr_static->user_generator_.GetPoseDetectionCap().StopPoseDetection(nId);
    ptr_static->user_generator_.GetSkeletonCap().RequestCalibration(nId, TRUE);
  }
  // Callback: Started calibration
  void XN_CB KinectInterface::User_CalStart(xn::SkeletonCapability& capability, 
                                            XnUserID nId, void* pCookie) {
    printf("Calibration started for user %d\n", nId);
  }
  // Callback: Finished calibration
  void XN_CB KinectInterface::User_CalFinish(xn::SkeletonCapability& capability,
                                             XnUserID nId, 
                                             XnCalibrationStatus eStatus, 
                                             void* pCookie) {
    if (eStatus == XN_CALIBRATION_STATUS_OK) {
      // Calibration succeeded
      printf("Calibration complete, start tracking user %d\n", nId);  
      ptr_static->user_generator_.GetSkeletonCap().StartTracking(nId);
      ptr_static->num_users_tracked_++;
    } else {
      // Calibration failed
      printf("Calibration failed for user %d\n", nId);
      if (g_bNeedPose) {
        ptr_static->user_generator_.GetPoseDetectionCap().StartPoseDetection(
                                                                      g_strPose,
                                                                      nId);
      } else {
        ptr_static->user_generator_.GetSkeletonCap().RequestCalibration(nId, 
                                                                        TRUE);
      }
    }
  }
  
  // ************************************************************
  // Calibration data load / save
  // ************************************************************
  
  // Save calibration to file
  const XnChar* calibration_file = "UserCalibration.bin";
  void KinectInterface::SaveCalibration() {
    XnUserID aUserIDs[20] = {0};
    XnUInt16 nUsers = 20;
    user_generator_.GetUsers(aUserIDs, nUsers);
    if (nUsers == 0) {
      printf("No user's detected yet --> Move around!  ");
      printf("Will save when calibration has finished.\n");
    } else {
      for (int i = 0; i < nUsers; ++i) {
        // Find a user who is already calibrated
        if (user_generator_.GetSkeletonCap().IsCalibrated(aUserIDs[i])) {
          // Save user's calibration to file
          XnStatus rc = 
          user_generator_.GetSkeletonCap().SaveCalibrationDataToFile(
                                                              aUserIDs[i], 
                                                              calibration_file);
          SHOW_RC(rc, "SaveCalibrationDataToFile() ");
          break;
        } else {
          printf("User %d is not yet calibrated, no data to save!  ", i);
          printf("Will save when calibration has finished.\n");
        }
      }
    }
  }
  
  // Load calibration from file
  bool KinectInterface::LoadCalibration() {
    XnUserID aUserIDs[20] = {0};
    XnUInt16 nUsers = 20;
    user_generator_.GetUsers(aUserIDs, nUsers);
    if (nUsers == 0) {
      printf("No user's detected yet --> Move around!\n");
      return false;
    } else {
      for (int i = 0; i < nUsers; ++i) {
        // Find a user who isn't calibrated or currently in pose
        if (user_generator_.GetSkeletonCap().IsCalibrated(aUserIDs[i])) {
          continue;
        }
        if (user_generator_.GetSkeletonCap().IsCalibrating(aUserIDs[i])) {
          continue;
        }
        
        // Load user's calibration from file
        XnStatus rc = 
        user_generator_.GetSkeletonCap().LoadCalibrationDataFromFile(
                                                            aUserIDs[i], 
                                                            calibration_file);
        SHOW_RC(rc, "LoadCalibrationDataFromFile() ");
        if (rc == XN_STATUS_OK) {
          // Make sure state is coherent
          user_generator_.GetPoseDetectionCap().StopPoseDetection(aUserIDs[i]);
          user_generator_.GetSkeletonCap().StartTracking(aUserIDs[i]);
          return true;
        } else {
          return false;
        }
        break;
      }
    }
    return false;
  }
  
  
  void KinectInterface::ResetTracking(bool tracking_on) {
    context_lock_.lock();
    data_lock_.lock();

    // This is a bit of a hack.  We need to completely rebuild the user gen.
    // since OpenNI doesn't have a Reset switch :-(
    std::cout << "Resetting tracking.  Please wait... " << endl;
    
    // Turn off the generator
    num_users_tracked_ = 0;
    num_users_found_ = 0;
    std::fill(joints_found_, joints_found_ + SKEL_NJOINTS, false);
    std::cout << "   --> Releasing user_generator_" << endl;
    user_generator_.Release();
    
    // Toggle tracking and turn it back on;
    tracking_skeleton_ = tracking_on;
    InitUserGenerator();
    std::cout << "Tracking Reset" << endl;
    
    for (uint32_t i = 0; i < SKEL_NJOINTS; i++) {
      joints_found_[i] = false;
    }
    rHand_->resetHand();
    lHand_->resetHand();
    
    hand_detector_->reset();

    data_lock_.unlock();
    context_lock_.unlock();
  }
  
  // ************************************************************
  // Initialization
  // ************************************************************
  
  bool KinectInterface::Init() {
    XnStatus nRetVal = XN_STATUS_OK;
    xn::EnumerationErrors errors;
    
    // INIT AN OPENNI CONTEXT
    nRetVal = context.Init();
    if (nRetVal != XN_STATUS_OK) {
      logErrors(errors);
    }
    CHECK_RC(nRetVal, "context.Init()");
    
    static std::string vendor("PrimeSense");
    static std::string key("0KOIk2JeIBYClPWVnMoRKn5cdY4=");
    addLicense(vendor, key);
    printf("\n");
    // enableLogging();
    
    // INIT AN OPENNI DEPTHGENERATOR
    XnMapOutputMode map_mode;
    nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth_generator_);
    // CHECK_RC(nRetVal, "Retrieving depth generator");
    if (nRetVal == XN_STATUS_OK) {
      // found the depth generator so set map_mode from it
      depth_generator_.GetMapOutputMode(map_mode);
    } else {
      nRetVal = depth_generator_.Create(context);
      CHECK_RC(nRetVal, "Creating depth generator");
      
      if (nRetVal != XN_STATUS_OK) 
        return false;
      
      // make new map mode -> default to 640 x 480 @ 30fps
      map_mode.nXRes = XN_VGA_X_RES;
      map_mode.nYRes = XN_VGA_Y_RES;
      map_mode.nFPS  = 30;
      
      depth_generator_.SetMapOutputMode(map_mode);
    }

    max_depth_ = depth_generator_.GetDeviceMaxDepth();
    depth_generator_.StartGenerating(); 
    
    // INIT AN OPENNI IMAGEGENERATOR
    nRetVal = context.FindExistingNode(XN_NODE_TYPE_IMAGE, image_generator_);
    //  CHECK_RC(nRetVal, "Retrieving image generator");
    if (nRetVal == XN_STATUS_OK) {
      // found the image generator so set map_mode from it
      image_generator_.GetMapOutputMode(map_mode);
    } else {
      nRetVal = image_generator_.Create(context);
      CHECK_RC(nRetVal, "Creating image generator");
      
      if (nRetVal != XN_STATUS_OK) 
        return false;
      
      // make new map mode -> default to 640 x 480 @ 30fps
      map_mode.nXRes = XN_VGA_X_RES;
      map_mode.nYRes = XN_VGA_Y_RES;
      map_mode.nFPS  = 30;
      
      image_generator_.SetMapOutputMode(map_mode);
      image_generator_.GetMapOutputMode(map_mode);  // Get it back, just in case
      if (width_ != map_mode.nXRes || height_ != map_mode.nYRes) {
        throw std::runtime_error(std::string("ERROR: width_ != ") +
          std::string("map_mode.nXRes || height_ != map_mode.nYRes"));
                              
      }
    }
    
    image_generator_.StartGenerating(); 
    
    bool OK = InitUserGenerator();
    if (OK == FALSE) {
      return false;
    }
    
    // Register the depth and rgb images
    depth_generator_.GetAlternativeViewPointCap().SetViewPoint(image_generator_);
    depth_generator_.GetFrameSyncCap().FrameSyncWith(image_generator_);
    image_generator_.GetFrameSyncCap().FrameSyncWith(depth_generator_);
    
    context.SetGlobalMirror(NI_MIRROR);
    
    // Start generating
    nRetVal = context.StartGeneratingAll();
    
    return true;
  }
  
  bool KinectInterface::InitUserGenerator() {
    // Only need a user generator if we're tracking the skeleton
    if (tracking_skeleton_) {
      XnStatus nRetVal;
      // INIT AN OPENNI USERGENERATOR
      nRetVal = context.FindExistingNode(XN_NODE_TYPE_USER, user_generator_);
      //  CHECK_RC(nRetVal, "Retrieving user generator");
      if (nRetVal != XN_STATUS_OK) {
        // if one doesn't exist then create user generator.
        nRetVal = user_generator_.Create(context);
        CHECK_RC(nRetVal, "Create user generator");

        if (nRetVal != XN_STATUS_OK) 
          return false;
      }

      XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete; 
      XnCallbackHandle hPoseDetected;
      if (!user_generator_.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
        printf("Supplied user generator doesn't support skeleton\n");
        return 1;
      }

      nRetVal = user_generator_.RegisterUserCallbacks(
        &KinectInterface::User_NewUser, &KinectInterface::User_LostUser, NULL, 
        hUserCallbacks);
      // CHECK_RC(nRetVal, "Register to user callbacks");
      nRetVal = user_generator_.GetSkeletonCap().RegisterToCalibrationStart(
        User_CalStart, NULL, hCalibrationStart);
      // CHECK_RC(nRetVal, "Register to calibration start");
      nRetVal = user_generator_.GetSkeletonCap().RegisterToCalibrationComplete(
        User_CalFinish, NULL, hCalibrationComplete);
      // CHECK_RC(nRetVal, "Register to calibration complete");

      if (user_generator_.GetSkeletonCap().NeedPoseForCalibration()) {
        g_bNeedPose = TRUE;
        if (!user_generator_.IsCapabilitySupported(
          XN_CAPABILITY_POSE_DETECTION)) {
            printf("Pose required, but not supported\n");
            return 1;
        }
        nRetVal = user_generator_.GetPoseDetectionCap().RegisterToPoseDetected(
          User_PoseDetected, NULL, hPoseDetected);
        // CHECK_RC(nRetVal, "Register to Pose Detected");
        user_generator_.GetSkeletonCap().GetCalibrationPose(g_strPose);
      }

      user_generator_.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
      user_generator_.GetSkeletonCap().SetSmoothing(SKELETON_SMOOTHING);

      user_generator_.StartGenerating();
    }

    return true;
  }
  
  // we need to programmatically add a license when playing back a recording
  // file otherwise the skeleton tracker will throw an error and not work
  void KinectInterface::addLicense(const std::string& sVendor, 
                                   const std::string& sKey) {
    XnLicense license = {0};
    XnStatus status = XN_STATUS_OK;
    
    status = xnOSStrNCopy(license.strVendor, sVendor.c_str(), sVendor.size(), 
                          sizeof(license.strVendor));
    if (status != XN_STATUS_OK) {
      printf("KinectInterface error creating license (vendor)\n");
      return;
    }
    
    status = xnOSStrNCopy(license.strKey, sKey.c_str(), sKey.size(), 
                          sizeof(license.strKey));
    if (status != XN_STATUS_OK) {
      printf("KinectInterface error creating license (key)\n");
      return;
    } 
    
    status = context.AddLicense(license);
    SHOW_RC(status, "AddLicense");
    
    // xnPrintRegisteredLicenses();
  }
  
  void KinectInterface::logErrors(xn::EnumerationErrors& rErrors) {
    for (xn::EnumerationErrors::Iterator it = rErrors.Begin(); 
        it != rErrors.End(); ++it) {
      XnChar desc[512];
      xnProductionNodeDescriptionToString(&it.Description(), desc, 512);
      printf("%s failed: %s\n", desc, xnGetStatusString(it.Error()));
    } 
  }
  
  void KinectInterface::enableLogging() {
    XnStatus result = xnLogSetConsoleOutput(true);
    SHOW_RC(result, "Set console output");
    
    result = xnLogSetSeverityFilter(XN_LOG_ERROR);
    SHOW_RC(result, "Set log level");
    
    xnLogSetMaskState(XN_LOG_MASK_ALL, TRUE);
  }
  
  void KinectInterface::updateJoint(XnUserID id, XnSkeletonJoint joint) {
    if (user_generator_.GetSkeletonCap().IsJointActive(joint)) {
      user_generator_.GetSkeletonCap().GetSkeletonJointPosition(
                                                              id, 
                                                              joint, 
                                                              tmp_joint_world_);
      if (tmp_joint_world_.fConfidence < 0.005f) {
        joints_found_[joint] = false;
      } else {
        joints_found_[joint] = true;
        joints_[joint].X = tmp_joint_world_.position.X;
        joints_[joint].Y = tmp_joint_world_.position.Y;
        joints_[joint].Z = tmp_joint_world_.position.Z;
        
        depth_generator_.ConvertRealWorldToProjective(1, 
          &tmp_joint_world_.position,  &tmp_joint_uvd_.position);
        
        joints_projected_[joint].X = tmp_joint_uvd_.position.X;
        joints_projected_[joint].Y = tmp_joint_uvd_.position.Y;
        joints_projected_[joint].Z = tmp_joint_uvd_.position.Z;
      }
    } else {
      joints_found_[joint] = false;
    }
  }
  
  // ************************************************************
  // Drawing
  // ************************************************************
  
  void KinectInterface::drawSkeletonPts() {
    GLboolean old_depthTest;
    glGetBooleanv(GL_DEPTH_TEST, &old_depthTest);
    glDisable(GL_DEPTH_TEST);
    GLfloat old_ps;
    glGetFloatv(GL_POINT_SIZE, &old_ps);
    glPointSize(12.0f);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);  // NOT using color array
    glVertexPointer(3, GL_FLOAT, 0, joints_projected_);
    glDrawArrays(GL_POINTS, 0, 8);
    glDisableClientState(GL_VERTEX_ARRAY);
    glPointSize(old_ps);
    if (old_depthTest)
      glEnable(GL_DEPTH_TEST);
  }
  
  void KinectInterface::drawLimb(XnSkeletonJoint a, XnSkeletonJoint b) {
    if (joints_found_[a] && joints_found_[b]) {
      glVertex2f(joints_projected_[a].X, joints_projected_[a].Y);
      glVertex2f(joints_projected_[b].X, joints_projected_[b].Y);
    }
  }
  
  void KinectInterface::drawSkeletonLines() {
    glBegin(GL_LINES);
    
    drawLimb(XN_SKEL_HEAD,            XN_SKEL_NECK);
    drawLimb(XN_SKEL_NECK,            XN_SKEL_TORSO);
    drawLimb(XN_SKEL_NECK,            XN_SKEL_LEFT_SHOULDER);
    drawLimb(XN_SKEL_LEFT_SHOULDER,   XN_SKEL_LEFT_ELBOW);
    drawLimb(XN_SKEL_LEFT_ELBOW,      XN_SKEL_LEFT_HAND);
    drawLimb(XN_SKEL_NECK,            XN_SKEL_RIGHT_SHOULDER);
    drawLimb(XN_SKEL_RIGHT_SHOULDER,  XN_SKEL_RIGHT_ELBOW);
    drawLimb(XN_SKEL_RIGHT_ELBOW,     XN_SKEL_RIGHT_HAND);
    drawLimb(XN_SKEL_TORSO,           XN_SKEL_LEFT_HIP);
    drawLimb(XN_SKEL_TORSO,           XN_SKEL_RIGHT_HIP);
    
    glEnd();
  }
  
  void KinectInterface::drawHandStatistics(bool draw_pts, bool draw_obb, 
                                           bool draw_finger) {
    lHand_->drawHandStatistics(draw_pts, draw_obb, draw_finger);
    rHand_->drawHandStatistics(draw_pts, draw_obb, draw_finger);
  }

  void KinectInterface::drawHandDetectorOBB() {
    data_lock_.lock();

    GLboolean old_depthTest;
    glGetBooleanv(GL_DEPTH_TEST, &old_depthTest);
    glDisable(GL_DEPTH_TEST);
    glLineWidth(1.0f);

    glBegin(GL_LINES);
    glColor3f(0.0f, 1.0f, 0.0f);
    for (uint32_t i = 0; i < hand_detector_->hands_uv_min().size(); i++) {
      glVertex2f(hand_detector_->hands_uv_min()[i][0] * DT_DOWNSAMPLE, 
        hand_detector_->hands_uv_min()[i][1] * DT_DOWNSAMPLE);  // top
      glVertex2f(hand_detector_->hands_uv_max()[i][0] * DT_DOWNSAMPLE, 
        hand_detector_->hands_uv_min()[i][1] * DT_DOWNSAMPLE);
      glVertex2f(hand_detector_->hands_uv_min()[i][0] * DT_DOWNSAMPLE, 
        hand_detector_->hands_uv_max()[i][1] * DT_DOWNSAMPLE);  // bottom
      glVertex2f(hand_detector_->hands_uv_max()[i][0] * DT_DOWNSAMPLE, 
        hand_detector_->hands_uv_max()[i][1] * DT_DOWNSAMPLE);
      glVertex2f(hand_detector_->hands_uv_min()[i][0] * DT_DOWNSAMPLE, 
        hand_detector_->hands_uv_min()[i][1] * DT_DOWNSAMPLE);  // left
      glVertex2f(hand_detector_->hands_uv_min()[i][0] * DT_DOWNSAMPLE, 
        hand_detector_->hands_uv_max()[i][1] * DT_DOWNSAMPLE);
      glVertex2f(hand_detector_->hands_uv_max()[i][0] * DT_DOWNSAMPLE, 
        hand_detector_->hands_uv_min()[i][1] * DT_DOWNSAMPLE);  // right
      glVertex2f(hand_detector_->hands_uv_max()[i][0] * DT_DOWNSAMPLE, 
        hand_detector_->hands_uv_max()[i][1] * DT_DOWNSAMPLE);
    }
    glEnd();

    glPointSize(4.0f);
    glBegin(GL_POINTS);
    glColor3f(0.0f, 0.0f, 1.0f);
    for (uint32_t i = 0; i < hand_detector_->hands_uvd().size(); i++) {
      glVertex2f(hand_detector_->hands_uvd()[i][0], 
        hand_detector_->hands_uvd()[i][1]);
    }
    glEnd();

    if (old_depthTest) {
      glEnable(GL_DEPTH_TEST);
    }

    data_lock_.unlock();
  }

  // Unsafe data access --> No lock taken on the data
  XnRGB24Pixel* KinectInterface::getRGBImageUnsafe() {
    return rgb;
  }
  
  // Unsafe data access --> No lock taken on the data
  XnDepthPixel* KinectInterface::getDepthRawUnsafe() {
    return depth;
  }
  
  // Unsafe data access --> No lock taken on the data
  XnUInt8* KinectInterface::getDepthImageUnsafe() {
    const XnDepthPixel *pDepth = depth;
    
    // copy depth into texture-map
    float max;
    for (XnUInt16 y = depth_md_yoffset_; 
         y < depth_md_yres_ + depth_md_yoffset_;
         y++) {
      unsigned char * texture = (unsigned char*)depth_image_ + y * 
      depth_md_xres_ * 4 + depth_md_xoffset_ * 4;
      for (XnUInt16 x = 0; x < depth_md_xres_; x++, pDepth++, texture += 4) {
        XnUInt8 red = 0;
        XnUInt8 green = 0;
        XnUInt8 blue = 0;
        XnUInt8 alpha = 255;
        
        XnUInt16 col_index;
        XnUInt16 tmp;
        
        switch (depth_coloring_) {
          case COLORING_BLUES:
            max = 256+255+255;
            col_index = (XnUInt16)(((*pDepth) / ( max_depth_ / max)));
            if ( col_index < 256 ) {
              blue = col_index;
              green = 0;
              red  = 0;
            } else if ( col_index < (256+255) ) {
              blue = 255;
              green = (col_index % 256) + 1;
              red  = 0;
            } else if ( col_index < (256+255+255) ) {
              blue = 255;
              green = 255;
              red  = (col_index % 256) + 1;
            } else {
              blue = 255;
              green = 255;
              red  = 255;
            }
            break;
          case COLORING_GREY:
            max = 255;  // half depth
            tmp = (XnUInt16)(((*pDepth) / ( max_depth_ / max)))<<4;
            red  = tmp;
            green = tmp;
            blue = tmp;
            break;
          case COLORING_INVERSE_RAW_DEPTH:
            if (*pDepth == 0) {
              tmp = 0;
            } else {
              tmp = 255 - static_cast<XnUInt16>(255*static_cast<float>(*pDepth) / ( max_depth_));
            }
            red  = tmp;
            green = tmp;
            blue = tmp;
            break;
        }
        texture[0] = red;
        texture[1] = green;
        texture[2] = blue;
        
        if (*pDepth == 0)
          texture[3] = 0;
        else
          texture[3] = alpha;
      } 
    }
    
    return depth_image_;
  }
  
  char* KinectInterface::getStatusMessage() {
    if (tracking_skeleton_) {
      snprintf(status_str_, sizeof(status_str_), "Users: %d, Tracked: %d, ", 
               num_users_found_, num_users_tracked_);
    } else {
      status_str_[0] = '\0';  // Empty string
    }
    return status_str_;
  }
  
  int KinectInterface::getNumUsersTracked() {
    return num_users_tracked_;
  }

  void KinectInterface::kinectUpdateThread() {
    double last_frame_time = clk_->getTime();
    double time_accum = 0;
    uint64_t last_frame_number = 0;
    
    // Spawn the thread pool, and setup callbacks for the worker threads
#ifdef MULTI_THREADED
    tpool_ = new ThreadPool(NUM_WORKER_THREADS);
    Callback<void>* lHand_update_cb = 
      MakeCallableMany(&HandStatistics::updateHand, lHand_);
    Callback<void>* rHand_update_cb = 
      MakeCallableMany(&HandStatistics::updateHand, rHand_);  
#endif

    while (kinect_running_) {
      // Update to next frame: Because of an odd performance bug, 
      // WaitAndUpdateAll gives very bad performance on Windows 7. 

      context_lock_.lock();
      // XnStatus nRetVal = context.WaitAndUpdateAll();
      // Better alignment if we wait on depth
      XnStatus nRetVal = context.WaitOneUpdateAll(depth_generator_);
      
      if (nRetVal != XN_STATUS_OK) {
        throw std::runtime_error("ERROR context.WaitOneUpdateAll() failed!");
      }

      // Lock the local data structure to copy over data
      data_lock_.lock();

      // Retrieve the RGB image and copy into local array
      xn::ImageMetaData image_md_;
      xn::DepthMetaData depth_md_;
      
#ifdef DELAY_RGB
      // But even waiting on depth above, the RGB is still mostly ahead.
      // Lag the RGB by one frame.
      memcpy(rgb, rgb_next_frame, sizeof(rgb[0])*width_*height_);
#endif

      image_generator_.GetMetaData(image_md_);
      const XnRGB24Pixel* pColor = image_md_.RGB24Data();
#ifdef DELAY_RGB
      memcpy(rgb_next_frame, pColor, sizeof(rgb_next_frame[0])*width_*height_);
#else 
      memcpy(rgb, pColor, sizeof(rgb[0])*width_*height_);
#endif

      // Retrieve the depth map and copy into local array
      depth_generator_.GetMetaData(depth_md_);
      const XnDepthPixel* pDepth = depth_md_.Data();
      memcpy(depth, pDepth, sizeof(depth[0])*width_*height_);
      depth_md_yoffset_ = depth_md_.YOffset(); 
      depth_md_yres_ = depth_md_.YRes();
      depth_md_xoffset_ = depth_md_.XOffset();
      depth_md_xres_ = depth_md_.XRes();

      // Now get the point cloud data
      int nIndex = 0;
      for (int nY = 0; nY < height_; nY += 1) {
        for (int nX = 0; nX < width_; nX += 1, nIndex += 1) {
          cloud_pts_uvd_[nIndex].X = nX;
          cloud_pts_uvd_[nIndex].Y = nY;
          cloud_pts_uvd_[nIndex].Z = pDepth[nIndex];
        }
      }
      // Now convert kinect space to world space (used for many calculations)
      depth_generator_.ConvertProjectiveToRealWorld(
        width_*height_, 
        reinterpret_cast<XnPoint3D*>(cloud_pts_uvd_),
        reinterpret_cast<XnPoint3D*>(cloud_pts_world_));

      // Now Get the skeleton
      if (tracking_skeleton_) {
        XnUserID aUsers[8];
        XnUInt16 nUsers = 8;
        user_generator_.GetUsers(aUsers, nUsers);

        if (nUsers > 0 && user_generator_.GetSkeletonCap().IsTracking(aUsers[0])) {
          updateJoint(aUsers[0], XN_SKEL_HEAD);
          updateJoint(aUsers[0], XN_SKEL_NECK);
          updateJoint(aUsers[0], XN_SKEL_TORSO);
          updateJoint(aUsers[0], XN_SKEL_LEFT_SHOULDER);
          updateJoint(aUsers[0], XN_SKEL_LEFT_ELBOW);
          updateJoint(aUsers[0], XN_SKEL_LEFT_HAND);
          updateJoint(aUsers[0], XN_SKEL_RIGHT_SHOULDER);
          updateJoint(aUsers[0], XN_SKEL_RIGHT_ELBOW);
          updateJoint(aUsers[0], XN_SKEL_RIGHT_HAND);
          updateJoint(aUsers[0], XN_SKEL_LEFT_HIP);
          updateJoint(aUsers[0], XN_SKEL_RIGHT_HIP);
        }
      } else {
        hand_detector_->findHands(reinterpret_cast<int16_t*>(depth),
          joints_found_[XN_SKEL_RIGHT_HAND], joints_found_[XN_SKEL_LEFT_HAND],
          &joints_projected_[XN_SKEL_RIGHT_HAND].X,  
          &joints_projected_[XN_SKEL_LEFT_HAND].X);
        if (joints_found_[XN_SKEL_RIGHT_HAND]) {
          depth_generator_.ConvertProjectiveToRealWorld(1, 
            &joints_projected_[XN_SKEL_RIGHT_HAND], &joints_[XN_SKEL_RIGHT_HAND]);  
        } else {
          rHand_->resetHand();
        }
        if (joints_found_[XN_SKEL_LEFT_HAND]) {
          depth_generator_.ConvertProjectiveToRealWorld(1, 
            &joints_projected_[XN_SKEL_LEFT_HAND], &joints_[XN_SKEL_LEFT_HAND]);   
        } else {
          lHand_->resetHand();
        }
          
      }

      frame_number_++;
      double frame_time = clk_->getTime();
      time_accum += (frame_time - last_frame_time);
      last_frame_time = frame_time;
      if (time_accum > 0.5) {
        fps_ = (frame_number_ - last_frame_number) / time_accum;
        time_accum = 0;
        last_frame_number = frame_number_;
      }
      
      if (calculate_hand_points_ || calculate_hand_statistics_) {
        if (tracking_skeleton_) {
          HandStatistics::setHandSearchRadius(BIG_HAND_RADIUS_SKELETON);
        } else {
          HandStatistics::setHandSearchRadius(BIG_HAND_RADIUS);
        }
        
#ifdef MULTI_THREADED
        num_hands_finished_ = 0;
        // While we still have the lock: put update hand requests on the 
        // thread pool work queue.
        uint32_t num_hands = 0;
        if (joints_found_[XN_SKEL_RIGHT_HAND]) {
          tpool_->addTask(rHand_update_cb);
          num_hands++;
        }
        if (joints_found_[XN_SKEL_LEFT_HAND]) {
          tpool_->addTask(lHand_update_cb);
          num_hands++;
        }
        
        // Wait until both hands are done
        std::unique_lock<std::mutex> unique_lock(hand_finished_lock_);
        while (num_hands_finished_ != num_hands) {  // check against spurious wakeups
          hand_finished_cv_.wait(unique_lock);
        }
        unique_lock.unlock();
#else
        // Just process hands sequentially
        if (joints_found_[XN_SKEL_RIGHT_HAND]) {
          rHand_->updateHand();
        }
        if (joints_found_[XN_SKEL_LEFT_HAND]) {
          lHand_->updateHand();
        }
#endif
        
      } else {
        // We're not tracking the hands at all.
        rHand_->resetHand();
        lHand_->resetHand();
      }
      
      
      // Unlock the local data structure since we've finished copying data
      data_lock_.unlock();

      context_lock_.unlock();
      std::this_thread::yield();

    }  // end while (app::App::app_running)
    printf("kinectUpdateThread shutting down...\n");
#ifdef MULTI_THREADED
    tpool_->stop();  // Blocking until all threads have shut down
    
    // clean up callbacks
    delete rHand_update_cb;
    delete lHand_update_cb;
#endif
  }

  void KinectInterface::setCalculateHandStatistics(bool b) { 
    calculate_hand_statistics_ = b; 
    if (calculate_hand_statistics_ == true) {
      calculate_hand_points_ = true;
    }
  }
  
  void KinectInterface::setCalculateHandPoints(bool b) { 
    calculate_hand_points_ = b; 
  }
  
  void KinectInterface::join() {
    kinect_thread_.join();
  }
  
  void KinectInterface::requestShutdown() {
    printf("shutdown requested\n");
    kinect_running_ = false;
  }
  
  bool KinectInterface::kinect_running() {
    return kinect_running_;
  }
 
}  // namespace kinect


