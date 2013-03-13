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
#include "kinect_interface/kinect_interface.h"
#include "kinect_interface/hand_detector/hand_detector.h"
#include "jtil/jtil.h"
#include "jtil/clk/clk.h"
#include "jtil/image_util/image_util.h"
#include "jtil/threading/callback.h"
#include "jtil/threading/thread.h"
#include "jtil/settings/settings_manager.h"
#include "jtil/exceptions/wruntime_error.h"
#include "kinect_interface/open_ni_funcs.h"
#include "kinect_interface/depth_images_io.h"
#include "kinect_interface/hand_net/hand_net.h"

#include "XnCppWrapper.h"
#include "XnVNite.h"
#include "XnLog.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }

#ifdef _WIN32
#ifndef snprintf
#define snprintf _snprintf_s
#endif
#endif

// Some macro defines copied from ofxOpenNIMacros.h
#define SHOW_RC(rc, what)  \
  std::cout << what << " status: " << xnGetStatusString(rc) << std::endl;

#define CHECK_RC(rc, what)  \
  if (rc != XN_STATUS_OK) {  \
    std::cout << what << " failed: " << xnGetStatusString(rc) << std::endl;  \
  } else {  \
    std::cout << what << " succeed: " << xnGetStatusString(rc) << std::endl;  \
  }
#define CALIBRATION_POSE "Psi"
#define CALIBRATION_FILE "UserCalibration.bin"

using std::cout;
using std::endl;
using std::thread;
using jtil::math::Float3;
using jtil::threading::Callback;
using jtil::threading::MakeThread;
using jtil::threading::MakeCallableOnce;
using jtil::clk::Clk;
using namespace kinect_interface::hand_detector;
using namespace kinect_interface::hand_net;

namespace kinect_interface {
  using hand_detector::HandDetector;

  KinectInterface* KinectInterface::g_kinect_;  
  bool g_bNeedPose = false;
  XnChar g_strPose[20] = "";

  KinectInterface::KinectInterface() {
    context_ = NULL;
    dg_ = NULL;
    ig_ = NULL;
    ug_ = NULL;
    clk_ = NULL;
    hand_detector_ = NULL;
    image_io_ = NULL;
    hand_net_ = NULL;

    if (g_kinect_) {
      throw std::wruntime_error("KinectInterface::KinectInterface() - ERROR: "
        "multiple kinects not yet supported!");
    }
    GET_SETTING("track_openni_skeleton", bool, tracking_skeleton_);
    cout << "Initializing KinectInterface..." << endl;
    if (!init()) {
      throw std::wruntime_error("Kinect Initialization failed!");
    }
    cout << "Kinect initialization finished"  << endl;
    g_kinect_ = this;

    for (int i = 0; i < SKEL_NJOINTS; i++) {
      joints_found_[i] = false;
    }
    num_users_found_ = 0;
    num_users_tracked_ = 0;
    frame_number_ = 0;
    fps_ = 0;

    //  Now spawn the Kinect Update Thread
    kinect_running_ = true;
    Callback<void>* threadBody = MakeCallableOnce(
      &KinectInterface::kinectUpdateThread, this);
    kinect_thread_ = MakeThread(threadBody);
  }

  KinectInterface::~KinectInterface() {
    // Clean up
    if (ig_) { ig_->Release(); }
    if (dg_) { dg_->Release(); }
    if (ug_) { ug_->Release(); }
    if (context_) { context_->Release(); }
    SAFE_DELETE(ig_);
    SAFE_DELETE(dg_);
    SAFE_DELETE(ug_);
    SAFE_DELETE(context_);
    SAFE_DELETE(clk_);
    SAFE_DELETE(hand_detector_);
    SAFE_DELETE(image_io_);
    SAFE_DELETE(hand_net_);
  }

  // ************************************************************
  // Initialization
  // ************************************************************

  bool KinectInterface::init() {
    clk_ = new Clk();
    hand_detector_ = new HandDetector();
    hand_detector_->init(src_width, src_height);

    hand_net_ = new HandNet();
    hand_net_->loadFromFile("./data/handmodel.net.convnet");

    image_io_ = new DepthImagesIO();
    image_io_->LoadCompressedImageWithRedHands("./kinect_image.bin", 
      depth_from_file_, labels_from_file_, rgb_from_file_, NULL);
    DepthImagesIO::convertSingleImageToXYZ(pts_world_from_file_,
      depth_from_file_);

    //// TEMP CODE:
    //jtil::file_io::SaveArrayToFile<int16_t>(depth_from_file_, 640*480, 
    //  "./HandNets/depth_from_file.bin");
    //float frac = 6.4f;
    //float* d = new float[src_dim];
    //float* d_down = new float[src_dim];
    //float* d_up = new float[4096 * 3072];
    //float* d_temp = new float[src_dim/4];
    //for (uint32_t i = 0; i < src_dim; i++) { d[i] = (float)depth_from_file_[i]; }
    //jtil::image_util::FracDownsampleImageBilinear<float>(d_down, d, src_width,
    //  src_height, frac, d_temp);
    //jtil::file_io::SaveArrayToFile<float>(d_down, 100*75, 
    //  "./HandNets/kinect_depth_image_uncompressed_down_float.bin");
    //jtil::image_util::FracUpsampleImageBilinear<float>(d_up, d, src_width,
    //  src_height, frac);
    //jtil::file_io::SaveArrayToFile<float>(d_up, 4096 * 3072, 
    //  "./HandNets/kinect_depth_image_uncompressed_up_float.bin");
    //jtil::image_util::FracDownsampleImageSAT<float>(d_down, 0, 0, 100, 75, 100,
    //  d, 0, 0, src_width, src_height, src_width);  // Destroys d
    //jtil::file_io::SaveArrayToFile<float>(d_down, 100*75, 
    //  "./HandNets/kinect_depth_image_uncompressed_down_float2.bin");
    //delete[] d;
    //delete[] d_down;
    //delete[] d_up;
    //delete[] d_temp;
    //// END TEMP CODE

    XnStatus nRetVal = XN_STATUS_OK;
    xn::EnumerationErrors errors;

    // INIT AN OPENNI CONTEXT
    context_ = new xn::Context();
    nRetVal = context_->Init();
    if (nRetVal != XN_STATUS_OK) {
      logErrors(errors);
    }
    CHECK_RC(nRetVal, "context.Init()");

    static std::string vendor("PrimeSense");
    static std::string key("0KOIk2JeIBYClPWVnMoRKn5cdY4=");
    addLicense(vendor, key);
    cout << endl;
    enableLogging();

    // INIT AN OPENNI DEPTHGENERATOR
    XnMapOutputMode map_mode;
    dg_ = new xn::DepthGenerator();
    nRetVal = context_->FindExistingNode(XN_NODE_TYPE_DEPTH, *dg_);
    if (nRetVal == XN_STATUS_OK) {
      // found the depth generator so set map_mode from it
      dg_->GetMapOutputMode(map_mode);
    } else {
      nRetVal = dg_->Create(*context_);
      CHECK_RC(nRetVal, "Creating depth generator");

      if (nRetVal != XN_STATUS_OK) 
        return false;

      // make new map mode -> default to 640 x 480 @ 30fps
      map_mode.nXRes = XN_VGA_X_RES;
      map_mode.nYRes = XN_VGA_Y_RES;
      map_mode.nFPS  = 30;

      dg_->SetMapOutputMode(map_mode);
    }

    max_depth_ = dg_->GetDeviceMaxDepth();
    dg_->StartGenerating(); 

    // INIT AN OPENNI IMAGEGENERATOR
    ig_ = new xn::ImageGenerator();
    nRetVal = context_->FindExistingNode(XN_NODE_TYPE_IMAGE, *ig_);
    if (nRetVal == XN_STATUS_OK) {
      // found the image generator so set map_mode from it
      ig_->GetMapOutputMode(map_mode);
    } else {
      nRetVal = ig_->Create(*context_);
      CHECK_RC(nRetVal, "Creating image generator");

      if (nRetVal != XN_STATUS_OK) 
        return false;

      // make new map mode -> default to 640 x 480 @ 30fps
      map_mode.nXRes = XN_VGA_X_RES;
      map_mode.nYRes = XN_VGA_Y_RES;
      map_mode.nFPS  = 30;

      ig_->SetMapOutputMode(map_mode);
      ig_->GetMapOutputMode(map_mode);  // Get it back, just in case
      if (src_width != map_mode.nXRes || src_height != map_mode.nYRes) {
        throw std::wruntime_error("INTERNAL ERROR: kinect image dimensions "
          "aren't what we were expecting!");
      }
    }

    ig_->StartGenerating(); 

    ug_ = new xn::UserGenerator();
    bool OK = initUserGenerator();
    if (OK == FALSE) {
      return false;
    }

    // Register the depth and rgb images
    dg_->GetAlternativeViewPointCap().SetViewPoint(*ig_);
    dg_->GetFrameSyncCap().FrameSyncWith(*ig_);
    ig_->GetFrameSyncCap().FrameSyncWith(*dg_);

    context_->SetGlobalMirror(MIRROR);

    // Start generating
    nRetVal = context_->StartGeneratingAll();

    return true;
  }

  bool KinectInterface::initUserGenerator() {
    // Only need a user generator if we're tracking the skeleton
    if (tracking_skeleton_) {
      XnStatus nRetVal;
      // INIT AN OPENNI USERGENERATOR
      nRetVal = context_->FindExistingNode(XN_NODE_TYPE_USER, *ug_);
      if (nRetVal != XN_STATUS_OK) {
        // if one doesn't exist then create user generator.
        nRetVal = ug_->Create(*context_);
        CHECK_RC(nRetVal, "Create user generator");

        if (nRetVal != XN_STATUS_OK) 
          return false;
      }

      XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete; 
      XnCallbackHandle hPoseDetected;
      if (!ug_->IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
        cout << "Supplied user generator doesn't support skeleton" << endl;
        return 1;
      }

      nRetVal = ug_->RegisterUserCallbacks(newUser, lostUser, NULL, 
        hUserCallbacks);
      nRetVal = ug_->GetSkeletonCap().RegisterToCalibrationStart(
        calStart, NULL, hCalibrationStart);
      nRetVal = ug_->GetSkeletonCap().RegisterToCalibrationComplete(
        calFinish, NULL, hCalibrationComplete);

      if (ug_->GetSkeletonCap().NeedPoseForCalibration()) {
        g_bNeedPose = TRUE;
        if (!ug_->IsCapabilitySupported(
          XN_CAPABILITY_POSE_DETECTION)) {
            cout << "Pose required, but not supported" << endl;
            return 1;
        }
        nRetVal = ug_->GetPoseDetectionCap().RegisterToPoseDetected(
          poseDetected, NULL, hPoseDetected);
        ug_->GetSkeletonCap().GetCalibrationPose(g_strPose);
      }

      ug_->GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
      ug_->GetSkeletonCap().SetSmoothing(SKELETON_SMOOTHING);

      ug_->StartGenerating();
    }

    return true;
  }

  // we need to programmatically add a license when playing back a recording
  // file otherwise the skeleton tracker will throw an error and not work
  void KinectInterface::addLicense(const std::string& sVendor, 
    const std::string& sKey) {
      XnLicense license = {0};
      XnStatus status = XN_STATUS_OK;

      status = xnOSStrNCopy(license.strVendor, sVendor.c_str(), 
        (XnUInt32)sVendor.size(), (XnUInt32)sizeof(license.strVendor));
      if (status != XN_STATUS_OK) {
        cout << "KinectInterface error creating license (vendor)" << endl;
        return;
      }

      status = xnOSStrNCopy(license.strKey, sKey.c_str(), 
        (XnUInt32)sKey.size(), (XnUInt32)sizeof(license.strKey));
      if (status != XN_STATUS_OK) {
        cout << "KinectInterface error creating license (key)" << endl;
        return;
      } 

      status = context_->AddLicense(license);
      SHOW_RC(status, "AddLicense");

      // xnPrintRegisteredLicenses();
  }

  void KinectInterface::enableLogging() {
    XnStatus result = xnLogSetConsoleOutput(true);
    SHOW_RC(result, "Set console output");

    result = xnLogSetSeverityFilter(XN_LOG_ERROR);
    SHOW_RC(result, "Set log level");

    xnLogSetMaskState(XN_LOG_MASK_ALL, TRUE);
  }

  void KinectInterface::updateJoint(const unsigned int user_id, 
    const int32_t joint /*XnUserID id, XnSkeletonJoint joint*/) {
    const XnUserID xn_user_id = (const XnUserID)user_id;
    const XnSkeletonJoint xn_joint = (const XnSkeletonJoint)joint;

    XnSkeletonJointPosition joint_pos_world;
    if (ug_->GetSkeletonCap().IsJointActive(xn_joint)){
      ug_->GetSkeletonCap().GetSkeletonJointPosition(user_id, 
        xn_joint, joint_pos_world);
      if (joint_pos_world.fConfidence < 0.005f) {
        joints_found_[joint] = false;
      } else {
        joints_found_[joint] = true;
        joints_world_[joint*3] = joint_pos_world.position.X;
        joints_world_[joint*3+1] = joint_pos_world.position.Y;
        joints_world_[joint*3+2] = joint_pos_world.position.Z;
        OpenNIFuncs::xnConvertRealWorldToProjective(1, &joints_world_[joint*3],
          &joints_uvd_[joint*3]);
      }
    } else {
      joints_found_[joint] = false;
    }
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

    while (kinect_running_) {
      // Update to next frame: Because of an odd performance bug, 
      // WaitAndUpdateAll gives very bad performance on Windows 7. 

      context_lock_.lock();
      // XnStatus nRetVal = context_->WaitAndUpdateAll();
      // Better alignment if we wait on depth
      XnStatus nRetVal = context_->WaitOneUpdateAll(*dg_);

      if (nRetVal != XN_STATUS_OK) {
        throw std::wruntime_error("ERROR WaitOneUpdateAll() failed!");
      }

      // Lock the local data structure to copy over data
      data_lock_.lock();

      // Retrieve the RGB image and copy into local array
      xn::ImageMetaData image_md_;
      xn::DepthMetaData depth_md_;

      ig_->GetMetaData(image_md_);
      const XnRGB24Pixel* pColor = image_md_.RGB24Data();
      memcpy(rgb_, pColor, sizeof(rgb_[0]) * src_dim * 3);

      // Retrieve the depth map and copy into local array
      dg_->GetMetaData(depth_md_);
      const XnDepthPixel* pDepth = depth_md_.Data();
      memcpy(depth_, pDepth, sizeof(depth_[0]) * src_dim);

      // Now get the point cloud data
      for (int v = 0, i = 0; v < src_height; v++) {
        for (int u = 0; u < src_width; u++, i++) {
          pts_uvd_[i * 3] = (float)u;
          pts_uvd_[i * 3 + 1] = (float)v;
          pts_uvd_[i * 3 + 2] = (float)pDepth[i];
        }
      }

      // Now convert kinect space to world space
      OpenNIFuncs::xnConvertProjectiveToRealWorld(src_dim, pts_uvd_, 
        pts_world_);

      bool use_depth_from_file;
      GET_SETTING("use_depth_from_file", bool, use_depth_from_file);
      if (use_depth_from_file) {
        memcpy(depth_, depth_from_file_, sizeof(depth_[0]) * src_dim);
        memcpy(labels_, labels_from_file_, sizeof(labels_[0]) * src_dim);
        memcpy(rgb_, rgb_from_file_, sizeof(rgb_[0]) * src_dim * 3);
        memcpy(pts_world_, pts_world_from_file_, 
          sizeof(pts_world_[0]) * src_dim * 3);
      }

      // Now Get the skeleton
      if (tracking_skeleton_) {
        XnUserID aUsers[8];
        XnUInt16 nUsers = 8;
        ug_->GetUsers(aUsers, nUsers);

        if (nUsers > 0 && ug_->GetSkeletonCap().IsTracking(aUsers[0])) {
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
      }

      bool detect_hands, detect_pose;
      GET_SETTING("detect_hands", bool, detect_hands);
      if (detect_hands) {
        //hand_detector_->evaluateForest((int16_t*)depth_);
        bool found_hand = hand_detector_->findHandLabels((int16_t*)depth_, 
          pts_world_, HDLabelMethod::HDFloodfill, labels_);
        if (!found_hand) {
          memset(labels_, 0, sizeof(labels_[0]) * src_dim);
        } else {
          GET_SETTING("detect_pose", bool, detect_pose);
          if (detect_pose) {
            hand_net_->calcHandCoeffConvnet((int16_t*)depth_, labels_);
          }
        }
      } else {
        memset(labels_, 0, sizeof(labels_[0]) * src_dim);
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

      // Unlock the local data structure since we've finished copying data
      data_lock_.unlock();

      context_lock_.unlock();
      std::this_thread::yield();

    }  // end while (app::App::app_running)
    cout << "kinectUpdateThread shutting down..." << endl;
  }

  void KinectInterface::shutdownKinect() {
    cout << "kinectUpdateThread shutdown requested..." << endl;
    kinect_running_ = false;
    kinect_thread_.join();
  }

  void KinectInterface::logErrors(xn::EnumerationErrors& rErrors) {
    for (xn::EnumerationErrors::Iterator it = rErrors.Begin(); 
      it != rErrors.End(); ++it) {
        XnChar desc[512];
        xnProductionNodeDescriptionToString(&it.Description(), desc, 512);
        cout << desc << " failed: " << xnGetStatusString(it.Error()) << endl;
    } 
  }

  // ************************************************************
  // XN Callbacks
  // ************************************************************
  // Callback: New user was detected
  void __stdcall KinectInterface::newUser(xn::UserGenerator& generator, 
    XnUserID nId, void* pCookie) {
    cout << "New User " << nId << endl;
    g_kinect_->num_users_found_++;
    // ONLY TRACK ONE USER
    if (KinectInterface::g_kinect_->num_users_found_ <= 1) {
      if (KinectInterface::g_kinect_->loadCalibration() == false) {
        // New user found
        if (g_bNeedPose) {
          g_kinect_->ug_->GetPoseDetectionCap().StartPoseDetection(
            g_strPose,
            nId);
        } else {
          g_kinect_->ug_->GetSkeletonCap().RequestCalibration(nId, 
            TRUE);
        }
      } else {
        g_kinect_->num_users_tracked_++;
      }
    }
  }

  // Callback: An existing user was lost
  void __stdcall KinectInterface::lostUser(xn::UserGenerator& generator, 
    XnUserID nId, void* pCookie) {
      cout << "Lost User " << nId << endl;
      g_kinect_->ug_->GetSkeletonCap().Reset(nId);
      g_kinect_->num_users_found_--;
  }

  // Callback: Detected a pose
  void __stdcall KinectInterface::poseDetected(
    xn::PoseDetectionCapability& capability, const char* strPose, 
    unsigned int nId, void* pCookie) {
    cout << "Pose " << strPose << " detected for user " << nId << endl;
    g_kinect_->ug_->GetPoseDetectionCap().StopPoseDetection(nId);
    g_kinect_->ug_->GetSkeletonCap().RequestCalibration(nId, TRUE);
  }

  // Callback: Started calibration
  void __stdcall KinectInterface::calStart(xn::SkeletonCapability& capability, 
    XnUserID nId, void* pCookie) {
    cout << "Calibration started for user " << nId << endl;
  }

  // Callback: Finished calibration
  void __stdcall KinectInterface::calFinish(xn::SkeletonCapability& capability,
    XnUserID nId, XnCalibrationStatus eStatus, void* pCookie) {
    if (eStatus == XN_CALIBRATION_STATUS_OK) {
      // Calibration succeeded
      cout << "Calibration complete, start tracking user " << nId << endl; 
      g_kinect_->ug_->GetSkeletonCap().StartTracking(nId);
      g_kinect_->num_users_tracked_++;
    } else {
      // Calibration failed
      cout << "Calibration failed for user " << nId << endl; 
      if (g_bNeedPose) {
        g_kinect_->ug_->GetPoseDetectionCap().StartPoseDetection(g_strPose,
          nId);
      } else {
        g_kinect_->ug_->GetSkeletonCap().RequestCalibration(nId, TRUE);
      }
    }
  }

  // ************************************************************
  // Calibration data load / save
  // ************************************************************

  // Save calibration to file
  void KinectInterface::saveCalibration() {
    XnUserID aUserIDs[20] = {0};
    XnUInt16 nUsers = 20;
    ug_->GetUsers(aUserIDs, nUsers);
    if (nUsers == 0) {
      cout << "No user's detected yet --> Move around!" << endl; 
      cout << "KinectInterface will save calibration when done." << endl; 
    } else {
      for (int i = 0; i < nUsers; ++i) {
        // Find a user who is already calibrated
        if (ug_->GetSkeletonCap().IsCalibrated(aUserIDs[i])) {
          // Save user's calibration to file
          XnStatus rc = 
            ug_->GetSkeletonCap().SaveCalibrationDataToFile(
            aUserIDs[i], 
            CALIBRATION_FILE);
          SHOW_RC(rc, "SaveCalibrationDataToFile() ");
          break;
        } else {
          cout << "User " << i << "is not yet calibrated, no data to save!" << endl; 
          cout << "KinectInterface will save calibration when done" << endl; 
        }
      }
    }
  }

  // Load calibration from file
  bool KinectInterface::loadCalibration() {
    XnUserID aUserIDs[20] = {0};
    XnUInt16 nUsers = 20;
    ug_->GetUsers(aUserIDs, nUsers);
    if (nUsers == 0) {
      cout << "No user's detected yet --> Move around!" << endl; 
      return false;
    } else {
      for (int i = 0; i < nUsers; ++i) {
        // Find a user who isn't calibrated or currently in pose
        if (ug_->GetSkeletonCap().IsCalibrated(aUserIDs[i])) {
          continue;
        }
        if (ug_->GetSkeletonCap().IsCalibrating(aUserIDs[i])) {
          continue;
        }

        // Load user's calibration from file
        XnStatus rc = 
          ug_->GetSkeletonCap().LoadCalibrationDataFromFile(aUserIDs[i], 
          CALIBRATION_FILE);
        SHOW_RC(rc, "LoadCalibrationDataFromFile() ");
        if (rc == XN_STATUS_OK) {
          // Make sure state is coherent
          ug_->GetPoseDetectionCap().StopPoseDetection(aUserIDs[i]);
          ug_->GetSkeletonCap().StartTracking(aUserIDs[i]);
          return true;
        } else {
          return false;
        }
        break;
      }
    }
    return false;
  }


  void KinectInterface::resetTracking() {
    GET_SETTING("track_openni_skeleton", bool, tracking_skeleton_);

    context_lock_.lock();
    data_lock_.lock();

    // This is a bit of a hack.  We need to completely rebuild the user gen.
    // since OpenNI doesn't have a Reset switch :-(
    std::cout << "Resetting tracking.  Please wait... " << endl;

    // Turn off the generator
    num_users_tracked_ = 0;
    num_users_found_ = 0;
    std::fill(joints_found_, joints_found_ + SKEL_NJOINTS, false);
    std::cout << "   --> Releasing ug_" << endl;
    ug_->Release();

    // Toggle tracking and turn it back on;
    initUserGenerator();
    std::cout << "Tracking Reset" << endl;

    for (uint32_t i = 0; i < SKEL_NJOINTS; i++) {
      joints_found_[i] = false;
    }

    hand_detector_->reset();

    data_lock_.unlock();
    context_lock_.unlock();
  }

  const uint8_t* KinectInterface::filteredDecisionForestLabels() const {
    return hand_detector_->labels_filtered();
  }

  const uint8_t* KinectInterface::rawDecisionForestLabels() const {
    return hand_detector_->labels_evaluated();
  }


  const float* KinectInterface::coeff_convnet() const {
    return hand_net_->coeff_convnet();
  }

  const jtil::math::Float3& KinectInterface::uvd_com() const {
    return hand_net_->uvd_com();
  }

}  // namespace kinect


