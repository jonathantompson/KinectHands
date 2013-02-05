//
//  app.cpp
//
//  Created by Jonathan Tompson on 5/15/12.
//
//  Main (singleton) application class for KinectHands
//
//  Some of this code was created by Otavio Braga (obraga@cs.nyu.edu) and 
//  modified by Jonathan Tompson (tompson@cs.nyu.edu)
// 

#include <future>
#include <thread>
#include <string>
#ifdef __APPLE__
#include <stdexcept>
#endif
#include "app/app.h"
#include "kinect_interface/kinect_interface.h"
#include "kinect_interface/hand_statistics.h"
#include "clock/clock.h"
#ifndef _WIN32
#include "shared_memory/shmem.h"  // Only for apple platforms
#else
#define HAND_FRAME_ELEMENTS 30
#define LEFTHAND_FRAME_BYTES HAND_FRAME_ELEMENTS * 4
#define RIGHTHAND_FRAME_BYTES HAND_FRAME_ELEMENTS * 4
#endif
#include "rendering/effects.h"
#include "math/math_types.h"
#include "zmq.hpp"
#include "hand_detector.h"

#ifdef _WIN32
  #ifndef snprintf
    #define snprintf _snprintf_s
  #endif
#endif

using std::string;
using std::runtime_error;
using kinect::KinectInterface;
using kinect::HandStatistics;
using std::thread;

namespace app {
  
  App* App::g_app = NULL;
  bool App::app_running = true;
  const std::string App::hand_data_filename_ = "Hand.bin";

#ifdef _WIN32
  typedef BOOL (APIENTRY *PFNWGLSWAPINTERVALFARPROC)( int );
  PFNWGLSWAPINTERVALFARPROC wglSwapIntervalEXT = 0;

  void setVSync(int interval=1)
  {
    const GLubyte* extensions = glGetString( GL_EXTENSIONS );

    if ( strstr( reinterpret_cast<char*>(const_cast<GLubyte*>(extensions)), "WGL_EXT_swap_control" ) == 0 )
      return; // Error: WGL_EXT_swap_control extension not supported on your computer.\n");
    else
    {
      wglSwapIntervalEXT = (PFNWGLSWAPINTERVALFARPROC)wglGetProcAddress( "wglSwapIntervalEXT" );

      if( wglSwapIntervalEXT )
        wglSwapIntervalEXT(interval);
    }
  }
#endif
  
  App::App() {
    window_width_ = static_cast<int>(floorf(1.5f*static_cast<float>(image_width_)));
    window_height_ = static_cast<int>(floorf(1.5f*static_cast<float>(image_height_)));
    window_width_ = 960;
    window_height_ = 720;
//    window_width_ = image_width_;
//    window_height_ = image_height_;    
    
    kinect_ = NULL;
    clk_ = new Clock();
    t1_ = clk_->getTime();
    blur_effect_ = new Effects<int>(image_width_, image_height_);
    
    cur_display_image_ = TEXTURE_ID_RGB_IMAGE;
    render_data_ = true;
    first_frame_ = true;
    render_hand_points_ = true;
    render_finger_points_ = true;
    render_bounding_box_points_ = true;
    render_skeleton_ = true;
    render_hud_ = true;
    render_hand_images_ = false;
    frame_num_ = 0;
    setKinectStatus(false);
    static std::string str("Looking for Kinect...");
    setKinectStatusStr(str);
    snprintf(fps_string_, sizeof(fps_string_), "");
    calculate_hand_statistics_ = true;
    tracking_skeleton_ = false;
    snapshot_timer_ = 0;
    shutdown_requested_ = false;
    continuous_dt_depth_snapshot_ = false;
    continuous_hp_depth_snapshot_ = false;
    continuous_snapshot_ = false;
    hands_mask = NULL;
      
    // HAND FIT TRAINING DATA SETTINGS
    tracking_skeleton_ = true;
    calculate_hand_statistics_ = false;
  }
  
  App::~App() {
    app_running = false;
    
    if (kinect_) {
      // Wait for all kinect threads to finish
      if (kinect_->kinect_running()) {
        printf("Waiting for Kinect thread to shut down\n");
        kinect_->requestShutdown();
        kinect_->join();
      }
      delete kinect_;
      kinect_ = NULL;
    }
    if (clk_) {
      delete clk_;
      clk_ = NULL;
    }
    if (blur_effect_) {
      delete blur_effect_;
      blur_effect_ = NULL;
    }
    
    if (hands_mask) {
      delete hands_mask;
      hands_mask = NULL;
    }
    
  }
  
  void App::PrintUsage() {
    cout << "KEYBOARD USAGE:" << endl;
    cout << " - ESC / q: Quit" << endl;
    cout << " - t: Save 1 frame of kinect data" << endl;
    cout << " - r: Save a continuous snapshot of kinect RGB + depth data" << endl;
    cout << " - y: Save a continuous snapshot of kinect RGB + depth data (for HandForest/Fit project)" << endl;
    cout << " - u: Save a continuous snapshot of kinect RGB + depth data (with the processed hand points)" << endl;
  }
  
  void App::setKinectStatus(bool status) {
    kinect_status_lock_.lock();
    kinect_status_ = status;
    kinect_status_lock_.unlock();
  }

  double App::getTime() {
    return g_app->clk_->getTime();
  }

  void App::setKinectStatusStr(std::string& str) {
    kinect_status_lock_.lock();
    kinect_status_str_ = str;
    kinect_status_lock_.unlock();
  }
  
  std::string App::getKinectStatusStr() {
    kinect_status_lock_.lock();
    std::string ret = kinect_status_str_;
    kinect_status_lock_.unlock();
    return ret;
  }
  
  bool App::getKinectStatus() {
    kinect_status_lock_.lock();
    bool ret = kinect_status_;
    kinect_status_lock_.unlock();
    return ret;
  }
  
  void App::InitApp(int argc, char *argv[]) {
    g_app = new App();
    g_app->initDownstreamPipe();
    
    //#ifdef __APPLE_CC__
    //    ServerStart();
    //#endif
    
    glutInit(&argc, argv);
    glutInitWindowSize(g_app->window_width_, g_app->window_height_);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    g_app->main_window_ = glutCreateWindow("KinectHands");
    glutDisplayFunc(App::display);
    glutReshapeFunc(App::reshape);
    glutMouseFunc(App::mouse);
    glutMotionFunc(App::click);
    glutKeyboardFunc(App::keyboard);
    glutIdleFunc(App::idle);
    
    g_app->initTextures();
    
    glutCreateMenu(App::menuSelected);
    glutAddMenuEntry("-----------------------------------------", 
                     MENU_ID_BLANK);
    glutAddMenuEntry("Render: ON/OFF", MENU_ID_RENDER_DATA_ON_OFF);
    glutAddMenuEntry("Render: RGB", MENU_ID_RGB_IMAGE);
    glutAddMenuEntry("Render: RGB Blurred", MENU_ID_RGB_BLUR);
    glutAddMenuEntry("Render: Depth", MENU_ID_DEPTH_IMAGE);
    glutAddMenuEntry("Render: RGB and Depth", MENU_ID_RGB_AND_DEPTH_IMAGE);
    glutAddMenuEntry("Render: Hand Labels (from Hand Detector)", 
                     MENU_ID_HAND_LABELS_IMAGE);
    glutAddMenuEntry("Render: Skeleton", MENU_ID_RENDER_SKELETON);
    glutAddMenuEntry("Render: Hand Point Cloud", MENU_ID_RENDER_HAND_POINTS);
    glutAddMenuEntry("Render: Hand Bounding Box", 
                     MENU_ID_RENDER_BOUNDING_BOX_POINTS);
    glutAddMenuEntry("Render: Finger Points", MENU_ID_RENDER_FINGER_POINTS);
    glutAddMenuEntry("Render: HUD", MENU_ID_RENDER_HUD);
    glutAddMenuEntry("Render: Hand Constellations", MENU_ID_RENDER_HAND_IMAGES);
    glutAddMenuEntry("-----------------------------------------", 
                     MENU_ID_BLANK);
    glutAddMenuEntry("Store calibration data", MENU_ID_STORE_CALIBRATION_DATA);
    glutAddMenuEntry("-----------------------------------------", 
                     MENU_ID_BLANK);
    glutAddMenuEntry("Track skeleton ON / OFF", MENU_ID_TRACK_SKELETON_ON_OFF);
    glutAddMenuEntry("Reset tracking", MENU_ID_RESET_TRACKING);
    glutAddMenuEntry("Reset Kinect", MENU_ID_RESET_KINECT);
    glutAddMenuEntry("Calculate hand statistics ON/OFF", 
                     MENU_ID_CALCULATE_HAND_STATISTICS);
    glutAddMenuEntry("-----------------------------------------", 
                     MENU_ID_BLANK);
    /*
    glutAddMenuEntry("+ HandStatistics Blur Radius", 
                     MENU_ID_HAND_STATISTICS_INCREASE_BLUR_RAD);
    glutAddMenuEntry("- HandStatistics Blur Radius", 
                     MENU_ID_HAND_STATISTICS_DECREASE_BLUR_RAD);    
    glutAddMenuEntry("+ HandStatistics finger threshold", 
                     MENU_ID_HAND_STATISTICS_INCREASE_FINGER_THRESHOLD);
    glutAddMenuEntry("- HandStatistics finger threshold", 
                     MENU_ID_HAND_STATISTICS_DECREASE_FINGER_THRESHOLD);     
    glutAddMenuEntry("+ HandStatistics finger inner radius", 
                     MENU_ID_HAND_STATISTICS_INCREASE_FINGER_INNER_RAD);
    glutAddMenuEntry("- HandStatistics finger inner radius", 
                     MENU_ID_HAND_STATISTICS_DECREASE_FINGER_INNER_RAD);      
    glutAddMenuEntry("+ HandStatistics finger outer radius", 
                     MENU_ID_HAND_STATISTICS_INCREASE_FINGER_OUTER_RAD);
    glutAddMenuEntry("- HandStatistics finger outer radius", 
                     MENU_ID_HAND_STATISTICS_DECREASE_FINGER_OUTER_RAD);      
    glutAddMenuEntry("-----------------------------------------", 
                     MENU_ID_BLANK);
     */
    glutAddMenuEntry("+ HandDetector stage 1 shrink filter radius", 
                     MENU_ID_HAND_DETECTOR_INCREASE_STAGE1_SHRINK_FILT_RAD);
    glutAddMenuEntry("- HandDetector stage 1 shrink filter radius", 
                     MENU_ID_HAND_DETECTOR_DECREASE_STAGE1_SHRINK_FILT_RAD);     
    glutAddMenuEntry("+ HandDetector stage 2 median filter radius", 
                     MENU_ID_HAND_DETECTOR_INCREASE_STAGE2_MED_FILT_RAD);
    glutAddMenuEntry("- HandDetector stage 2 median filter radius", 
                     MENU_ID_HAND_DETECTOR_DECREASE_STAGE2_MED_FILT_RAD);  
    glutAddMenuEntry("+ HandDetector number of decision trees", 
                     MENU_ID_HAND_DETECTOR_INCREASE_NUM_DECISION_TREES);
    glutAddMenuEntry("- HandDetector number of decision trees", 
                     MENU_ID_HAND_DETECTOR_DECREASE_NUM_DECISION_TREES);  
    glutAddMenuEntry("+ HandDetector max height to evaluate", 
                     MENU_ID_HAND_DETECTOR_INCREASE_MAX_HEIGHT_TO_EVALUATE);
    glutAddMenuEntry("- HandDetector max height to evaluate", 
                     MENU_ID_HAND_DETECTOR_DECREASE_MAX_HEIGHT_TO_EVALUATE);  
    
    glutAddMenuEntry("-----------------------------------------", 
                     MENU_ID_BLANK);    

    glutAttachMenu(GLUT_LEFT_BUTTON);
    
    g_app->initGLState();

#ifdef _WIN32
    setVSync(0);
#endif
    
    // Initialize the kinect in a new thread (asynchronously)
    std::thread t1 = std::thread(InitKinect);
    t1.detach();
  }
  
  void App::RunApp() {
    glutMainLoop();
  }
  
  void App::KillApp() {
    delete g_app;
    exit(0);  // Quit openGL loop
  } 
  
  void App::idle() {
    if (g_app->shutdown_requested_) {
      KillApp();
    } else {
      if (g_app->app_running) {
        bool new_data = false;
        if (g_app->getKinectStatus()) {
          new_data = g_app->getKinectData();
        } 
        
        if (new_data) {
          glutSetWindow(g_app->main_window_);
          glutPostRedisplay();
        }
        
        std::this_thread::yield();
        // It's actually safe to do some sleeping here since we know that the
        // kinect cannot update more than 30fps and the server + display code
        // is in lock step with it.
        // std::this_thread::sleep_for(std::chrono::milliseconds(4));  // "Lowish", to reduce latency
        
        // TO DO: We need a barrier here (with the Kinect update thread)
        // Need to implement a C++11 barrier.
      }
    }
  }
  
  // This will be slow --> Only do it on debug
  void convert_rgb_to_3CInt(int* retR, int* retG, int* retB, XnRGB24Pixel* src, 
                            int width, int height) {
    for (int V = 0; V < height; V++) {
      for (int U = 0; U < width; U ++) {
        int ind = V*width + U;
        retR[ind] = static_cast<float>(src[ind].nRed);
        retG[ind] = static_cast<float>(src[ind].nGreen);
        retB[ind] = static_cast<float>(src[ind].nBlue);
      }
    }
  }
  void convert_3CInt_to_rgb(XnUInt8* ret, int* srcR, int* srcG, int* srcB, 
                            int width, int height) {
    for (int V = 0; V < height; V++) {
      for (int U = 0; U < width; U ++) {
        int ind = V*width + U;
        ret[ind*3] = (XnUInt8)srcR[ind];    // r
        ret[ind*3+1] = (XnUInt8)srcG[ind];  // g
        ret[ind*3+2] = (XnUInt8)srcB[ind];  // b
      }
    }
  }
  void convert_1CInt_to_rgb(XnUInt8* ret, int* src, int width, int height) {
    for (int V = 0; V < height; V++) {
      for (int U = 0; U < width; U ++) {
        int ind = V*width + U;
        ret[ind*3] = (XnUInt8)src[ind];    // r
        ret[ind*3+1] = (XnUInt8)src[ind];  // g
        ret[ind*3+2] = (XnUInt8)src[ind];  // b
      }
    }
  }
  void scale_1CInt(int* ret, int maxRetVal, int width, int height) {
    int maxVal = ret[0];
    for (int i = 2; i < width * height; i ++) {
      if (ret[i] > maxVal)
        maxVal = ret[i];
    }
    if (maxVal == 0)
      return;
    for (int V = 0; V < height; V++) {
      for (int U = 0; U < width; U ++) {
        int ind = V*width + U;
        ret[ind] = (ret[ind]*maxRetVal) / maxVal;
      }
    }
  }
  
  bool App::getKinectData() {
    // This is a volitile lookup of the kinect frame number.  We're not
    // protected here by the lock in any way.  But the reality is that we'll
    // only fall back by one frame, so it's not the end of the world.
    if (kinect_->getFrameNumber() == zmq_frame_number) {
      return false;
    }
    
    kinect_->lockData();
    
    zmq_frame_number = kinect_->getFrameNumber();
    updateServer();

    // Save the image texture
    XnRGB24Pixel* rgb_image;
    XnUInt8* depth_image_;
    uint8_t* label_image_;
    int16_t* depth_image_downsampled_;
    HandDetector* hd;    
    uint32_t w_downsampled;
    uint32_t h_downsampled;
    static const float a = 0.75f;
    static const float a_m_1 = 1.0f - a;
    switch (cur_display_image_) {
      case TEXTURE_ID_RGB_IMAGE:
        rgb_image = kinect_->getRGBImageUnsafe();
        glBindTexture(GL_TEXTURE_2D, texture_[TEXTURE_ID_RGB_IMAGE]);
        glTexImage2D(GL_TEXTURE_2D,  0, GL_RGB, 640, 480, 0, GL_RGB, 
                     GL_UNSIGNED_BYTE, rgb_image);

        break;
      case TEXTURE_ID_RGB_BLUR_IMAGE:
        // THIS IS SLOW -> REALLY JUST FOR DEBUG!
        rgb_image = kinect_->getRGBImageUnsafe();
        convert_rgb_to_3CInt(tmpImageR, tmpImageG, tmpImageB, 
                             rgb_image, 
                             image_width_, image_height_);
        blur_effect_->blurX(tmpImageR, 10, image_width_, image_height_);
        blur_effect_->blurX(tmpImageG, 10, image_width_, image_height_);
        blur_effect_->blurX(tmpImageB, 10, image_width_, image_height_);
        blur_effect_->blurY(tmpImageR, 10, image_width_, image_height_);
        blur_effect_->blurY(tmpImageG, 10, image_width_, image_height_);
        blur_effect_->blurY(tmpImageB, 10, image_width_, image_height_);
        convert_3CInt_to_rgb(tmpImageRGB, tmpImageR, tmpImageG, tmpImageB, 
                             image_width_, image_height_);
        glBindTexture(GL_TEXTURE_2D, texture_[TEXTURE_ID_RGB_BLUR_IMAGE]);
        glTexImage2D(GL_TEXTURE_2D,  0, GL_RGB, 640, 480, 0, GL_RGB, 
                     GL_UNSIGNED_BYTE, tmpImageRGB);
        break;
      case TEXTURE_ID_DEPTH_IMAGE:
        kinect_->setDepthColoring(kinect::COLORING_GREY);
        depth_image_ = kinect_->getDepthImageUnsafe();
        glBindTexture(GL_TEXTURE_2D, texture_[TEXTURE_ID_DEPTH_IMAGE]);
        glTexImage2D(GL_TEXTURE_2D,  0, GL_RGB, 640, 480, 0, GL_RGBA, 
                     GL_UNSIGNED_BYTE, depth_image_);
        break;
      case TEXTURE_ID_DEPTH_AND_RGB_IMAGE:
        kinect_->setDepthColoring(kinect::COLORING_INVERSE_RAW_DEPTH);
        rgb_image = kinect_->getRGBImageUnsafe();
        depth_image_ = kinect_->getDepthImageUnsafe();
        for (uint32_t i = 0; i < 640*480; i ++) {
          if (depth_image_[i*4] != 0) {
            tmpImageRGB[i*3] = static_cast<XnUInt8>(floor(
                               a_m_1*static_cast<float>(rgb_image[i].nRed)+
                               a*static_cast<float>(depth_image_[i*4])));
            tmpImageRGB[i*3+1] = static_cast<XnUInt8>(floor(
                               a_m_1*static_cast<float>(rgb_image[i].nGreen)+
                               a*static_cast<float>(depth_image_[i*4])));      
            tmpImageRGB[i*3+2] = static_cast<XnUInt8>(floor(
                               a_m_1*static_cast<float>(rgb_image[i].nBlue)+
                               a*static_cast<float>(depth_image_[i*4]))); 
          } else {
            tmpImageRGB[i*3] = rgb_image[i].nRed;
            tmpImageRGB[i*3+1] = rgb_image[i].nGreen;
            tmpImageRGB[i*3+2] = rgb_image[i].nBlue;
          }
        }
        glBindTexture(GL_TEXTURE_2D, texture_[TEXTURE_ID_DEPTH_AND_RGB_IMAGE]);
        glTexImage2D(GL_TEXTURE_2D,  0, GL_RGB, 640, 480, 0, GL_RGB, 
                     GL_UNSIGNED_BYTE, tmpImageRGB);
      case TEXTURE_ID_LHAND_FINGERS:
        break;
      case TEXTURE_ID_RHAND_FINGERS:
        break;
      case TEXTURE_ID_HAND_LABELS:
        hd = kinect_->getHandDetector();
        depth_image_downsampled_ = hd->getDepthImDownsampled();
        label_image_ = hd->getLabelIm();
        w_downsampled = hd->width();
        h_downsampled = hd->height();
        for (uint32_t i = 0; i < w_downsampled*h_downsampled; i ++) {
          tmpImageRGB[i*3] = depth_image_downsampled_[i] >> 3;
          tmpImageRGB[i*3+1] = depth_image_downsampled_[i] >> 3;
          tmpImageRGB[i*3+2] = depth_image_downsampled_[i] >> 3;
          if (label_image_[i] == 1) {
            tmpImageRGB[i*3+1] = 0;
            tmpImageRGB[i*3+2] = 0;
          }
        }
        glBindTexture(GL_TEXTURE_2D, texture_[TEXTURE_ID_HAND_LABELS]);
        glTexImage2D(GL_TEXTURE_2D,  0, GL_RGB, w_downsampled, h_downsampled, 0, 
                     GL_RGB, GL_UNSIGNED_BYTE, tmpImageRGB);
        break;
      case TEXTURE_ID_NUM_TEXTURES:
        break;
    }
    kinect_->unlockData();

    if (render_hand_images_) {
      kinect_->getLHand()->calHandImage(lHandImageRGB, hand_image_dim_);
      glBindTexture(GL_TEXTURE_2D, texture_[TEXTURE_ID_LHAND_FINGERS]);
      glTexImage2D(GL_TEXTURE_2D,  0, GL_RGB, hand_image_dim_, hand_image_dim_, 0, GL_RGB, 
        GL_UNSIGNED_BYTE, lHandImageRGB);
      kinect_->getRHand()->calHandImage(rHandImageRGB, hand_image_dim_);
      glBindTexture(GL_TEXTURE_2D, texture_[TEXTURE_ID_RHAND_FINGERS]);
      glTexImage2D(GL_TEXTURE_2D,  0, GL_RGB, hand_image_dim_, hand_image_dim_, 0, GL_RGB, 
        GL_UNSIGNED_BYTE, rHandImageRGB);
    }
    return true;
  }
  
  void App::menuSelected(int id) {
    static string reset_txt("Resetting user tracking, please wait");
    static string reset_kinect_txt("Resetting kinect, please wait");
    int rad;
    float inner_rad;
    float outer_rad;
    uint16_t threshold;
    switch (id) {
      case MENU_ID_RGB_IMAGE:
        g_app->cur_display_image_ = TEXTURE_ID_RGB_IMAGE;
        break;
      case MENU_ID_DEPTH_IMAGE:
        g_app->cur_display_image_ = TEXTURE_ID_DEPTH_IMAGE;
        break;
      case MENU_ID_RGB_AND_DEPTH_IMAGE:
        g_app->cur_display_image_ = TEXTURE_ID_DEPTH_AND_RGB_IMAGE;
        break;
      case MENU_ID_HAND_LABELS_IMAGE:
        g_app->cur_display_image_ = TEXTURE_ID_HAND_LABELS;
        break;
      case MENU_ID_STORE_CALIBRATION_DATA:
        g_app->kinect_->SaveCalibration();
        break;
      case MENU_ID_LOAD_CALIBRATION_DATA:
        g_app->kinect_->LoadCalibration();
        break;
      case MENU_ID_RENDER_DATA_ON_OFF:
        g_app->render_data_ = !g_app->render_data_;
        g_app->first_frame_ = true;
        break;
      case MENU_ID_TRACK_SKELETON_ON_OFF:
        g_app->tracking_skeleton_ = !g_app->tracking_skeleton_;
        g_app->kinect_->ResetTracking(g_app->tracking_skeleton_);
        break;
      case MENU_ID_CALCULATE_HAND_STATISTICS:
        g_app->calculate_hand_statistics_ = !g_app->calculate_hand_statistics_;
        g_app->kinect_->setCalculateHandStatistics(g_app->calculate_hand_statistics_);
        g_app->kinect_->setCalculateHandPoints(g_app->calculate_hand_statistics_);
        break;
      case MENU_ID_RESET_TRACKING:
        g_app->renderBlackScreenWithText(reset_txt);
        g_app->kinect_->ResetTracking(g_app->tracking_skeleton_);
        break;
      case MENU_ID_RESET_KINECT:
        g_app->renderBlackScreenWithText(reset_kinect_txt);
        if (g_app->kinect_) {
          // Wait for all kinect threads to finish
          if (g_app->kinect_->kinect_running()) {
            g_app->kinect_->requestShutdown();
            g_app->kinect_->join();
          }
          delete g_app->kinect_;
          g_app->kinect_ = NULL;
        }
        g_app->InitKinect();
        break;
      case MENU_ID_RGB_BLUR:
        g_app->cur_display_image_ = TEXTURE_ID_RGB_BLUR_IMAGE;
        break;
      case MENU_ID_RENDER_SKELETON:
        g_app->render_skeleton_ = !g_app->render_skeleton_;
        break;      
      case MENU_ID_RENDER_HAND_POINTS:
        g_app->render_hand_points_ = !g_app->render_hand_points_;
        break;
      case MENU_ID_RENDER_FINGER_POINTS:
        g_app->render_finger_points_ = !g_app->render_finger_points_;
        break;
      case MENU_ID_RENDER_BOUNDING_BOX_POINTS:
        g_app->render_bounding_box_points_ = 
        !g_app->render_bounding_box_points_;
        break;
      case MENU_ID_RENDER_HUD:
        g_app->render_hud_ = !g_app->render_hud_;
        break;
      case MENU_ID_RENDER_HAND_IMAGES:
        g_app->render_hand_images_ = !g_app->render_hand_images_;
        break;
      case MENU_ID_HAND_STATISTICS_INCREASE_BLUR_RAD:
        rad = kinect::HandStatistics::getFingerBlurRad();
        rad++;
        kinect::HandStatistics::setFingerBlurRad(rad);
        printf("Setting HandStatistics blur rad = %d\n", rad);
        break;
      case MENU_ID_HAND_STATISTICS_DECREASE_BLUR_RAD:
        rad = kinect::HandStatistics::getFingerBlurRad();
        rad = (rad > 0) ? rad - 1 : rad;
        kinect::HandStatistics::setFingerBlurRad(rad);
        printf("Setting HandStatistics blur rad = %d\n", rad);
        break;      
      case MENU_ID_HAND_STATISTICS_INCREASE_FINGER_THRESHOLD:
        threshold = kinect::HandStatistics::getFingerThreshold();
        threshold += 200;
        kinect::HandStatistics::setFingerThreshold(threshold);
        printf("Setting HandStatistics finger threshold = %d\n", threshold);
        break;
      case MENU_ID_HAND_STATISTICS_DECREASE_FINGER_THRESHOLD:
        threshold = kinect::HandStatistics::getFingerThreshold();
        threshold = (threshold >= 200) ? threshold - 200 : threshold;
        kinect::HandStatistics::setFingerThreshold(threshold);
        printf("Setting HandStatistics finger threshold = %d\n", threshold);
        break;
      case MENU_ID_HAND_STATISTICS_INCREASE_FINGER_INNER_RAD:
        inner_rad = kinect::HandStatistics::getFingerInnerRad();
        outer_rad = kinect::HandStatistics::getFingerOuterRad();
        inner_rad = ((inner_rad + 10) <= outer_rad) ? inner_rad + 10 : inner_rad;
        kinect::HandStatistics::setFingerInnerRad(inner_rad);
        printf("Setting HandStatistics inner rad = %.1f\n", inner_rad);
        break;
      case MENU_ID_HAND_STATISTICS_DECREASE_FINGER_INNER_RAD:
        inner_rad = kinect::HandStatistics::getFingerInnerRad();
        inner_rad = (inner_rad >= 10) ? inner_rad - 10 : inner_rad;
        kinect::HandStatistics::setFingerInnerRad(inner_rad);
        printf("Setting HandStatistics inner rad = %.1f\n", inner_rad);
        break;    
      case MENU_ID_HAND_STATISTICS_INCREASE_FINGER_OUTER_RAD:
        outer_rad = kinect::HandStatistics::getFingerOuterRad();
        outer_rad = outer_rad + 10;
        kinect::HandStatistics::setFingerOuterRad(outer_rad);
        printf("Setting HandStatistics outer rad = %.1f\n", outer_rad);
        break;  
      case MENU_ID_HAND_STATISTICS_DECREASE_FINGER_OUTER_RAD:
        inner_rad = kinect::HandStatistics::getFingerInnerRad();
        outer_rad = kinect::HandStatistics::getFingerOuterRad();
        outer_rad = ((outer_rad - 10) >= inner_rad) ? outer_rad - 10 : outer_rad;
        kinect::HandStatistics::setFingerOuterRad(outer_rad);
        printf("Setting HandStatistics outer rad = %.1f\n", outer_rad);
        break;
      case MENU_ID_HAND_DETECTOR_INCREASE_STAGE1_SHRINK_FILT_RAD:
        rad = g_app->kinect_->getHandDetector()->stage1_shrink_filter_radius();
        rad++;
        g_app->kinect_->getHandDetector()->stage1_shrink_filter_radius(rad);
        printf("Setting HandDetector stage 1 shrink filter rad = %d\n", rad);
        break;
      case MENU_ID_HAND_DETECTOR_DECREASE_STAGE1_SHRINK_FILT_RAD:
        rad = g_app->kinect_->getHandDetector()->stage1_shrink_filter_radius();
        if (rad > 0) {
          rad--;
          g_app->kinect_->getHandDetector()->stage1_shrink_filter_radius(rad);
        }
        printf("Setting HandDetector stage 1 shrink filter rad = %d\n", rad);
        break;
      case MENU_ID_HAND_DETECTOR_INCREASE_STAGE2_MED_FILT_RAD:
        rad = g_app->kinect_->getHandDetector()->stage2_med_filter_radius();
        rad++;
        g_app->kinect_->getHandDetector()->stage2_med_filter_radius(rad);
        printf("Setting HandDetector stage 2 median filter rad = %d\n", rad);
        break;
      case MENU_ID_HAND_DETECTOR_DECREASE_STAGE2_MED_FILT_RAD:
        rad = g_app->kinect_->getHandDetector()->stage2_med_filter_radius();
        if (rad > 0) {
          rad--;
          g_app->kinect_->getHandDetector()->stage2_med_filter_radius(rad);
        }
        printf("Setting HandDetector stage 2 median filter rad = %d\n", rad);
        break;
      case MENU_ID_HAND_DETECTOR_INCREASE_NUM_DECISION_TREES:
        rad = g_app->kinect_->getHandDetector()->num_trees_to_evaluate();
        rad++;
        if (rad < static_cast<int>(g_app->kinect_->getHandDetector()->num_trees())) {
          g_app->kinect_->getHandDetector()->num_trees_to_evaluate(rad);
        }
        printf("Setting HandDetector number of trees = %d\n", rad);
        break;
      case MENU_ID_HAND_DETECTOR_DECREASE_NUM_DECISION_TREES:
        rad = g_app->kinect_->getHandDetector()->num_trees_to_evaluate();
        if (rad > 2) {
          rad--;
          g_app->kinect_->getHandDetector()->num_trees_to_evaluate(rad);
        }
        printf("Setting HandDetector number of trees = %d\n", rad);
        break;
      case MENU_ID_HAND_DETECTOR_INCREASE_MAX_HEIGHT_TO_EVALUATE:
        rad = g_app->kinect_->getHandDetector()->max_height_to_evaluate();
        rad++;
        if (rad < static_cast<int>(g_app->kinect_->getHandDetector()->max_height())) {
          g_app->kinect_->getHandDetector()->max_height_to_evaluate(rad);
        }
        printf("Setting HandDetector max height to evaluate = %d\n", rad);
        break;
      case MENU_ID_HAND_DETECTOR_DECREASE_MAX_HEIGHT_TO_EVALUATE:
        rad = g_app->kinect_->getHandDetector()->max_height_to_evaluate();
        if (rad > 2) {
          rad--;
          g_app->kinect_->getHandDetector()->max_height_to_evaluate(rad);
        }
        printf("Setting HandDetector max height to evaluate = %d\n", rad);
        break;
      default:
        throw std::runtime_error(string("ERROR: App::menuSelected(int id) - ") +
          string("unhandeled id number"));
    }
  }
  
  void App::keyboard(unsigned char key, int x, int y) {
    switch (key) {
      case 27:  // ESC
      case 'q':
      case 'Q':
        RequestShutdown();
        break;
      case 't':  // Save kinect hand data
        printf("Saving data in 2 secs\n");
        g_app->snapshot_timer_ = 2;
        break;
      case 'r':  // Save kinect hand data
        if (g_app->continuous_snapshot_) {
          printf("End continuous snapshots.\n");          
        } else {
          printf("Begin continuous snapshots.\n");          
        }
        g_app->continuous_snapshot_ = !g_app->continuous_snapshot_ &&
          !g_app->continuous_dt_depth_snapshot_ &&
          !g_app->continuous_hp_depth_snapshot_;
        break;
      case 'y':  // Save kinect depth data (for decision tree clasifier)
        if (g_app->continuous_dt_depth_snapshot_) {
          printf("End continuous decision tree depth snapshots.\n");          
        } else {
          printf("Begin continuous decision tree depth snapshots.\n");          
        }
        g_app->continuous_dt_depth_snapshot_ = !g_app->continuous_dt_depth_snapshot_  &&
          !g_app->continuous_snapshot_ &&
          !g_app->continuous_hp_depth_snapshot_;
        break;
      case 'u':  // Save kinect depth data (for hand fitting)
        if (g_app->continuous_hp_depth_snapshot_) {
          printf("End continuous hand fit depth snapshots.\n");
        } else {
          printf("Begin continuous hand fit depth snapshots.\n");
        }
        g_app->continuous_hp_depth_snapshot_ = !g_app->continuous_hp_depth_snapshot_  &&
          !g_app->continuous_snapshot_ &&
          !g_app->continuous_dt_depth_snapshot_;
        break;
    }
  }
  
  void App::RequestShutdown() {
    g_app->shutdown_requested_ = true;
  }
  
  void App::click(int x, int y) {
    // Empty
  }
  
  void App::reshape(int width, int height) {
    g_app->window_width_ = width;
    g_app->window_height_ = height;
    
    glutReshapeWindow(width, height);
    glutPostRedisplay();
    
    glutSetWindow(g_app->main_window_);
    g_app->first_frame_ = true;
  }
  
  void App::mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
      // nothing to do
    } else if (button == GLUT_RIGHT_BUTTON) {
      // nothing to do
    }
  }
  
  void App::display() {
    g_app->render();
  }
  
  void App::render() {

    if (getKinectStatus()) {
      t0_ = t1_;
      t1_ = clk_->getTime();
      fps_accumulate_ += t1_ - t0_;
      
      // For the first frame after we turn off rendering, render 1 frame with a
      // "rendering off" message.
      if (!render_data_) {
        if (first_frame_) {
          first_frame_ = false;
          static string txt("Render Kinect Data = OFF");
          renderBlackScreenWithText(txt);
        }
        return;
      }
      
      frame_num_++;
      if (fps_accumulate_ >= 0.5f) {
        snprintf(fps_string_, sizeof(fps_string_), 
                 "FPS = %.2f, kinect FPS = %.2f", static_cast<float>(frame_num_) / 
                 fps_accumulate_, kinect_->fps());
        frame_num_ = 0;
        fps_accumulate_ = 0;
      }
      
      if (snapshot_timer_ > 0) {
        snapshot_timer_ -= t1_ - t0_;
      }
      if (snapshot_timer_ < 0) {
        std::stringstream ss;
        ss << "hands_" << clk_->getAbsoluteTimeNano() << ".bin";

        if (kinect::HandStatistics::saveHandData(hand_data_filename_,
                                                 true, true)) {
          printf("Saved Hand RGB and Depth data to filename: %s\n", ss.str().c_str());
        } else {
          printf("No Hand data to save!\n");
        }        
        snapshot_timer_ = 0;
      }
      
      if (continuous_snapshot_) {
        std::stringstream ss;
        ss << "hands_" << clk_->getAbsoluteTimeNano() << ".bin";
        //cout << ss.str() << endl;
        if (kinect::HandStatistics::saveHandData(ss.str(), true, true)) {
          printf("Saved RGB and Depth data to filename: %s\n", ss.str().c_str());
        } else {
          printf("No data to save!\n");
        }
      }
      if (continuous_dt_depth_snapshot_) {
        std::stringstream ss;
        ss << "hands_" << clk_->getAbsoluteTimeNano() << ".bin";
        //cout << ss.str() << endl;
        if (kinect::HandStatistics::saveHandDataForDecisionTree(ss.str())) {
          printf("Saved Depth data to filename: %s\n", ss.str().c_str());
        } else {
          printf("No data to save!\n");
        }
      }
      if (continuous_hp_depth_snapshot_) {
        std::stringstream ss;
        ss << "hands_" << clk_->getAbsoluteTimeNano() << ".bin";
        if (kinect::HandStatistics::saveHandDataWithHandPoints(ss.str())) {
          printf("Saved Depth data to filename: %s\n", ss.str().c_str());
        } else {
          printf("No data to save!\n");
        }
      }
      
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      
      glViewport(0, 0, window_width_, window_height_);
      
      drawKinectData();
      
      glutSwapBuffers();
      first_frame_ = false;
    } else {
      renderBlackScreenWithText(getKinectStatusStr().c_str());
    }
  }

  void App::drawQuad() {
    glColor3f(1, 1, 1);
    glDisable(GL_CULL_FACE);
    glBegin(GL_QUADS);
    glTexCoord2d(0.0, 1.0); 
    glVertex2d(0.0, 0.0);
    glTexCoord2d(1.0, 1.0); 
    glVertex2d(1.0, 0.0);
    glTexCoord2d(1.0, 0.0); 
    glVertex2d(1.0, 1.0);
    glTexCoord2d(0.0, 0.0); 
    glVertex2d(0.0, 1.0);
    glEnd();
  }
  
  void App::drawKinectData() {
    glEnable(GL_TEXTURE_2D);
    glPointSize(3.0f);  
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();  // Push to 1
    glLoadIdentity();
    gluOrtho2D(0, 1, 0, 1);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();  // Push to 2
    glLoadIdentity();
    
    // RENDER EVERYTHING MIRRORED
    if (RENDER_MIRRORED) {
      glPushMatrix();  // Push to 3 mirrored
      glLoadIdentity();
      glScalef(-1.0f, 1.0f, 1.0f);
      glTranslatef(-1.0f, 0.0f, 0.0f);
    }
    
    // Select the texture based on the image mode we are currently on
    glBindTexture(GL_TEXTURE_2D, texture_[cur_display_image_]);
    drawQuad();

    if (render_hand_images_) {
      static const float x_scale = 0.25f;
      float y_scale = x_scale*static_cast<float>(window_width_)/static_cast<float>(window_height_);  
      // hand images are 1:1
      glPushMatrix();  // Push to 3 mirrored
      glTranslatef(0.05f, 0.05f, 0);
      glScalef(x_scale, y_scale, 1.0f);
      glBindTexture(GL_TEXTURE_2D, texture_[TEXTURE_ID_LHAND_FINGERS]);
      drawQuad();
      glTranslatef(1.1f, 0, 0);
      glBindTexture(GL_TEXTURE_2D, texture_[TEXTURE_ID_RHAND_FINGERS]);
      drawQuad();
      glPopMatrix();
    }

    // Draw the projected joints
    glDisable(GL_TEXTURE_2D);
    glLineWidth(5);
    glPointSize(1);
    
    glPushMatrix();  // Push to 4 mirrored, push to 3 otherwise
    glScalef(1.0f/image_width_, 1.0f/image_height_, 1.0f);  // viewport tf
    glScalef(1.0f, -1.0f, 1.0f);
    glTranslatef(0, -image_height_, 0);
    
    glColor3f(1.0f, 0.0f, 0.0f);
    
    glColor3f(1.0f, 0.0f, 0.0f);
    if (render_skeleton_) {
      kinect_->drawSkeletonLines();
    }
    
    if (render_bounding_box_points_) {
      kinect_->drawHandDetectorOBB();
    }

    kinect_->drawHandStatistics(render_hand_points_, 
                                render_bounding_box_points_,
                                render_finger_points_);
    
    glPopMatrix();  // Pop to 3 mirrored, pop to 2 otherwise
    
    if (RENDER_MIRRORED) {
      glPopMatrix();  // Pop to 2 mirrored
    }
    
    if (render_hud_) {
      // Render the FPS and status string
      glPushMatrix();  // Push to 3
      glScalef(1.0f/image_width_, 1.0f/image_height_, 1.0f);
      if (kinect_->getNumUsersTracked() > 0) {
        glColor3f(0.0f, 0.0f, 1.0f);
      } else {
        glColor3f(1.0f, 0.0f, 0.0f);
      }
      glLineWidth(2);
      snprintf(hud_string_, sizeof(hud_string_), 
               "%s%s", kinect_->getStatusMessage(), fps_string_);
      renderStrokeFontString(0, 10.0f, GLUT_STROKE_ROMAN, 
                             reinterpret_cast<const unsigned char *>(hud_string_), 
                             0.15f);
      glPopMatrix();  // Pop to 2
    }
    
    glPopMatrix();  // Pop to 1
    
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();  // Pop to 0
  }
  
  void App::renderBlackScreenWithText(const string& text) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, window_width_, window_height_);
    glPushMatrix();
    glLineWidth(1);
    glScalef(2.0f/image_width_, 2.0f/image_height_, 1.0f);  // viewport tf
    glTranslatef(-image_width_/2, -image_height_/2, 0.0f);
    renderStrokeFontString(0, 10.0f, GLUT_STROKE_ROMAN, 
                           reinterpret_cast<const unsigned char *>(text.c_str()), 0.2f);
    glPopMatrix();
    glutSwapBuffers();
  }
  
  void App::renderStrokeFontString(float x, float y, void *font,
                                   const unsigned char *string, float scale) {
    glPushMatrix();
    glTranslatef(x, y, 0);
    glScalef(scale, scale, scale);
    for (const unsigned char *c = string; *c; c++)
      glutStrokeCharacter(font, *c);
    glPopMatrix();
  }
  
  void App::InitKinect() {
    try {
      static std::string init_str("Initializing the Kinect");
      g_app->setKinectStatusStr(init_str);
      g_app->kinect_ = new KinectInterface(g_app->tracking_skeleton_);
    } catch(std::runtime_error e) {
      printf("%s\n", e.what());
      static std::string error_str("Kinect not connected (or not found)");
      g_app->setKinectStatusStr(error_str);
      return;
    }
    if (g_app->image_width_ != g_app->kinect_->getWidth() || 
        g_app->image_height_ != g_app->kinect_->getHeight()) {
      string err(string("ERROR: initKinect() - kinect size doesn't match +") + 
                 string("is not App::image_width_ x App::image_height_!"));
      throw runtime_error(err);
    }
    static std::string ok_str("Kinect connected");
    g_app->setKinectStatusStr(ok_str);
    g_app->setKinectStatus(true);
    g_app->kinect_->setCalculateHandStatistics(g_app->calculate_hand_statistics_);
    g_app->kinect_->setCalculateHandPoints(g_app->calculate_hand_statistics_);
  }
  
  void App::initGLState(void) {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    
    glDisable(GL_LIGHTING);
    // glEnable(GL_DEPTH_TEST);
    glDisable(GL_TEXTURE_2D);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    
    glPointSize(2.0f);
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }
  
  void App::initTextures() { 
    glEnable(GL_TEXTURE_2D);
    glGenTextures(TEXTURE_ID_NUM_TEXTURES, texture_);
    
    // Set the texture parameters
    for (int i = 0; i < TEXTURE_ID_NUM_TEXTURES; i++) {
      glBindTexture(GL_TEXTURE_2D, texture_[i]); 
      glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }
  }
  
  void App::initDownstreamPipe() {
    clientpipe = new BytePipe("tcp://*:5557");
  }

  float round(float r) {
    return (r > 0.0f) ? floor(r + 0.5f) : ceil(r - 0.5f);
  }
  
  void App::updateServer() {
    // Assume: kinect_->lockData() is already called
    unsigned char* rgbpix = NULL;
    unsigned char* depthpix = NULL;
    
    // SEND RGB IMAGE //
    rgbpix = reinterpret_cast<unsigned char*>(
      const_cast<XnRGB24Pixel*>(kinect_->getRGBImageUnsafe()));
    uint64_t rgb_bytes = 640 * 480 * 3;      
    zmq::message_t rgb_message(rgb_bytes);
    memcpy(reinterpret_cast<unsigned char*>(rgb_message.data()), 
           rgbpix, rgb_bytes);
    clientpipe->sendWithKey("v", rgb_message);
    
    // SEND DEPTH IMAGE //
    depthpix = reinterpret_cast<unsigned char*>(
      const_cast<XnDepthPixel*>(kinect_->getDepthRawUnsafe()));
    uint64_t depth_bytes = 640 * 480 * 2;
    zmq::message_t depth_message(depth_bytes);
    memcpy(reinterpret_cast<unsigned char*>(depth_message.data()), 
           depthpix, depth_bytes);
    clientpipe->sendWithKey("d", depth_message);
    
    // SEND LEFT HAND DATA //
    HandStatistics* lHand = kinect_->getLHand();
    uint64_t lhand_bytes = lHand->computeOutMatrix();
    //      zmq::message_t lhand_message(lhand_bytes);
    zmq::message_t lhand_message(LEFTHAND_FRAME_BYTES);
    memcpy(reinterpret_cast<unsigned char*>(lhand_message.data()), 
           (unsigned char*)lHand->outmat, lhand_bytes);
    clientpipe->sendWithKey("l", lhand_message);
    
    // SEND RIGHT HAND DATA //
    HandStatistics* rHand = kinect_->getRHand();
    uint64_t rhand_bytes = rHand->computeOutMatrix();
    //      zmq::message_t rhand_message(rhand_bytes);
    zmq::message_t rhand_message(RIGHTHAND_FRAME_BYTES);
    memcpy(reinterpret_cast<unsigned char*>(rhand_message.data()), 
           reinterpret_cast<unsigned char*>(rHand->outmat), rhand_bytes);
    clientpipe->sendWithKey("r", rhand_message);
    
    
    // new code to send mask of hand
    if (hands_mask == NULL) {
      hands_mask = new unsigned char[image_width_ * image_height_];
    }
    
    // Collect the left hand points
    if (lHand->npts >= kinect::HandStatistics::min_npts) {
      
      std::fill_n(hands_mask, image_width_*image_height_, 
                  static_cast<char>(0));    
      
      // We need the point cloud data in UV space (so it alignes with RGB data)
      lHand->convertPointsToUVD();
      //rHand->convertPointsToUVD();
      
      for (int i = 0; i < lHand->npts; i++) {
        int u = static_cast<int>(round(*lHand->pts_UVD.at(i*3)));
        int v = static_cast<int>(round(*lHand->pts_UVD.at(i*3 + 1)));
        
        if (u >= 0 && u < image_width_ && v >= 0 && v <= image_height_) {
          int cur_index_output_array = (v*image_width_ + u);
          hands_mask[cur_index_output_array] |= 1;
        }
      }
    }
    
    // Collect the right hand points
    if (rHand->npts >= kinect::HandStatistics::min_npts) {
      
      // We need the point cloud data in UV space (so it alignes with RGB data)
      rHand->convertPointsToUVD();
      
      for (int i = 0; i < rHand->npts; i++) {
        int u = static_cast<int>(round(*rHand->pts_UVD.at(i*3)));
        int v = static_cast<int>(round(*rHand->pts_UVD.at(i*3 + 1)));
        
        if (u >= 0 && u < image_width_ && v >= 0 && v <= image_height_) {
          int cur_index_output_array = (v*image_width_ + u);
          hands_mask[cur_index_output_array] |= 2;
        }
      }
    }
    /////////////////////////////////
    
    
    // SEND HANDS MASK IMAGE //
    uint64_t hands_mask_bytes = image_width_ * image_height_;
    zmq::message_t hands_mask_message(hands_mask_bytes);
    memcpy(reinterpret_cast<unsigned char*>(hands_mask_message.data()), 
           hands_mask, hands_mask_bytes);
    clientpipe->sendWithKey("m", hands_mask_message);
  }
  
};  // namespace app


