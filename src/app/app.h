//
//  app.h
//
//  Created by Jonathan Tompson on 5/15/12.
//
//  Main (singleton) application class for KinectHands
//
//  Some of this code was created by Otavio Braga (obraga@cs.nyu.edu) and 
//  modified by Jonathan Tompson (tompson@cs.nyu.edu)
// 
#include <mutex>
#include <thread>
#include <stdlib.h>
#ifdef __APPLE__
#include <unistd.h>
#endif
#include <string>
#include <iostream>
#include "rendering/opengl_include.h"
#include "XnPlatform.h"
#include "zmq/bytepipe.h"
#include "kinect_interface/hand_statistics.h"
#include <sstream>

#ifndef APP_APP_HEADER
#define APP_APP_HEADER

#define NUM_KINECTS 1  // No support for more than one yet
#define RENDER_MIRRORED false

namespace kinect { class KinectInterface; }
class Clock;
template <class T> class Effects;

namespace app {
  
  // Glut menu ids
  enum {
    MENU_ID_RGB_IMAGE,
    MENU_ID_DEPTH_IMAGE,
    MENU_ID_RGB_AND_DEPTH_IMAGE,
    MENU_ID_HAND_LABELS_IMAGE,
    MENU_ID_LOAD_CALIBRATION_DATA,
    MENU_ID_STORE_CALIBRATION_DATA,
    MENU_ID_RENDER_DATA_ON_OFF,
    MENU_ID_TRACK_SKELETON_ON_OFF,
    MENU_ID_FIND_HAND_POINTS,
    MENU_ID_CALCULATE_HAND_STATISTICS,
    MENU_ID_RESET_TRACKING,
    MENU_ID_RESET_KINECT,
    MENU_ID_RGB_BLUR,
    MENU_ID_RENDER_HAND_POINTS,
    MENU_ID_RENDER_FINGER_POINTS,
    MENU_ID_RENDER_BOUNDING_BOX_POINTS,
    MENU_ID_RENDER_SKELETON,
    MENU_ID_RENDER_HUD,
    MENU_ID_RENDER_HAND_IMAGES,
    MENU_ID_HAND_STATISTICS_INCREASE_BLUR_RAD,
    MENU_ID_HAND_STATISTICS_DECREASE_BLUR_RAD,
    MENU_ID_HAND_STATISTICS_INCREASE_FINGER_THRESHOLD,
    MENU_ID_HAND_STATISTICS_DECREASE_FINGER_THRESHOLD,
    MENU_ID_HAND_STATISTICS_INCREASE_FINGER_INNER_RAD,
    MENU_ID_HAND_STATISTICS_DECREASE_FINGER_INNER_RAD,
    MENU_ID_HAND_STATISTICS_INCREASE_FINGER_OUTER_RAD,
    MENU_ID_HAND_STATISTICS_DECREASE_FINGER_OUTER_RAD, 
    MENU_ID_HAND_DETECTOR_INCREASE_STAGE1_SHRINK_FILT_RAD, 
    MENU_ID_HAND_DETECTOR_DECREASE_STAGE1_SHRINK_FILT_RAD, 
    MENU_ID_HAND_DETECTOR_INCREASE_STAGE2_MED_FILT_RAD, 
    MENU_ID_HAND_DETECTOR_DECREASE_STAGE2_MED_FILT_RAD, 
    MENU_ID_HAND_DETECTOR_INCREASE_NUM_DECISION_TREES, 
    MENU_ID_HAND_DETECTOR_DECREASE_NUM_DECISION_TREES, 
    MENU_ID_HAND_DETECTOR_INCREASE_MAX_HEIGHT_TO_EVALUATE, 
    MENU_ID_HAND_DETECTOR_DECREASE_MAX_HEIGHT_TO_EVALUATE, 
    MENU_ID_BLANK,
  };
  
  enum TEXTURE_ID {
    TEXTURE_ID_RGB_IMAGE,
    TEXTURE_ID_RGB_BLUR_IMAGE,
    TEXTURE_ID_DEPTH_IMAGE,
    TEXTURE_ID_DEPTH_AND_RGB_IMAGE,
    TEXTURE_ID_LHAND_FINGERS,
    TEXTURE_ID_RHAND_FINGERS,
    TEXTURE_ID_HAND_LABELS,
    TEXTURE_ID_NUM_TEXTURES
  };

  class App {
  public:
    App();
    ~App();
    static App* g_app;
    
    // Static functions called by main (interface to class)
    static void InitApp(int argc, char *argv[]);
    static void RunApp();
    static void RequestShutdown();
    static void KillApp();
    static void PrintUsage();  // to std out 
    
    // Static functions for OpenGL callbacks
    static void display();
    static void keyboard(unsigned char key, int x, int y);
    static void reshape(const int width, const int height);
    static void motion(const int x, const int y);
    static void mouse(int button, int state, int x, int y);
    static void menuSelected(int id);
    static void selectDisplay(int id);
    static void idle();
    static void click(int x, int y);
    static void renderStrokeFontString(float x, float y, void *font, 
                                       const unsigned char *string, 
                                       float scale);
    static double getTime();
    inline static kinect::KinectInterface* getKinect() { return g_app->kinect_; }

    static bool app_running;
    
    // tmp data to copy out hand data

  private:
    int main_window_;  // Glut window id
    static const int image_width_ = 640;
    static const int image_height_ = 480;
    static const int hand_image_dim_ = kinect::HandStatistics::dim_hand_pixels;
    static const int npts_ = image_width_*image_height_;
    int window_width_;
    int window_height_;
    kinect::KinectInterface* kinect_;
    Clock* clk_;
    Effects<int>* blur_effect_;
    TEXTURE_ID cur_display_image_;
    bool render_data_;
    bool first_frame_;
    bool render_hand_points_;
    bool render_finger_points_;
    bool render_bounding_box_points_;
    bool render_skeleton_;
    bool render_hud_;
    bool render_hand_images_;
    bool tracking_skeleton_;
    GLuint texture_[TEXTURE_ID_NUM_TEXTURES];  // The texture ids
    uint64_t frame_num_;
    uint64_t zmq_frame_number;
    char fps_string_[256];
    char hud_string_[256];
    static const int frame_num_max_ = 30;
    bool kinect_status_;
    std::string kinect_status_str_;
    std::mutex kinect_status_lock_;
    bool find_hand_points_;
    bool calculate_hand_statistics_;
    std::thread kinect_thread_;
    const static std::string hand_data_filename_;
    double t0_, t1_;
    double fps_accumulate_;
    double snapshot_timer_;
    bool continuous_snapshot_;
    bool continuous_dt_depth_snapshot_;  // Record data for decision tree
    bool continuous_hp_depth_snapshot_;  // Record data for hand fit
    bool shutdown_requested_;
    
    // Some temp data for the blur routine
    int tmpImageR[image_width_*image_height_];
    int tmpImageG[image_width_*image_height_];
    int tmpImageB[image_width_*image_height_];
    XnUInt8 tmpImageRGB[image_width_*image_height_*3];
    XnUInt8 lHandImageRGB[hand_image_dim_*hand_image_dim_*3];
    XnUInt8 rHandImageRGB[hand_image_dim_*hand_image_dim_*3];
  
    // some data for handmask
    unsigned char* hands_mask;

    void render();
    static void InitKinect();  // Run in a seperate thread on startup    
    void initTextures();
    void initGLState();
    void initDownstreamPipe();
    void drawKinectData();
    bool getKinectData();
    void updateServer();
    void renderBlackScreenWithText(const std::string& text);
    void setKinectStatusStr(std::string& str);
    void setKinectStatus(bool status);
    bool getKinectStatus();
    std::string getKinectStatusStr();
    void drawQuad();
    
    BytePipe* clientpipe;
  };

};  // namespace app

#endif  // APP_APP_HEADER

