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
#include "threading/thread.h"
#include "threading/callback.h"

#ifdef _WIN32
  #ifndef snprintf
    #define snprintf _snprintf
  #endif
#endif

using std::string;
using std::runtime_error;
using kinect::KinectInterface;
using kinect::HandStatistics;
using std::thread;
using threading::MakeThread;
using threading::Callback;
using threading::MakeCallableOnce;
using threading::MakeCallableMany;

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
    // window_width_ = image_width_;
    // window_height_ = image_height_;    
    
    kinect_ = NULL;
    clk_ = new Clock();
    t1_ = clk_->getTime();
    blur_effect_ = new Effects<int>(image_width_, image_height_);
    
    cur_display_image_ = TEXTURE_ID_RGB_IMAGE;
    render_data_ = true;
    first_frame_ = true;
    render_hand_points_ = true;
    render_skeleton_ = true;
    render_hud_ = true;
    frame_num_ = 0;
    setKinectStatus(false);
    static std::string str("Looking for Kinect...");
    setKinectStatusStr(str);
    snprintf(fps_string_, sizeof(fps_string_), "");
    tracking_skeleton_ = true;
    shutdown_requested_ = false;
  }
  
  App::~App() {
    app_running = false;
    
    if (kinect_) {
      // Wait for all kinect threads to finish
      if (getKinectStatus()) {
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
    
    delete hands_mask;
    
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
    glutAddMenuEntry("Render: Skeleton", MENU_ID_RENDER_SKELETON);
    glutAddMenuEntry("Render: Hand Point Cloud", MENU_ID_RENDER_HAND_POINTS);
    glutAddMenuEntry("Render: HUD", MENU_ID_RENDER_HUD);
    glutAddMenuEntry("-----------------------------------------", 
                     MENU_ID_BLANK);
    glutAddMenuEntry("Store calibration data", MENU_ID_STORE_CALIBRATION_DATA);
    glutAddMenuEntry("-----------------------------------------", 
                     MENU_ID_BLANK);
    glutAddMenuEntry("Track skeleton ON / OFF", MENU_ID_TRACK_SKELETON_ON_OFF);
    glutAddMenuEntry("Reset tracking", MENU_ID_RESET_TRACKING);
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
        if (g_app->getKinectStatus()) {
          g_app->getKinectData();
        } 
        
        glutSetWindow(g_app->main_window_);
        glutPostRedisplay();
        
        std::this_thread::yield();
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
  
  void App::getKinectData() {
    // Save the image texture
    kinect_->lockData();
    XnRGB24Pixel* rgb_image;
    XnUInt8* depth_image_;
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
        //kinect_->lockData();
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
        //kinect_->unlockData();
      case TEXTURE_ID_NUM_TEXTURES:
        break;
    }
    kinect_->unlockData();
  }
  
  void App::menuSelected(int id) {
    static string reset_txt("Resetting user tracking, please wait");
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
      case MENU_ID_RESET_TRACKING:
        g_app->renderBlackScreenWithText(reset_txt);
        g_app->kinect_->ResetTracking(g_app->tracking_skeleton_);
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
      case MENU_ID_RENDER_HUD:
        g_app->render_hud_ = !g_app->render_hud_;
        break;     
    }
  }
  
  void App::keyboard(unsigned char key, int x, int y) {
    switch (key) {
      case 27:  // ESC
      case 'q':
      case 'Q':
        RequestShutdown();
        break;
      case 's':
        g_app->kinect_->SaveCalibration();
        break;
      case 'l':
        g_app->kinect_->LoadCalibration();
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
    
    kinect_->drawHandStatistics(render_hand_points_, false, false);
    
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
  
  // InitKinect() is run asynchronously in a seperate thread on startup
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
    g_app->kinect_->setCalculateHandStatistics(false);
    g_app->kinect_->setCalculateHandPoints(true);  
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

};  // namespace app


