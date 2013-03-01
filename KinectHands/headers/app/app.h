//
//  app.h
//
//  Created by Jonathan Tompson on 5/1/12.
//

#ifndef APP_APP_HEADER
#define APP_APP_HEADER

#include <thread>
#include <string>
#include <random>
#include "jtil/jtil.h"
#include "kinect_interface/kinect_interface.h"

#define NUM_PT_LIGHTS 128
#define PT_LIGHT_ANIM_XDIM 15.0f
#define PT_LIGHT_ANIM_YDIM 15.0f
#define PT_LIGHT_ANIM_ZDIM 10.0f
#define PT_LIGHT_START_VEL 10.0f
#define PT_LIGHT_MAX_VEL 20.0f

#if defined(_WIN32)
class DebugBuf;
#endif

namespace kinect_interface { class KinectInterface; }

namespace app {

  typedef enum {
    OUTPUT_RGB = 0,
    OUTPUT_DEPTH = 1
  } KinectOutput;

  class App {
  public:
    App();
    ~App();

    // Static methods for interfacing with the global singleton
    static void newApp();  // Creates the g_app singleton
    static void initApp();  // Init routines
    static void runApp();  // Triggers main event loop
    static void killApp();  // Call once when you really want to shut down

    // Getter methods
    static inline App* app() { return g_app_; }
    
    static inline void requestShutdown() { g_app_->app_running_ = false; }
    static inline bool appRunning() { return g_app_->app_running_; }
    static void keyboardCB(const int key, const int action);
    static void mousePosCB(const int x, const int y);
    static void mouseButtonCB(const int button, const int action);
    static void mouseWheelCB(const int pos);
    static void characterInputCB(const int character, const int action);

  private:
    static App* g_app_;  // Global singleton

#if defined(_WIN32)
    static DebugBuf* debug_buf;
#endif
    bool app_running_;

    // Kinect data
    kinect_interface::KinectInterface* kinect_;
    uint64_t kinect_frame_number_;

    jtil::clk::Clk* clk_;
    jtil::math::Int2 mouse_pos_;
    jtil::math::Int2 mouse_pos_old_;
    double frame_time_;
    double frame_time_prev_;

    // Light animation data
    jtil::math::Float3 light_vel_[NUM_PT_LIGHTS];
    RAND_ENGINE rand_eng_;
    NORM_DIST<float>* rand_norm_;
    UNIF_DIST<float>* rand_uni_;

    jtil::renderer::Texture* background_tex_;  // not owned here
    uint8_t rgb_[src_dim * 3];
    uint8_t labels_[src_dim];
    uint16_t depth_[src_dim];
    uint8_t im_[src_dim * 3];
    uint8_t im_flipped_[src_dim * 3];

    void run();
    void init();
    static void resetScreenCB();
    static int closeWndCB();
    void moveCamera(const double dt);
    void moveStuff(const double dt);  // Temporary: just to play with renderer
    void addStuff();
    void registerNewRenderer();

    // Non-copyable, non-assignable.
    App(App&);
    App& operator=(const App&);
  };

};  // namespace app

#endif  // NAMESPACE_CLASS_TEMPLATE_HEADER
