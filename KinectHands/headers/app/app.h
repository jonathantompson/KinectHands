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
#include "app/frame_data.h"
#include "jtil/jtil.h"
#include "kinect_interface/kinect_interface.h"
#include "kinect_interface/hand_net/hand_image_generator.h"  // for HN_IM_SIZE
#include "kinect_interface/hand_net/hand_net.h"  // for HandCoeffConvnet

#define MAX_NUM_KINECTS 4
#define NUM_APP_WORKER_THREADS 4

#if defined(_WIN32)
class DebugBuf;
#endif

namespace kinect_interface { class KinectInterface; }
namespace kinect_interface { namespace hand_net { class HandNet; } }
namespace kinect_interface { namespace hand_net { class HandModelCoeff; } }
namespace kinect_interface { namespace hand_net { class RobotHandModel; } }

namespace app {
  struct FrameData;

  typedef enum {
    OUTPUT_RGB_IR = 0,
    OUTPUT_RGB_REGISTERED = 1,
    OUTPUT_DEPTH = 2,
    OUTPUT_DEPTH_ALL_VIEWS = 3,
    OUTPUT_DEPTH_RAINBOW = 4,
    OUTPUT_HAND_DETECTOR_DEPTH = 5,
    OUTPUT_CONVNET_DEPTH = 6,
    OUTPUT_CONVNET_SRC_DEPTH = 7,
    OUTPUT_CONVNET_HEAT_MAPS = 8,
    OUTPUT_HAND_NORMALS = 9,
    OUTPUT_BLUE = 10,
  } KinectOutput;

  typedef enum {
    OUTPUT_NO_LABELS = 0,
    OUTPUT_UNFILTERED_LABELS = 1,
    OUTPUT_FILTERED_LABELS = 2,
    OUTPUT_FLOODFILL_LABELS = 3
  } LabelType;

  typedef enum {
    HAND_TYPE_NONE = 0,
    HAND_TYPE_LIBHAND = 1,
    HAND_TYPE_ROBOT = 2,
  } HandModelType;

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
    static void keyboardCB(int key, int scancode, int action, int mods);
    static void mousePosCB(double x, double y);
    static void mouseButtonCB(int button, int action, int mods);
    static void mouseWheelCB(double xoffset, double yoffset);
    static void screenshotCB();
    static void resetTrackingCB();
    static void greyscaleScreenshotCB();

  private:
    static App* g_app_;  // Global singleton

#if defined(_WIN32)
    static DebugBuf* debug_buf;
#endif
    bool app_running_;

    // Kinect data
    jtil::data_str::VectorManaged<char*> kinect_uris_;
    kinect_interface::KinectInterface* kinect_[MAX_NUM_KINECTS];
    FrameData* kdata_[MAX_NUM_KINECTS];
    bool new_data_;
    uint8_t rainbowPalletR[256];
    uint8_t rainbowPalletG[256];
    uint8_t rainbowPalletB[256];

    // Convolutional Neural Network
    kinect_interface::hand_net::HandNet* hand_net_;
    kinect_interface::hand_net::HandModelCoeff* hands_[2];
    kinect_interface::hand_net::RobotHandModel* robot_hand_model_;

    jtil::clk::Clk* clk_;
    jtil::math::Double2 mouse_pos_;
    jtil::math::Double2 mouse_pos_old_;
    double frame_time_;
    double frame_time_prev_;
    static uint32_t screenshot_counter_;

    uint8_t render_labels_[src_dim];

    // Typically 640 x 480
    jtil::renderer::Texture* background_tex_;
    uint8_t im_[src_dim * 3];
    uint16_t depth_tmp_[src_dim];
    uint8_t im_flipped_[src_dim * 3];

    // Typically 96 x 96
    jtil::renderer::Texture* convnet_background_tex_;  // smaller dimension
    uint8_t convnet_im_flipped_[HN_IM_SIZE * HN_IM_SIZE * 3];

    // Typically 384 x 384
    jtil::renderer::Texture* convnet_src_background_tex_;  // smaller dimension
    uint8_t convnet_src_im_flipped_[HN_SRC_IM_SIZE * HN_SRC_IM_SIZE * 3];

    // Typically (24 * 3) x (24 * 3)
    jtil::renderer::Texture* convnet_hm_background_tex_;  // smaller dimension
    uint8_t* convnet_hm_im_flipped_;
    uint32_t hm_size_;
    uint32_t hm_nfeats_;
    jtil::math::Int2 hm_feats_dim_;  // Number of tiles width and height

    // In-air drawing
    bool drawing_;
    bool was_drawing_;
    jtil::math::Int2 prev_pen_position_;
    jtil::math::Int2 pen_position_;
    uint8_t* drawing_canvas_;

    void run();
    void init();
    static void resetScreenCB();
    static int closeWndCB();
    void moveCamera(const double dt);
    void moveStuff(const double dt);  // Temporary: just to play with renderer
    void addStuff();
    void registerNewRenderer();
    void initRainbowPallet();

    // Thread pool to get the KinectData from the kinects in parallel
    jtil::threading::ThreadPool* tp_;
    int32_t threads_finished_;
    std::mutex thread_update_lock_;
    std::condition_variable not_finished_;
    jtil::data_str::VectorManaged<jtil::threading::Callback<void>*>* kinect_update_cbs_; 
    jtil::data_str::VectorManaged<jtil::threading::Callback<void>*>* data_save_cbs_; 
    void syncKinectData(const uint32_t index);
    void saveKinectData(const uint32_t index);
    void executeThreadCallbacks(jtil::threading::ThreadPool* tp, 
      jtil::data_str::VectorManaged<jtil::threading::Callback<void>*>* cbs);

    // Non-copyable, non-assignable.
    App(App&);
    App& operator=(const App&);
  };

};  // namespace app

#endif  // APP_APP_HEADER
