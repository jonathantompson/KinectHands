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

#define MAX_NUM_KINECTS 4
#define NUM_APP_WORKER_THREADS 4

#if defined(_WIN32)
class DebugBuf;
#endif

namespace kinect_interface { namespace hand_detector { class HandDetector; } }
namespace jtil { namespace threading { class ThreadPool; } }
namespace jtil { namespace renderer { class GeometryInstance; } }
namespace jzmq { class Connection; }

namespace app {
  struct FrameData;

  typedef enum {
    OUTPUT_RGB = 0,
    OUTPUT_DEPTH = 1,
    OUTPUT_DEPTH_UNDISTORT = 2,
    OUTPUT_DEPTH_ALL_VIEWS = 3,
    OUTPUT_DEPTH_RAINBOW = 4,
    OUTPUT_DEPTH_COLORED = 5,
    OUTPUT_BLUE = 6,
  } KinectOutput;

  typedef enum {
    RDF_LABELS_NONE = 0,
    RDF_LABELS_UNFILTERED = 1,
    RDF_LABELS_FILTERED = 2,
    RDF_LABELS_FINAL = 3,
  } RDFLabels;

  typedef enum {
    BLUE_BACKGROUND = 0,
    LBLUE_BACKGROUND = 1,
    WHITE_BACKGROUND = 2,
  } BackgroundColor;

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
    static void resetCameraCB();

  private:
    static App* g_app_;  // Global singleton

#if defined(_WIN32)
    static DebugBuf* debug_buf;
#endif
    bool app_running_;

    bool is_time_server_;
    jzmq::Connection* time_server_conn_;  // Either a publisher or subscriber

    // Kinect data
    kinect_interface::KinectInterface* kinects_[MAX_NUM_KINECTS];
    int64_t kinect_last_saved_depth_time_[MAX_NUM_KINECTS];
    uint32_t num_kinects_;
    uint8_t rainbowPalletR[256];
    uint8_t rainbowPalletG[256];
    uint8_t rainbowPalletB[256];

    jtil::clk::Clk* clk_;
    jtil::math::Double2 mouse_pos_;
    jtil::math::Double2 mouse_pos_old_;
    double frame_time_;
    double time_since_start_;
    double remote_time_since_start_;
    double frame_time_prev_;
    static uint64_t screenshot_counter_;
    uint64_t depth_frame_number_;
    int64_t depth_frame_time_;

    // We copy the kinect data here (at the cost of another O(n) copy) to avoid
    // holding the kinect's data lock too long.
    uint16_t depth_[kinect_interface::depth_dim];
    uint16_t depth_undistorted_[kinect_interface::depth_dim];
    uint8_t depth_colored_[kinect_interface::depth_dim * 3];
    uint8_t rgb_[kinect_interface::rgb_dim * 3];
    float xyz_[kinect_interface::depth_dim * 3];
    float* depth_undistort_lookup_table;

    jtil::renderer::Texture* depth_tex_;
    jtil::renderer::Texture* rgb_tex_;
    uint8_t depth_im_[kinect_interface::depth_dim * 3];
    uint8_t rgb_im_[kinect_interface::rgb_dim * 3];
    jtil::renderer::GeometryInstance* geom_inst_pts_;  // Not owned here!
    jtil::renderer::Geometry* geom_pts_;  // Not owned here!
    jtil::renderer::GeometryInstance* geom_inst_joints_;  // Not owned here!
    jtil::renderer::Geometry* geom_joints_;  // Not owned here!

    // This temporary array is used to store the data to be saved.  We allocate
    // a little extra space just in case the compression inflates the data
    // (however this is very unlikely)
    uint16_t tmp_data1_[kinect_interface::depth_arr_size_bytes*MAX_NUM_KINECTS*2];
    uint16_t tmp_data2_[kinect_interface::depth_arr_size_bytes*MAX_NUM_KINECTS*2];
    
    // Randomized Decision Forest Hand Detector
    kinect_interface::hand_detector::HandDetector* hd_;
    uint8_t hand_labels_[kinect_interface::depth_dim];

    void run();
    void init();
    static void resetScreenCB();
    static int closeWndCB();
    void moveCamera(const double dt);
    void moveStuff(const double dt);
    void addStuff();
    void registerNewRenderer();
    void initRainbowPallet();

    // Multithreading
    // Thread pool to get the KinectData from the kinects in parallel:
    jtil::threading::ThreadPool* tp_;
    int32_t threads_finished_;
    std::mutex thread_update_lock_;
    std::condition_variable not_finished_;
    jtil::data_str::VectorManaged<jtil::threading::Callback<void>*>* data_save_cbs_; 
    void saveKinectData(const uint32_t index);
    void executeThreadCallbacks(jtil::threading::ThreadPool* tp, 
      jtil::data_str::VectorManaged<jtil::threading::Callback<void>*>* cbs);

    // Non-copyable, non-assignable.
    App(App&);
    App& operator=(const App&);
  };

};  // namespace app

#endif  // APP_APP_HEADER
