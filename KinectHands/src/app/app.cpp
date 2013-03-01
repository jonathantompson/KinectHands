#include <stdio.h>  // for printf
#include <thread>
#include <string>
#include <iostream>  // for cout
#include "app/app.h"
#include "jtil/ui/ui.h"
#include "kinect_interface/kinect_interface.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"
#include "jtil/glew/glew.h"
#include "jtil/image_util/image_util.h"

#ifndef NULL
#define NULL 0
#endif
#define SAFE_DELETE(x) if (x) { delete x; x = NULL; }

using std::wruntime_error;
using namespace jtil;
using namespace jtil::data_str;
using namespace jtil::clk;
using namespace jtil::renderer;
using namespace jtil::settings;
using namespace jtil::math;
using namespace jtil::settings;
using namespace kinect_interface;
using namespace jtil::renderer;
using namespace jtil::image_util;

namespace app {

  App* App::g_app_ = NULL;
#if defined(_WIN32)
  DebugBuf* App::debug_buf = NULL;
  std::streambuf* old_cout_buf = NULL;
#endif

  App::App() {
    app_running_ = false;
    clk_ = NULL;
    kinect_ = NULL;
    rand_norm_ = new NORM_DIST<float>(0.0f, 1.0f);
    rand_uni_ = new UNIF_DIST<float>(-1.0f, 1.0f);
    kinect_frame_number_ = MAX_UINT64;
  }

  App::~App() {
    kinect_->shutdownKinect();
    SAFE_DELETE(kinect_);
    SAFE_DELETE(clk_);
    SAFE_DELETE(rand_norm_);
    SAFE_DELETE(rand_uni_);
    Renderer::ShutdownRenderer();
  }

  void App::newApp() {
#if defined(_WIN32)
    // If in windows and using a debug build we should also redirect the std
    // output to the debug window (in visual studio)
    if (IsDebuggerPresent()) {
      debug_buf = new DebugBuf();
      old_cout_buf = std::cout.rdbuf(debug_buf);  // for 'std::cout << x' calls
      std::cout << "WARNING: std::cout redirected to Debug window."
        << " Remember to call '<< std::endl' to flush." << std::endl;
      // TO DO: WORK OUT HOW TO REDIRECT PRINTF
    }
#endif
    g_app_ = new App();
  }

  void App::initApp() {
    g_app_->init();
  }


  void App::init() {
    Renderer::InitRenderer();
    registerNewRenderer();

    clk_ = new Clk();
    frame_time_ = clk_->getTime();
    app_running_ = true;

    kinect_ = new KinectInterface();
  }

  void App::killApp() {
    SAFE_DELETE(g_app_);

#if defined(_WIN32)
    // make sure to restore the original cout so we don't get a crash on close!
    std::cout << std::endl;  // Force a flush on exit
    std::cout.rdbuf(old_cout_buf);
    SAFE_DELETE(debug_buf);
#endif
  }

  void App::runApp() {
    g_app_->run();
  }

  void App::registerNewRenderer() {
    Renderer::g_renderer()->registerKeyboardCB(App::keyboardCB);
    Renderer::g_renderer()->registerMousePosCB(App::mousePosCB);
    Renderer::g_renderer()->registerMouseButCB(App::mouseButtonCB);
    Renderer::g_renderer()->registerMouseWheelCB(App::mouseWheelCB);
    Renderer::g_renderer()->registerCharInputCB(App::characterInputCB);
    Renderer::g_renderer()->registerResetScreenCB(App::resetScreenCB);
    Renderer::g_renderer()->getMousePosition(g_app_->mouse_pos_);
    Renderer::g_renderer()->registerCloseWndCB(App::closeWndCB);
    g_app_->addStuff();
  }

  void App::run() {
    while (app_running_) {
      frame_time_prev_ = frame_time_;
      frame_time_ = clk_->getTime();
      double dt = frame_time_ - frame_time_prev_;

      // Grab the kinect data
      kinect_->lockData();
      bool update_tex = false;
      if (kinect_frame_number_ != kinect_->frame_number()) {
        memcpy(rgb_, kinect_->rgb(), sizeof(rgb_[0]) * src_dim * 3);
        memcpy(depth_, kinect_->depth(), sizeof(depth_[0]) * src_dim);

        bool df_labels;
        GET_SETTING("render_decision_forest_labels", bool, df_labels);
        if (df_labels) {
          const uint32_t w = src_width / DT_DOWNSAMPLE;
          const uint8_t* hdlabels = kinect_->filteredDecisionForestLabels();
          for (uint32_t v = 0; v < src_height; v++) {
            for (uint32_t u = 0; u < src_width; u++) {
              uint8_t val = hdlabels[(v / DT_DOWNSAMPLE) * w + u/DT_DOWNSAMPLE];
              uint32_t idst = v * src_width + u;
              // Texture needs to be flipped vertically and 0 --> 255
              labels_[idst] = val;
            }
          }
        } else {
          memcpy(labels_, kinect_->labels(), sizeof(labels_[0]) * src_dim);
        }

        kinect_frame_number_ = kinect_->frame_number();
        update_tex = true;
      }
      kinect_->unlockData();

      if (update_tex) {
        int kinect_output;
        GET_SETTING("kinect_output", int, kinect_output);
        switch (kinect_output) {
        case OUTPUT_RGB:
          memcpy(im_, rgb_, sizeof(im_[0]) * src_dim * 3);
          break;
        case OUTPUT_DEPTH:
          for (uint32_t i = 0; i < src_dim; i++) {
            uint8_t val = (depth_[i] * 2) % 255;
            im_[i*3] = val;
            im_[i*3+1] = val;
            im_[i*3+2] = val;
          }
          break;
        default:
          throw std::wruntime_error("App::run() - ERROR: output image type"
            "not recognized!");
        }

        bool render_hand_labels;
        GET_SETTING("render_hand_labels", bool, render_hand_labels);
        if (render_hand_labels) {
          // Make hand points red
          for (uint32_t i = 0; i < src_dim; i++) {
            if (labels_[i] == 1) {
              im_[i*3] = (uint8_t)std::min<uint16_t>(255, 
                3 * (uint16_t)im_[i*3] / 2);
              im_[i*3 + 1] = (uint8_t)(3 * (uint16_t)im_[i*3+1] / 4);
              im_[i*3 + 2] = (uint8_t)(3 * (uint16_t)im_[i*3+2] / 4);
            }
          }
        }

        FlipImage<uint8_t>(im_flipped_, im_, src_width, src_height, 3);
        background_tex_->flagDirty();
      }

      // Update camera based on real-time inputs
      if (!Renderer::ui()->mouse_over_ui()) {
        moveCamera(dt);
      }
      moveStuff(dt);

      Renderer::g_renderer()->renderFrame();
 
      // Give OS the opportunity to deschedule
      std::this_thread::sleep_for(std::chrono::milliseconds(2));  
      std::this_thread::yield();
    }
  }
  
  void App::moveCamera(double dt) {
    
  }

  void App::moveStuff(const double dt) {

  }

  void App::addStuff() {
    memset(im_flipped_, 0, sizeof(im_flipped_[0]) * src_dim * 3);
    background_tex_ = new Texture(GL_RGB, src_width, src_height, GL_RGB, 
      GL_UNSIGNED_BYTE, (unsigned char*)im_flipped_, false,
      TextureWrapMode::TEXTURE_CLAMP, TextureFilterMode::TEXTURE_LINEAR, 
      false);

    // Transfer ownership of the texture to the renderer
    Renderer::g_renderer()->setBackgroundTexture(background_tex_);

    ui::UI* ui = Renderer::g_renderer()->ui();
    ui->addHeadingText("Kinect Render Output:");
    ui->addSelectbox("kinect_output", "Kinect Output");
    ui->addSelectboxItem("kinect_output", ui::UIEnumVal(OUTPUT_RGB, "RGB"));
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_DEPTH, "Depth"));
    ui->addCheckbox("use_depth_from_file", "(Debug) Use Depth From File");

    ui->addHeadingText("Hand Detection:");
    ui->addCheckbox("detect_hands", "Enable Hand Detection");
    ui->addCheckbox("render_hand_labels", "Mark Hand Pixels");
    ui->addCheckbox("render_decision_forest_labels", 
      "Render Decision Forest Labels");
  }

  int App::closeWndCB() {
    g_app_->app_running_ = false;
    return 0;
  }

  void App::resetScreenCB() {
    g_app_->registerNewRenderer();
  }

  void App::keyboardCB(int key, int action) {
    switch (key) {
    case KEY_ESC:
      if (action == PRESSED) {
        requestShutdown();
      }
      break;
    case KEY_SPACE:
      if (action == PRESSED) {
        bool pause_physics;
        GET_SETTING("pause_physics", bool, pause_physics);
        SET_SETTING("pause_physics", bool, !pause_physics);
        Renderer::g_renderer()->ui()->setSettingsCheckboxVal("pause_physics",
          !pause_physics);
      }
      break;
    default:
      break;
    }
  }
  
  void App::mousePosCB(int x, int y) {

  }
  
  void App::mouseButtonCB(int button, int action) {
 
  }
  
  void App::mouseWheelCB(int pos) {

  }
  
  void App::characterInputCB(int character, int action) {

  }

}  // namespace app
