#include <stdio.h>  // for printf
#include <thread>
#include <string>
#include <iostream>  // for cout
#include "app/app.h"
#include "jtil/ui/ui.h"
#include "kinect_interface/kinect_interface.h"
#include "kinect_interface/hand_detector/hand_detector.h"
#include "kinect_interface/hand_net/hand_image_generator.h"
#include "kinect_interface/hand_net/hand_model.h"  // for camera parameters
#include "kinect_interface/hand_detector/decision_tree_structs.h"
#include "jtil/glew/glew.h"
#include "jtil/image_util/image_util.h"

#ifndef NULL
#define NULL 0
#endif
#define SAFE_DELETE(x) if (x) { delete x; x = NULL; }

#ifdef _WIN32
#ifndef snprintf
#define snprintf _snprintf_s
#endif
#endif

using std::wruntime_error;
using namespace jtil;
using namespace jtil::data_str;
using namespace jtil::clk;
using namespace jtil::renderer;
using namespace jtil::settings;
using namespace jtil::math;
using namespace jtil::settings;
using namespace kinect_interface;
using kinect_interface::hand_net::HandCoeffConvnet;
using kinect_interface::hand_net::HandModel;
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
    if (kinect_) {
      kinect_->shutdownKinect();
    }
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

   // Set the camera to the kinect camera parameters
    float fov_deg = HAND_CAMERA_FOV;
    float view_plane_near = -HAND_CAMERA_VIEW_PLANE_NEAR;
    float view_plane_far = -HAND_CAMERA_VIEW_PLANE_FAR;
    SET_SETTING("fov_deg", float, fov_deg);
    SET_SETTING("view_plane_near", float, view_plane_near);
    SET_SETTING("view_plane_far", float, view_plane_far);

    g_app_->addStuff();
  }

  void App::run() {
    while (app_running_) {
      frame_time_prev_ = frame_time_;
      frame_time_ = clk_->getTime();
      double dt = frame_time_ - frame_time_prev_;

      int kinect_output;
      GET_SETTING("kinect_output", int, kinect_output);

      // Grab the kinect data
      kinect_->lockData();

      bool update_tex = false;
      if (kinect_frame_number_ != kinect_->frame_number()) {
        memcpy(rgb_, kinect_->rgb(), sizeof(rgb_[0]) * src_dim * 3);
        memcpy(depth_, kinect_->depth(), sizeof(depth_[0]) * src_dim);
        if (kinect_output == OUTPUT_HAND_DETECTOR_DEPTH) {
          image_util::UpsampleNoFiltering<uint16_t>(hand_detector_depth_, 
            (uint16_t*)kinect_->hand_detector()->depth_downsampled(), 
            src_width / DT_DOWNSAMPLE, src_height / DT_DOWNSAMPLE, 
            DT_DOWNSAMPLE);
        }

        memcpy(labels_, kinect_->labels(), sizeof(labels_[0]) * src_dim);

        int label_type_enum;
        GET_SETTING("label_type_enum", int, label_type_enum);
        switch ((LabelType)label_type_enum) {
        case OUTPUT_UNFILTERED_LABELS:
          UpsampleNoFiltering<uint8_t>(render_labels_, 
            kinect_->hand_detector()->labels_evaluated(), 
            src_width / DT_DOWNSAMPLE, src_height / DT_DOWNSAMPLE, DT_DOWNSAMPLE);
          break;
        case OUTPUT_FILTERED_LABELS:
          image_util::UpsampleNoFiltering<uint8_t>(render_labels_, 
            kinect_->filteredDecisionForestLabels(), src_width / DT_DOWNSAMPLE,
            src_height / DT_DOWNSAMPLE, DT_DOWNSAMPLE);
          break;
        case OUTPUT_FLOODFILL_LABELS:
          memcpy(render_labels_, labels_, sizeof(render_labels_[0]) * src_dim);
          break;
        default:
          throw std::wruntime_error("App::run() - ERROR - label_type_enum "
            "invalid enumerant!");
        }
        hand_pos_wh_.set(kinect_->hand_net()->image_generator()->hand_pos_wh());

        kinect_frame_number_ = kinect_->frame_number();
        update_tex = true;

        snprintf(kinect_fps_str_, 255, "Kinect FPS: %.2f", 
          (float)kinect_->fps());
        Renderer::g_renderer()->ui()->setTextWindowString("kinect_fps_wnd",
          kinect_fps_str_);
      }
      kinect_->unlockData();

      bool detect_pose;
      GET_SETTING("detect_pose", bool, detect_pose);
      if (detect_pose) {
        kinect_->detectPose(depth_, labels_);
        update_tex = true;
      }

      if (update_tex) {
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
        case OUTPUT_HAND_DETECTOR_DEPTH:
          for (uint32_t i = 0; i < src_dim; i++) {
            uint8_t val = (hand_detector_depth_[i] * 2) % 255;
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
            if (render_labels_[i] == 1) {
              im_[i*3] = (uint8_t)std::max<int16_t>(0, 
                (int16_t)im_[i*3] - 100);
              im_[i*3 + 1] = (uint8_t)std::min<int16_t>(255, 
                (int16_t)im_[i*3] + 100);
              im_[i*3 + 2] = (uint8_t)std::max<int16_t>(0, 
                (int16_t)im_[i*3] - 100);
            }
          }
        }

        FlipImage<uint8_t>(im_flipped_, im_, src_width, src_height, 3);

        bool render_convnet_points, detect_pose;
        GET_SETTING("detect_pose", bool, detect_pose);
        GET_SETTING("render_convnet_points", bool, render_convnet_points);
        if (render_convnet_points && detect_pose) {
          kinect_->hand_net()->image_generator()->annotateFeatsToKinectImage(
            im_flipped_, kinect_->hand_net()->coeff_convnet());
        }

        background_tex_->flagDirty();
      }

      // Update camera based on real-time inputs
      if (!Renderer::ui()->mouse_over_ui()) {
        moveCamera(dt);
      }
      moveStuff(dt);

      bool render_kinect_fps;
      GET_SETTING("render_kinect_fps", bool, render_kinect_fps);
      Renderer::g_renderer()->ui()->setTextWindowVisibility("kinect_fps_wnd",
        render_kinect_fps);

      Renderer::g_renderer()->renderFrame();
 
      // Give OS the opportunity to deschedule
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
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_HAND_DETECTOR_DEPTH, "Hand Detector Depth"));
    ui->addCheckbox("use_depth_from_file", "(Debug) Use Depth From File");
    ui->addCheckbox("use_coeff_convnet_from_file", "(Debug) Use coeffs "
      "from file");
    ui->addCheckbox("render_kinect_fps", "Render Kinect FPS");

    ui->addHeadingText("Hand Detection:");
    ui->addCheckbox("detect_hands", "Enable Hand Detection");
    ui->addCheckbox("detect_pose", "Enable Pose Detection");
    ui->addCheckbox("render_hand_labels", "Mark Hand Pixels");

    ui->addSelectbox("label_type_enum", "Hand label type");
    ui->addSelectboxItem("label_type_enum", 
      ui::UIEnumVal(OUTPUT_UNFILTERED_LABELS, "Unfiltered DF"));
    ui->addSelectboxItem("label_type_enum", 
      ui::UIEnumVal(OUTPUT_FILTERED_LABELS, "Filtered DF"));
    ui->addSelectboxItem("label_type_enum", 
      ui::UIEnumVal(OUTPUT_FLOODFILL_LABELS, "Floodfill"));
    ui->addCheckbox("render_convnet_points", 
      "Render Convnet salient points");

    ui->createTextWindow("kinect_fps_wnd", kinect_fps_str_);
    jtil::math::Int2 pos(400, 0);
    ui->setTextWindowPos("kinect_fps_wnd", pos);

    HandModel::loadHandModels(false, true);

    LightSpotCVSM* light_spot_vsm = new LightSpotCVSM(Renderer::g_renderer());
    light_spot_vsm->dir_world().set(0, 0, -1);
    light_spot_vsm->pos_world().set(0, 0, 20);
    light_spot_vsm->near_far().set(1.0f, 2000.0f);
    light_spot_vsm->outer_fov_deg() = 35.0f;
    light_spot_vsm->diffuse_intensity() = 1.0f;
    light_spot_vsm->inner_fov_deg() = 30.0f;
    light_spot_vsm->cvsm_count(1);
    Renderer::g_renderer()->addLight(light_spot_vsm);
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
        Renderer::g_renderer()->ui()->setCheckboxVal("pause_physics",
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
