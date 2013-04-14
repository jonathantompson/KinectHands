#include <stdio.h>  // for printf
#include <thread>
#include <string>
#include <iostream>  // for cout
#if defined(WIN32) || defined(_WIN32) 
  #include <direct.h>
#endif
#include "app/app.h"
#include "jtil/ui/ui.h"
#include "kinect_interface/kinect_interface.h"
#include "kinect_interface/hand_detector/hand_detector.h"
#include "kinect_interface/hand_net/hand_image_generator.h"
#include "kinect_interface/hand_net/hand_model.h"  // for camera parameters
#include "kinect_interface/hand_detector/decision_tree_structs.h"
#include "jtil/glew/glew.h"
#include "jtil/image_util/image_util.h"
#include "jtil/fastlz/fastlz.h"

#ifndef NULL
#define NULL 0
#endif
#define SAFE_DELETE(x) if (x) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x) { delete[] x; x = NULL; }

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
using namespace kinect_interface::hand_net;
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
    disk_im_ = NULL;
    disk_im_compressed_ = NULL;
    rand_norm_ = new NORM_DIST<float>(0.0f, 1.0f);
    rand_uni_ = new UNIF_DIST<float>(-1.0f, 1.0f);
    kinect_frame_number_ = MAX_UINT64;
    convnet_background_tex_ = NULL;
    background_tex_ = NULL;
  }

  App::~App() {
    if (kinect_) {
      kinect_->shutdownKinect();
    }
    SAFE_DELETE(kinect_);
    SAFE_DELETE(clk_);
    SAFE_DELETE(rand_norm_);
    SAFE_DELETE(rand_uni_);
    SAFE_DELETE(convnet_background_tex_);
    SAFE_DELETE(background_tex_);
    SAFE_DELETE_ARR(disk_im_);
    SAFE_DELETE_ARR(disk_im_compressed_);
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

    kinect_ = new KinectInterface(NULL);

    // We have lots of hard-coded dimension values (check that they are
    // consistent).
    Int2 expected_dim(src_width, src_height);
    if (!Int2::equal(kinect_->depth_dim(), expected_dim) ||
      !Int2::equal(kinect_->rgb_dim(), expected_dim)) {
      throw std::wruntime_error("App::newApp() - ERROR: Sensor image "
        "dimensions don't match our hard-coded values!");
    }

    uint16_t dummy_16;
    uint8_t dummy_8;
    static_cast<void>(dummy_8);
    static_cast<void>(dummy_16);
    disk_im_size_ = src_dim * sizeof(dummy_16) + 
      src_dim * 3 * sizeof(dummy_8);
    disk_im_ = new uint8_t[disk_im_size_];
    // Compressed image could be 5% + overhead larger! Just allocate 2x space
    disk_im_compressed_ = new uint8_t[disk_im_size_*2];  
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

        if (kinect_output == OUTPUT_HAND_NORMALS) {
          kinect_->hand_net()->image_generator()->calcNormalImage(normals_xyz_,
            kinect_->xyz(), kinect_->labels());
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

      if (kinect_output == OUTPUT_CONVNET_DEPTH) {
        memset(convnet_depth_, 0, sizeof(convnet_depth_[0]) * src_dim);
        const float* src_depth = (float*)kinect_->hand_net()->hpf_hand_image();
        float min = std::numeric_limits<float>::infinity();
        float max = -std::numeric_limits<float>::infinity();
        for (uint32_t v = 0; v < HN_IM_SIZE; v++) {
          for (uint32_t u = 0; u < HN_IM_SIZE; u++) {
            const float src_val = src_depth[v * HN_IM_SIZE + u];
            convnet_depth_[v * HN_IM_SIZE + u] = src_val;
            min = std::min<float>(min, src_val);
            max = std::max<float>(max, src_val);
          }
        }
        // Rescale convnet depth 0-->1
        const float range = max - min;
        for (uint32_t i = 0; i < HN_IM_SIZE * HN_IM_SIZE; i++) {
          convnet_depth_[i] = (convnet_depth_[i] - min) / range;
        }
      }

      if (update_tex) {
        uint16_t* cur_depth;
        switch (kinect_output) {
        case OUTPUT_RGB:
          memcpy(im_, rgb_, sizeof(im_[0]) * src_dim * 3);
          break;
        case OUTPUT_DEPTH:
        case OUTPUT_HAND_DETECTOR_DEPTH:
          switch (kinect_output) {
            case OUTPUT_DEPTH:
              cur_depth = (uint16_t*)depth_;
              break;
            case OUTPUT_HAND_DETECTOR_DEPTH:
              cur_depth = hand_detector_depth_;
              break;
          }
          for (uint32_t i = 0; i < src_dim; i++) {
            const uint8_t val = (cur_depth[i] * 2) % 255;
            im_[i*3] = val;
            im_[i*3+1] = val;
            im_[i*3+2] = val;
          }
          break;
        case OUTPUT_CONVNET_DEPTH:
          for (uint32_t i = 0; i < HN_IM_SIZE * HN_IM_SIZE; i++) {
            const uint8_t val = (uint8_t)(convnet_depth_[i] * 255.0f);
            convnet_im_[i*3] = val;
            convnet_im_[i*3+1] = val;
            convnet_im_[i*3+2] = val;
          }
          break;
        case OUTPUT_HAND_NORMALS:
          for (uint32_t i = 0; i < src_dim; i++) {
            im_[i*3] = (uint8_t)(((normals_xyz_[i*3] + 1.0f) * 0.5f) * 255.0f);
            im_[i*3+1] = (uint8_t)(((normals_xyz_[i*3+1] + 1.0f) * 0.5f) * 255.0f);
            im_[i*3+2] = (uint8_t)(((normals_xyz_[i*3+2] + 1.0f) * 0.5f) * 255.0f);
          }
          break;
        default:
          throw std::wruntime_error("App::run() - ERROR: output image type"
            "not recognized!");
        }

        if (kinect_output != OUTPUT_CONVNET_DEPTH) {
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
        }

        if (kinect_output == OUTPUT_CONVNET_DEPTH) {
          FlipImage<uint8_t>(convnet_im_flipped_, convnet_im_, 
            HN_IM_SIZE, HN_IM_SIZE, 3);
        } else {
          FlipImage<uint8_t>(im_flipped_, im_, src_width, src_height, 3);
        }

        bool render_convnet_points, detect_pose;
        GET_SETTING("detect_pose", bool, detect_pose);
        GET_SETTING("render_convnet_points", bool, render_convnet_points);
        if (render_convnet_points && detect_pose) {
          if (kinect_output == OUTPUT_CONVNET_DEPTH) {
            kinect_->hand_net()->image_generator()->annotateFeatsToHandImage(
              convnet_im_flipped_, kinect_->hand_net()->coeff_convnet());
          } else {
            kinect_->hand_net()->image_generator()->annotateFeatsToKinectImage(
              im_flipped_, kinect_->hand_net()->coeff_convnet());
          }
        }

        if (kinect_output == OUTPUT_CONVNET_DEPTH) {
          convnet_background_tex_->flagDirty();
          Renderer::g_renderer()->setBackgroundTexture(convnet_background_tex_);
        } else {
          background_tex_->flagDirty();
          Renderer::g_renderer()->setBackgroundTexture(background_tex_);
        }

      }

      // Update camera based on real-time inputs
      if (!Renderer::ui()->mouse_over_ui()) {
        moveCamera(dt);
      }
      moveStuff(dt);

      bool render_kinect_fps, show_hand_model;
      GET_SETTING("render_kinect_fps", bool, render_kinect_fps);
      GET_SETTING("show_hand_model", bool, show_hand_model);
      Renderer::g_renderer()->ui()->setTextWindowVisibility("kinect_fps_wnd",
        render_kinect_fps);
      HandModel::setHandModelVisibility(show_hand_model);
      if (show_hand_model) {
        HandModel::setHandModelPose(HandType::RIGHT, 
          kinect_->hand_net()->image_generator(), 
          kinect_->hand_net()->coeff_convnet());
      }

      Renderer::g_renderer()->renderFrame();

      // Save the frame to file if we have been asked to:
      bool continuous_snapshot;
      GET_SETTING("continuous_snapshot", bool, continuous_snapshot);
      if (continuous_snapshot) {
        std::stringstream ss;
        uint64_t time = (uint64_t)(clk_->getTime() * 1e9);
        ss << "hands_" << time << ".bin";
        saveSensorData(ss.str());
      }

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
    convnet_background_tex_ = new Texture(GL_RGB, HN_IM_SIZE, HN_IM_SIZE, 
      GL_RGB, GL_UNSIGNED_BYTE, (unsigned char*)convnet_im_flipped_, false,
      TextureWrapMode::TEXTURE_CLAMP, TextureFilterMode::TEXTURE_NEAREST, 
      false);

    // Transfer ownership of the texture to the renderer
    Renderer::g_renderer()->setBackgroundTexture(background_tex_);

    ui::UI* ui = Renderer::g_renderer()->ui();
    ui->addHeadingText("Kinect Settings:");
    ui->addSelectbox("kinect_output", "Kinect Output");
    ui->addSelectboxItem("kinect_output", ui::UIEnumVal(OUTPUT_RGB, "RGB"));
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_DEPTH, "Depth"));
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_HAND_DETECTOR_DEPTH, "Hand Detector Depth"));
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_CONVNET_DEPTH, "Convnet Depth"));
    ui->addSelectboxItem("kinect_output",
      ui::UIEnumVal(OUTPUT_HAND_NORMALS, "Hand Normals"));
    ui->addCheckbox("render_kinect_fps", "Render Kinect FPS");
    ui->addCheckbox("crop_depth_to_rgb", "Crop depth to RGB");
    ui->addCheckbox("continuous_snapshot", "Save continuous video stream");

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
    ui->addCheckbox("show_hand_model", "Show hand model");

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

  void App::saveSensorData(const std::string& filename) {
    // Now copy over the depth image and also flag the user pixels (which for
    // now are just pixels less than some threshold):
    uint16_t* depth_dst = (uint16_t*)disk_im_;
    memcpy(depth_dst, depth_, src_dim * sizeof(depth_dst[0]));
    for (int i = 0; i < src_width * src_height; i++) {
      if (depth_[i] < GDT_INNER_DIST) {
        depth_dst[i] |= 0x8000;  // Mark the second MSB
      }
    }
    
    // Now copy over the rgb image
    uint8_t* rgb_dst = (uint8_t*)&depth_dst[src_dim];
    memcpy(rgb_dst, rgb_, 3 * src_dim * sizeof(rgb_dst[0]));

    // Compress the array
    static const int compression_level = 1;  // 1 fast, 2 better compression
    int compressed_length = fastlz_compress_level(compression_level,
      (void*)disk_im_, disk_im_size_, (void*)disk_im_compressed_);

    // Now save the array to file
#ifdef _WIN32
    _mkdir("./data/hand_depth_data/");  // Silently fails if dir exists
    std::string full_filename = "./data/hand_depth_data/" + filename;
#endif
#ifdef __APPLE__
    std::string full_path = file_io::GetHomePath() +
      std::string("Desktop/data/hand_depth_data/");
    struct stat st;
    if (stat(full_path.c_str(), &st) != 0) {
      if (mkdir(full_path.c_str(), S_IRWXU|S_IRWXG) != 0) {
        printf("Error creating directory %s: %s\n", full_path.c_str(),
               strerror(errno));
        return false;
      } else {
        printf("%s created\n", full_path.c_str());
      }
    }
    std::string full_filename = full_path + filename;
#endif
    
    // Save the file compressed
    std::ofstream file(full_filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + filename);
    }
    file.write((const char*)disk_im_compressed_, compressed_length);
    file.flush();
    file.close();
  }

}  // namespace app
