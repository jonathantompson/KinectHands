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
#include "kinect_interface/open_ni_funcs.h"
#include "kinect_interface/hand_detector/hand_detector.h"
#include "kinect_interface/hand_net/hand_image_generator.h"
#include "kinect_interface/hand_net/hand_model_coeff.h"  // for camera parameters
#include "kinect_interface/hand_net/robot_hand_model.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"
#include "jtil/glew/glew.h"
#include "jtil/image_util/image_util.h"
#include "jtil/fastlz/fastlz.h"
#include "jtil/threading/thread_pool.h"
#include "jtil/threading/thread.h"
#include "jtorch/jtorch.h"
#include "jtil/video/video_stream.h"

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
using namespace jtil::threading;

const uint32_t num_pose_smoothing_factors = 10;
const float pose_smoothing_factors[num_pose_smoothing_factors] = {
  0.0f,
  0.1f,
  0.2f,
  0.3f,
  0.4f,
  0.5f,
  0.6f,
  0.7f,
  0.8f,
  0.9f,
};

namespace app {

  App* App::g_app_ = NULL;
  uint32_t App::screenshot_counter_ = 0;
#if defined(_WIN32)
  DebugBuf* App::debug_buf = NULL;
  std::streambuf* old_cout_buf = NULL;
#endif

  App::App() {
    app_running_ = false;
    clk_ = NULL;
    for (uint32_t i = 0; i < MAX_NUM_KINECTS; i++) {
      kinect_[i] = NULL;
      kdata_[i] = NULL;
    }
   
    convnet_background_tex_ = NULL;
    convnet_src_background_tex_ = NULL;
    background_tex_ = NULL;
    convnet_hm_background_tex_ = NULL;
    hand_net_ = NULL;
    convnet_hm_im_flipped_ = NULL;
    kinect_update_cbs_ = NULL;
    data_save_cbs_ = NULL;
    robot_hand_model_ = NULL;
    tp_ = NULL;
    drawing_ = false;
    was_drawing_ = false;
  }

  App::~App() {
    for (uint32_t i = 0; i < MAX_NUM_KINECTS; i++) {
      if (kinect_[i]) {
        kinect_[i]->shutdownKinect(); 
      }
      SAFE_DELETE(kinect_[i]);
      SAFE_DELETE(kdata_[i]);
    }
    if (tp_) {
      tp_->stop();
    }
    SAFE_DELETE(robot_hand_model_);
    SAFE_DELETE(hand_net_);
    SAFE_DELETE(clk_);
    SAFE_DELETE(tp_);
    SAFE_DELETE(convnet_background_tex_);
    SAFE_DELETE(convnet_src_background_tex_);
    SAFE_DELETE(convnet_hm_background_tex_);
    SAFE_DELETE(background_tex_);
    SAFE_DELETE(kinect_update_cbs_); 
    SAFE_DELETE(data_save_cbs_);
    SAFE_DELETE_ARR(convnet_hm_im_flipped_);
    Renderer::ShutdownRenderer();
    jtorch::ShutdownJTorch();
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
    jtorch::InitJTorch("../jtorch");  // Initialize jtorch
    hand_net_ = new HandNet();
    hand_net_->loadFromFile("./data/handmodel.net.convnet");
    hm_size_ = hand_net_->heat_map_size();
    hm_nfeats_ = hand_net_->num_output_features();
    hm_feats_dim_[1] = (int)ceilf(sqrtf((float)hm_nfeats_));  // Round up
    // The following is hm_nfeats_ / hm_feats_dim_[1], rounded up
    // http://stackoverflow.com/questions/2745074/fast-ceiling-of-an-integer-division-in-c-c
    hm_feats_dim_[0] = (hm_nfeats_ + hm_feats_dim_[1] - 1) / hm_feats_dim_[1];
    uint32_t w = hm_feats_dim_[0] * hm_size_;
    uint32_t h = hm_feats_dim_[1] * hm_size_;
    convnet_hm_im_flipped_ = new uint8_t[w * h * 3];
    memset(convnet_hm_im_flipped_, 0, 
      sizeof(convnet_hm_im_flipped_[0]) * w * h * 3);

    Renderer::InitRenderer();
    registerNewRenderer();

    clk_ = new Clk();
    frame_time_ = clk_->getTime();
    app_running_ = true;

    // Initialize the space to store the app local kinect data
    for (uint32_t i = 0; i < MAX_NUM_KINECTS; i++) {
      kdata_[i] = new FrameData();
    }

    // Find and initialize all kinects up to MAX_NUM_KINECTS
    KinectInterface::findDevices(kinect_uris_);
    if (kinect_uris_.size() == 0) {
      throw std::wruntime_error("App::init() - ERROR: Found no OpenNI "
        "compatible devices!");
    }
    for (uint32_t i = 0; i < kinect_uris_.size() && i < MAX_NUM_KINECTS; i++) {
      kinect_[i] = new KinectInterface(kinect_uris_[i]);
    }
    int cur_kinect;
    GET_SETTING("cur_kinect", int, cur_kinect);
    if (cur_kinect >= (int)kinect_uris_.size()) {
      cur_kinect = 0;
      SET_SETTING("cur_kinect", int, cur_kinect);
    }

    // Initialize data handling and multithreading
    tp_ = new ThreadPool(kinect_uris_.size());
    kinect_update_cbs_ = new VectorManaged<Callback<void>*>(kinect_uris_.size());
    data_save_cbs_ = new VectorManaged<Callback<void>*>(kinect_uris_.size());
    for (uint32_t i = 0; i < kinect_uris_.size(); i++) {
      kinect_update_cbs_->pushBack(MakeCallableMany(&App::syncKinectData, this, i));
      data_save_cbs_->pushBack(MakeCallableMany(&App::saveKinectData, this, i));
    }

    // We have lots of hard-coded dimension values (check that they are
    // consistent).
    bool sync_ir_stream;
    GET_SETTING("sync_ir_stream", bool, sync_ir_stream);
    Int2 expected_dim(src_width, src_height);
    if (!Int2::equal(kinect_[0]->depth_dim(), expected_dim) ||
      !Int2::equal(kinect_[0]->rgb_dim(), expected_dim)) {
      throw std::wruntime_error("App::newApp() - ERROR: Sensor image "
        "dimensions don't match our hard-coded values!");
    }

    initRainbowPallet();
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
    Renderer::g_renderer()->registerResetScreenCB(App::resetScreenCB);
    Renderer::g_renderer()->getMousePosition(g_app_->mouse_pos_);
    Renderer::g_renderer()->registerCloseWndCB(App::closeWndCB);

   // Set the camera to the kinect camera parameters
    float fov_vert_deg = 
      360.0f * OpenNIFuncs::fVFOV_primesense_109 / (2.0f * (float)M_PI);
    float view_plane_near = -HAND_MODEL_CAMERA_VIEW_PLANE_NEAR;
    float view_plane_far = -HAND_MODEL_CAMERA_VIEW_PLANE_FAR;
    SET_SETTING("fov_deg", float, fov_vert_deg);
    SET_SETTING("view_plane_near", float, view_plane_near);
    SET_SETTING("view_plane_far", float, view_plane_far);

    g_app_->addStuff();
  }

  void App::initRainbowPallet() {
    unsigned char r, g, b;
    for (int i = 0; i < 256; i++) {
      if (i <= 29) {
        r = (unsigned char)(129.36-i*4.36);
        g = 0;
        b = (unsigned char)255;
      } else if (i<=86) {
        r = 0;
        g = (unsigned char)(-133.54+i*4.52);
        b = (unsigned char)255;
      } else if (i<=141) {
        r = 0;
        g = (unsigned char)255;
        b = (unsigned char)(665.83-i*4.72);
      } else if (i<=199) {
        r = (unsigned char)(-635.26+i*4.47);
        g = (unsigned char)255;
        b = 0;
      } else {
        r = (unsigned char)255;
        g = (unsigned char)(1166.81-i*4.57);
        b = 0;
      }

      rainbowPalletR[i] = r;
      rainbowPalletG[i] = g;
      rainbowPalletB[i] = b;
    }
  }

//#define PROFILE
#ifdef PROFILE
  double hand_pose_time = 0;
  uint32_t hand_pose_nframes = 0;
  double convnet_time = 0;
  uint32_t convnet_nframes = 0;
#endif
  void App::run() {
    while (app_running_) {
      frame_time_prev_ = frame_time_;
      frame_time_ = clk_->getTime();
      double dt = frame_time_ - frame_time_prev_;
      FrameData::time_sec = frame_time_;

      int kinect_output = 0, cur_kinect = 0, label_type_enum = 0;
      GET_SETTING("cur_kinect", int, cur_kinect);
      GET_SETTING("kinect_output", int, kinect_output);
      GET_SETTING("label_type_enum", int, label_type_enum);

      // Copy over the data from all the kinect threads
      new_data_ = false;
      executeThreadCallbacks(tp_, kinect_update_cbs_);

      std::stringstream ss;
      for (uint32_t i = 0; i < kinect_uris_.size(); i++) {
        ss << "K" << i << ": " << kdata_[i]->kinect_fps_str << "fps, ";
      }
      Renderer::g_renderer()->ui()->setTextWindowString("kinect_fps_wnd",
        ss.str().c_str());

      if (new_data_) {
        if (kinect_output == OUTPUT_HAND_NORMALS) {
            hand_net_->image_generator()->calcNormalImage(
              kdata_[cur_kinect]->normals_xyz, kdata_[cur_kinect]->xyz, 
              kdata_[cur_kinect]->labels);
        } 
        int hand_size_enum = 0;
        bool detect_pose = false, detect_heat_map = false;
        GET_SETTING("detect_pose", bool, detect_pose);
        GET_SETTING("detect_heat_map", bool, detect_heat_map);
        GET_SETTING("hand_size", int, hand_size_enum);
        float hand_size = 1.0f - 0.05f * hand_size_enum;
        hand_net_->setHandSize(hand_size);
        if (detect_heat_map) {
#ifdef PROFILE
          double t0 = clk_->getTime();
#endif
          hand_net_->calcConvnetHeatMap(kdata_[cur_kinect]->depth, 
            kdata_[cur_kinect]->labels);
#ifdef PROFILE
          double t1 = clk_->getTime();
          convnet_time += (t1 - t0);
          convnet_nframes++;
          if ( convnet_nframes >= 60 ) {
            std::cout << "Convnet time per frame = " << convnet_time / 
              convnet_nframes << std::endl;
            convnet_time = 0;
            convnet_nframes = 0;
          }
#endif

          if (detect_pose) {
            int smoothing_factor;
            bool smoothing_on;
            GET_SETTING("pose_smoothing_factor_enum", int, smoothing_factor);
            GET_SETTING("pose_smoothing_on", bool, smoothing_on);
#ifdef PROFILE
            double t0 = clk_->getTime();
#endif
            hand_net_->calcConvnetPose(kdata_[cur_kinect]->depth, 
              kdata_[cur_kinect]->labels, 
              smoothing_on ? pose_smoothing_factors[smoothing_factor] : 0);
#ifdef PROFILE
            double t1 = clk_->getTime();
            hand_pose_time += (t1 - t0);
            hand_pose_nframes++;
            if ( hand_pose_nframes >= 60 ) {
              std::cout << "Hand Pose time per frame = " << hand_pose_time / 
                hand_pose_nframes << std::endl;
              hand_pose_time = 0;
              hand_pose_nframes = 0;
            }
#endif
            robot_hand_model_->updateMatrices(hand_net_->rhand_cur_pose()->coeff());
          }
        }
      }  // if (new_data_)

      if (kinect_output == OUTPUT_CONVNET_DEPTH) {
        const float* src_depth = hand_net_->hpf_hand_image();
        //float min = std::numeric_limits<float>::infinity();
        //float max = -std::numeric_limits<float>::infinity();
        //for (uint32_t v = 0; v < HN_IM_SIZE; v++) {
        //  for (uint32_t u = 0; u < HN_IM_SIZE; u++) {
        //    min = std::min<float>(min, src_depth[v * HN_IM_SIZE + u]);
        //    max = std::max<float>(max, src_depth[v * HN_IM_SIZE + u]);
        //  }
        //}
        //const float scale = 2.0f * std::max<float>(fabsf(max), fabsf(min));
        const float scale = 4.0f;
        for (uint32_t i = 0; i < HN_IM_SIZE * HN_IM_SIZE; i++) {
          float val = 0.5f + src_depth[i] / scale;  // Centered around 0.5
          val = std::min<float>(1.0f, std::max<float>(0.0f, val));  // 0 to 1
          depth_tmp_[i] = (uint16_t)(255.0f * val);
        }
      } else if (kinect_output == OUTPUT_CONVNET_SRC_DEPTH) {
        const float* src_depth = hand_net_->image_generator()->cropped_hand_image();
        for (uint32_t i = 0; i < HN_SRC_IM_SIZE * HN_SRC_IM_SIZE; i++) {
          depth_tmp_[i] = (uint16_t)src_depth[i];
        }
      }

      if (new_data_) {
        switch (kinect_output) {
        case OUTPUT_RGB_IR:
          memcpy(im_, kdata_[cur_kinect]->rgb_ir, sizeof(im_[0]) * src_dim * 3);
          break;
        case OUTPUT_RGB_REGISTERED:
          memcpy(im_, kdata_[cur_kinect]->registered_rgb, 
            sizeof(im_[0]) * src_dim * 3);
          break;
        case OUTPUT_BLUE:
          {
            // Set to 0.15f, 0.15f, 0.3f
            for (uint32_t i = 0; i < src_dim; i++) {
              im_[i*3] = 38;
              im_[i*3+1] = 38;
              im_[i*3+2] = 77;
            }
          }
          break;
        case OUTPUT_DEPTH: 
          {
            int16_t* depth = kdata_[cur_kinect]->depth;
            for (uint32_t i = 0; i < src_dim; i++) {
              if (depth[i] < 1250 && depth[i] > 0) {
                const uint8_t val = (depth[i] / 5) % 255;
                im_[i*3] = val;
                im_[i*3+1] = val;
                im_[i*3+2] = val;
              } else {
                im_[i*3] = 25;
                im_[i*3+1] = 25;
                im_[i*3+2] = 100;
              }
            }
          }
          break;
        case OUTPUT_DEPTH_ALL_VIEWS:
          {
            const uint32_t n_tiles_x = 
              (uint32_t)std::ceil(sqrtf((float)MAX_NUM_KINECTS));
            const uint32_t n_tiles_y = MAX_NUM_KINECTS / n_tiles_x;
            const uint32_t tile_res_x = src_width / n_tiles_x;
            const uint32_t tile_res_y = src_height / n_tiles_y;
            uint32_t k = 0;
            for (uint32_t vtile = 0; vtile < n_tiles_y; vtile++) {
              for (uint32_t utile = 0; utile < n_tiles_x; utile++, k++) {
                // Just point sample the kinects (no need to get fancy)
                uint32_t v_dst = vtile * tile_res_y;
                uint32_t v_src = 0;
                for (; v_dst < ((vtile + 1) * tile_res_y) && v_src < src_height;
                  v_dst++, v_src+=n_tiles_y) {
                  uint32_t u_dst = utile * tile_res_x;
                  uint32_t u_src = 0;
                  for (; u_dst < ((utile + 1) * tile_res_x) && u_src < src_width;
                    u_dst++, u_src+=n_tiles_x) {
                    uint32_t i_src = v_src * src_width + u_src;
                    uint32_t i_dst = v_dst * src_width + u_dst;
                    const uint8_t val = ((kdata_[k]->depth[i_src] & 0x7fff) / 5) % 255;
                    im_[i_dst*3] = val;
                    im_[i_dst*3+1] = val;
                    im_[i_dst*3+2] = val;
                  }
                }
              }
            }
          }
          break;
        case OUTPUT_DEPTH_RAINBOW:
          {
            int16_t* depth = kdata_[cur_kinect]->depth;
            for (uint32_t i = 0; i < src_dim; i++) {
              uint32_t nColIndex = (uint32_t)(depth[i] % 256);
              im_[i * 3] = rainbowPalletR[nColIndex];
              im_[i * 3 + 1] = rainbowPalletG[nColIndex];
              im_[i * 3 + 2] = rainbowPalletB[nColIndex];
            }
          }
          break;
        case OUTPUT_HAND_DETECTOR_DEPTH:
          image_util::UpsampleNoFiltering<uint16_t>(depth_tmp_, 
            (uint16_t*)kdata_[cur_kinect]->depth_hand_detector, 
            src_width / DT_DOWNSAMPLE, src_height / DT_DOWNSAMPLE, 
            DT_DOWNSAMPLE);
          for (uint32_t i = 0; i < src_dim; i++) {
            const uint8_t val = (depth_tmp_[i] * 2) % 255;
            im_[i*3] = val;
            im_[i*3+1] = val;
            im_[i*3+2] = val;
          }
          break;
        case OUTPUT_CONVNET_DEPTH:
          {
            bool color_convnet_depth;
            GET_SETTING("color_convnet_depth", bool, color_convnet_depth);
            if (color_convnet_depth) {
              int feature, threshold;
              GET_SETTING("color_convnet_depth_feature", int, feature);
              GET_SETTING("color_convnet_depth_threshold", int, threshold);
              const float* cur_hm = &hand_net_->heat_map_convnet()[feature * 
                (hm_size_ * hm_size_)];

              float hm_max = (float)threshold;
              float hm_min = 0.0f;
              float hm_range = hm_max - hm_min;
              const uint32_t upsample_factor = (uint32_t)HN_IM_SIZE / hm_size_;
              for (uint32_t v = 0; v < HN_IM_SIZE; v++) {
                for (uint32_t u = 0; u < HN_IM_SIZE; u++) {
                  uint32_t isrc = v * HN_IM_SIZE + u;
                  uint32_t isrc_hm = (v / upsample_factor) * hm_size_ + u / upsample_factor;
                  float val = (float)depth_tmp_[isrc]/255.0f;  // 0 to 1
                  float hm_val = (std::max<float>(cur_hm[isrc_hm], 0) - hm_min) / hm_range;  // 0 to 1
                  const uint32_t idst = (HN_IM_SIZE-v-1)*HN_IM_SIZE + u;
                  convnet_im_flipped_[idst * 3] = (uint8_t)(std::max<float>(0, 
                    std::min<float>(255.0f * (val + 0.25f * hm_val), 255.0f)));
                  convnet_im_flipped_[idst * 3 + 1] = (uint8_t)(std::max<float>(0, 
                    std::min<float>(255.0f * val * (1 - hm_val), 255.0f)));
                  convnet_im_flipped_[idst * 3 + 2] = (uint8_t)(std::max<float>(0, 
                    std::min<float>(255.0f * val * (1 - hm_val), 255.0f)));
                }
              }
            } else {
              uint32_t isrc = 0;
              for (uint32_t v = 0; v < HN_IM_SIZE; v++) {
                for (uint32_t u = 0; u < HN_IM_SIZE; u++, isrc++) {
                  const uint8_t val = (uint8_t)(depth_tmp_[isrc]);
                  const uint32_t idst = (HN_IM_SIZE-v-1)*HN_IM_SIZE + u;
                  convnet_im_flipped_[idst * 3] = val;
                  convnet_im_flipped_[idst * 3 + 1] = val;
                  convnet_im_flipped_[idst * 3 + 2] = val;
                }
              }
            }
          }
          break;
        case OUTPUT_CONVNET_SRC_DEPTH:
          {
            uint32_t isrc = 0;
            for (uint32_t v = 0; v < HN_SRC_IM_SIZE; v++) {
              for (uint32_t u = 0; u < HN_SRC_IM_SIZE; u++, isrc++) {
                const uint8_t val = (uint8_t)(depth_tmp_[isrc] * 255.0f);
                const uint32_t idst = (HN_SRC_IM_SIZE-v-1)*HN_SRC_IM_SIZE + u;
                convnet_src_im_flipped_[idst * 3] = val;
                convnet_src_im_flipped_[idst * 3 + 1] = val;
                convnet_src_im_flipped_[idst * 3 + 2] = val;
              }
            }
          }
          break;
        case OUTPUT_HAND_NORMALS:
          for (uint32_t i = 0; i < src_dim; i++) {
            im_[i*3] = (uint8_t)(std::max<float>(0, 
              kdata_[cur_kinect]->normals_xyz[i*3]) * 255.0f);
            im_[i*3+1] = (uint8_t)(std::max<float>(0, 
              kdata_[cur_kinect]->normals_xyz[i*3+1]) * 255.0f);
            im_[i*3+2] = (uint8_t)(std::max<float>(0, 
              -kdata_[cur_kinect]->normals_xyz[i*3+2]) * 255.0f);
          }
          break;
        case OUTPUT_CONVNET_HEAT_MAPS:
          {
            const float* hm = hand_net_->heat_map_convnet();
            for (uint32_t f = 0; f < hm_nfeats_; f++) {
              const float* cur_hm = &hm[f * (hm_size_ * hm_size_)];
              uint32_t tileu = f % hm_feats_dim_[1];
              uint32_t tilev = f / hm_feats_dim_[1];
              float hm_min = std::numeric_limits<float>::infinity();
              float hm_max = -std::numeric_limits<float>::infinity();
              for (uint32_t i = 0; i < hm_size_ * hm_size_; i++) {
                hm_min = std::min<float>(hm_min, cur_hm[i]);
                hm_max = std::max<float>(hm_max, cur_hm[i]);
              }
              float hm_range = hm_max - hm_min;
              for (uint32_t v = 0; v < hm_size_; v++) {
                for (uint32_t u = 0; u < hm_size_; u++) {
                  const uint8_t src_val = (uint8_t)(255.0f * ((cur_hm[v * hm_size_ + u] - hm_min) / hm_range));
                  const uint32_t vdst = tilev * hm_size_ + v;
                  const uint32_t vdst_flipped = hm_feats_dim_[1] * hm_size_ - vdst - 1;
                  const uint32_t idst = vdst_flipped * hm_size_ * hm_feats_dim_[0] + (tileu * hm_size_ + u);
                  convnet_hm_im_flipped_[idst * 3] = src_val;
                  convnet_hm_im_flipped_[idst * 3 + 1] = src_val;
                  convnet_hm_im_flipped_[idst * 3 + 2] = src_val;
                }
              }
            }
          }
          break;
        default:
          throw std::wruntime_error("App::run() - ERROR: output image type"
            "not recognized!");
        }

        if (kinect_output != OUTPUT_CONVNET_DEPTH && 
          kinect_output != OUTPUT_CONVNET_SRC_DEPTH) {
          if (label_type_enum != OUTPUT_NO_LABELS) {
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

        if (kinect_output != OUTPUT_CONVNET_SRC_DEPTH && 
          kinect_output != OUTPUT_CONVNET_HEAT_MAPS && 
          kinect_output != OUTPUT_CONVNET_DEPTH) {
          FlipImage<uint8_t>(im_flipped_, im_, src_width, src_height, 3);
        }

        switch (kinect_output) {
        case OUTPUT_CONVNET_DEPTH:
          convnet_background_tex_->flagDirty();
          Renderer::g_renderer()->setBackgroundTexture(convnet_background_tex_);
          break;
        case OUTPUT_CONVNET_SRC_DEPTH:
          convnet_src_background_tex_->flagDirty();
          Renderer::g_renderer()->setBackgroundTexture(convnet_src_background_tex_);
          break;
        case OUTPUT_CONVNET_HEAT_MAPS:
          convnet_hm_background_tex_->flagDirty();
          Renderer::g_renderer()->setBackgroundTexture(convnet_hm_background_tex_);
          break;
        default:
          bool show_gaussian_labels, detect_heat_map;
          GET_SETTING("show_gaussian_labels", bool, show_gaussian_labels);
          GET_SETTING("detect_heat_map", bool, detect_heat_map);
          if (detect_heat_map && show_gaussian_labels) {
            const float* gauss_coeff = hand_net_->gauss_coeff();
            const uint32_t num_feats = hand_net_->num_output_features();
            for (uint32_t i = 0; i < num_feats; i++) {
              const float* cur_dist = &gauss_coeff[i * NUM_COEFFS_PER_GAUSSIAN];
              const int32_t u_cen = (int32_t)floorf(cur_dist[GaussMeanU]);
              const int32_t v_cen = (int32_t)floorf(cur_dist[GaussMeanV]);
              const int32_t u_std = (int32_t)floorf(sqrt(cur_dist[GaussVarU]));
              const int32_t v_std = (int32_t)floorf(sqrt(cur_dist[GaussVarV]));
              for (int32_t v = v_cen - 3*v_std; v <= v_cen + 3*v_std; v++) {
                for (int32_t u = u_cen - 3*u_std; u <= u_cen + 3*u_std; u++) {
                  if (u >= 0 && u < src_width && v >= 0 && v < src_height) {
                    float du = (float)u - cur_dist[GaussMeanU];
                    float dv = (float)v - cur_dist[GaussMeanV];
                    float prob = exp(-(du*du/(2*cur_dist[GaussVarU]) + dv*dv/(2*cur_dist[GaussVarV])));
                    int32_t index = (src_height - v - 1) * src_width + u;
                    if (index >= 0 && index < src_width * src_height) {
                      //im_flipped_[index * 3] *= prob;
                      im_flipped_[index * 3 + 1] = (uint8_t)((float)im_flipped_[index * 3 + 1] * (1.0f - prob));
                      im_flipped_[index * 3 + 2] = (uint8_t)((float)im_flipped_[index * 3 + 2] * (1.0f - prob));
                    }
                  }
                }
              }
            }
          }

          background_tex_->flagDirty();
          Renderer::g_renderer()->setBackgroundTexture(background_tex_);
          break;
        }
      }  // if (new_data_)

      // Update camera based on real-time inputs
      if (!Renderer::ui()->mouse_over_ui()) {
        moveCamera(dt);
      }
      moveStuff(dt);

      bool render_kinect_fps;
      int show_hand_model_type;
      GET_SETTING("render_kinect_fps", bool, render_kinect_fps);
      GET_SETTING("show_hand_model_type", int, show_hand_model_type);
      Renderer::g_renderer()->ui()->setTextWindowVisibility("kinect_fps_wnd",
        render_kinect_fps);
      switch (show_hand_model_type) {
      case HAND_TYPE_NONE:
        hand_net_->setModelVisibility(false);
        robot_hand_model_->setRenderVisiblity(false);
        break;
      case HAND_TYPE_LIBHAND:
        hand_net_->setModelVisibility(true);
        robot_hand_model_->setRenderVisiblity(false);
        break;
      case HAND_TYPE_ROBOT:
        hand_net_->setModelVisibility(false);
        robot_hand_model_->setRenderVisiblity(true);
        break;
      default:
        throw std::wruntime_error("INTERNAL ERROR: Undefined model enum");
      }

      Renderer::g_renderer()->renderFrame();

      // Save the frame to file if we have been asked to:
      bool continuous_snapshot, continuous_cal_snapshot;
      GET_SETTING("continuous_snapshot", bool, continuous_snapshot);
      GET_SETTING("continuous_cal_snapshot", bool, continuous_cal_snapshot);
      if (continuous_cal_snapshot && continuous_snapshot) {
        SET_SETTING("continuous_cal_snapshot", bool, false);
      }
      if (continuous_snapshot || continuous_cal_snapshot) {
        executeThreadCallbacks(tp_, data_save_cbs_);
      }

      // Give OS the opportunity to deschedule
      std::this_thread::yield();
    }
  }

  void App::moveCamera(double dt) {

  }

  void App::executeThreadCallbacks(jtil::threading::ThreadPool* tp, 
    jtil::data_str::VectorManaged<jtil::threading::Callback<void>*>* cbs) {
    threads_finished_ = 0;
    for (uint32_t i = 0; i < cbs->size(); i++) {
      tp_->addTask((*cbs)[i]);
    }
    // Wait for all threads to finish
    std::unique_lock<std::mutex> ul(thread_update_lock_);  // Get lock
    while (threads_finished_ != static_cast<int32_t>(cbs->size())) {
      not_finished_.wait(ul);
    }
    ul.unlock();  // Release lock
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
    convnet_src_background_tex_ = new Texture(GL_RGB, HN_SRC_IM_SIZE, 
      HN_SRC_IM_SIZE, GL_RGB, GL_UNSIGNED_BYTE, 
      (unsigned char*)convnet_src_im_flipped_, false, 
      TextureWrapMode::TEXTURE_CLAMP, TextureFilterMode::TEXTURE_NEAREST, 
      false);
    convnet_hm_background_tex_ = new Texture(GL_RGB, hm_size_ * hm_feats_dim_[0],
      hm_size_ * hm_feats_dim_[1], GL_RGB, GL_UNSIGNED_BYTE, 
      (unsigned char*)convnet_hm_im_flipped_, false, 
      TextureWrapMode::TEXTURE_CLAMP, TextureFilterMode::TEXTURE_NEAREST, 
      false);

    // Transfer ownership of the texture to the renderer
    Renderer::g_renderer()->setBackgroundTexture(background_tex_);

    ui::UI* ui = Renderer::g_renderer()->ui();
    ui->addHeadingText("Kinect Settings:");
    ui->addSelectbox("kinect_output", "Kinect Output");
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_RGB_IR, "RGB / IR"));
    ui->addSelectboxItem("kinect_output",
      ui::UIEnumVal(OUTPUT_RGB_REGISTERED, "RGB Registered"));
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_DEPTH, "Depth"));
    ui->addSelectboxItem("kinect_output",
      ui::UIEnumVal(OUTPUT_DEPTH_ALL_VIEWS, "Depth (all views)"));
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_DEPTH_RAINBOW, "Depth Rainbow"));
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_HAND_DETECTOR_DEPTH, "Hand Detector Depth"));
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_CONVNET_DEPTH, "Convnet Depth"));
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_CONVNET_SRC_DEPTH, "Convnet Source Depth"));
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_CONVNET_HEAT_MAPS, "Convnet Heat Map"));
    ui->addSelectboxItem("kinect_output",
      ui::UIEnumVal(OUTPUT_HAND_NORMALS, "Hand Normals"));
    ui->addSelectboxItem("kinect_output",
      ui::UIEnumVal(OUTPUT_BLUE, "Blue Background"));
    ui->addCheckbox("render_kinect_fps", "Render Kinect FPS");
    ui->addCheckbox("continuous_snapshot", "Save continuous video stream");
    ui->addCheckbox("continuous_cal_snapshot", "Save continuous calibration stream");
    ui->addCheckbox("color_convnet_depth", "Color Convnet Depth");
    ui->addSelectbox("color_convnet_depth_feature", "Current Feature");
    std::stringstream ss;
    for (uint32_t i = 0; i < hm_nfeats_; i++) {
      ss.str("");
      ss << "Feature " << i;
      ui->addSelectboxItem("color_convnet_depth_feature", 
        ui::UIEnumVal(i, ss.str().c_str()));
    }
    ui->addSelectbox("color_convnet_depth_threshold", "Feature Threshold");
    for (uint32_t i = 5; i <= 40; i+= 5) {
      ss.str("");
      ss << "Threshold " << i;
      ui->addSelectboxItem("color_convnet_depth_threshold", 
        ui::UIEnumVal(i, ss.str().c_str()));
    }

    ui->addButton("screenshot_button", "RGB Screenshot", 
      App::screenshotCB);
    ui->addButton("greyscale_screenshot_button", "Greyscale Screenshot", 
      App::greyscaleScreenshotCB);

    ui->addHeadingText("Hand Detection:");
    ui->addCheckbox("detect_hands", "Enable Hand Detection");
    ui->addCheckbox("detect_heat_map", "Enable Heat Map Detection");
    ui->addCheckbox("detect_pose", "Enable Pose Detection");
    ui->addCheckbox("pose_smoothing_on", "Enable Pose Smoothing");
    ui->addSelectbox("pose_smoothing_factor_enum", "Smoothing Factor");
    for (uint32_t i = 0; i < num_pose_smoothing_factors; i++) {
      ss.str("");
      ss << pose_smoothing_factors[i];
      ui->addSelectboxItem("pose_smoothing_factor_enum", 
        ui::UIEnumVal(i, ss.str().c_str()));
    }
    ui->addCheckbox("show_gaussian_labels", "Show Gaussian Labels");
    ui->addButton("reset_tracking_button", "Reset Tracking", 
      App::resetTrackingCB);
    ui->addCheckbox("in_air_drawing_enabled", "In Air Drawing");
    ui->addSelectbox("label_type_enum", "Hand label type");
    ui->addSelectboxItem("label_type_enum", 
      ui::UIEnumVal(OUTPUT_NO_LABELS, "Labels off"));
    ui->addSelectboxItem("label_type_enum", 
      ui::UIEnumVal(OUTPUT_UNFILTERED_LABELS, "Unfiltered DF"));
    ui->addSelectboxItem("label_type_enum", 
      ui::UIEnumVal(OUTPUT_FILTERED_LABELS, "Filtered DF"));
    ui->addSelectboxItem("label_type_enum", 
      ui::UIEnumVal(OUTPUT_FLOODFILL_LABELS, "Floodfill"));
    ui->addSelectbox("show_hand_model_type", "Show hand model");
    ui->addSelectboxItem("show_hand_model_type", 
      ui::UIEnumVal(HAND_TYPE_NONE, "None"));
    ui->addSelectboxItem("show_hand_model_type", 
      ui::UIEnumVal(HAND_TYPE_LIBHAND, "LibHand"));
    ui->addSelectboxItem("show_hand_model_type", 
      ui::UIEnumVal(HAND_TYPE_ROBOT, "Robot"));

    ui->addSelectbox("cur_kinect", "Current Kinect");
    for (uint32_t i = 0; i < MAX_NUM_KINECTS; i++) {
      ss.str("");
      ss << "device " << i;
      ui->addSelectboxItem("cur_kinect", ui::UIEnumVal(i, ss.str().c_str()));
    }

    ui->addSelectbox("hand_size", "Hand Size");
    for (uint32_t i = 0; i < 8; i++) {
      ss.str("");
      float val = 1.0f - (0.05f * i); 
      ss << (val * 100) << "%";
      ui->addSelectboxItem("hand_size", ui::UIEnumVal(i, ss.str().c_str()));
    }

    ui->createTextWindow("kinect_fps_wnd", " ");
    jtil::math::Int2 pos(400, 0);
    ui->setTextWindowPos("kinect_fps_wnd", pos);

    LightSpotCVSM* light_spot_vsm = new LightSpotCVSM(Renderer::g_renderer());
    light_spot_vsm->pos_world().set(-200, 0, 200);
    Float3 target(0, 0, 1000);
    Float3 dir;
    Float3::sub(dir, target, light_spot_vsm->pos_world());
    dir.normalize();
    light_spot_vsm->dir_world().set(dir);
    light_spot_vsm->near_far().set(1.0f, 2000.0f);
    light_spot_vsm->outer_fov_deg() = 40.0f;
    light_spot_vsm->diffuse_intensity() = 1.0f;
    light_spot_vsm->inner_fov_deg() = 35.0f;
    light_spot_vsm->cvsm_count(1);
    Renderer::g_renderer()->addLight(light_spot_vsm);

    hand_net_->loadHandModels();
    robot_hand_model_ = new RobotHandModel(HandType::RIGHT);
  }

  void App::resetTrackingCB() {
    g_app_->hand_net_->resetTracking();
  }

  void App::screenshotCB() {
    bool sync_ir_stream;
    int cur_kinect = 0;
    GET_SETTING("sync_ir_stream", bool, sync_ir_stream);
    GET_SETTING("cur_kinect", int, cur_kinect);
    g_app_->kinect_[cur_kinect]->lockData();
    const uint8_t* rgb_src;
    if (!sync_ir_stream) {
      rgb_src = g_app_->kinect_[cur_kinect]->rgb();
    } else {
      rgb_src = g_app_->kinect_[cur_kinect]->ir();
    }
    std::stringstream ss;
    ss << "rgb_screenshot" << g_app_->screenshot_counter_ << ".jpg";
    jtil::renderer::Texture::saveRGBToFile(ss.str(), rgb_src, src_width, 
      src_height, true);
    std::cout << "RGB saved to file " << ss.str() << std::endl;
    const uint16_t* depth =  g_app_->kinect_[cur_kinect]->depth();
    ss.str("");
    ss << "depth_screenshot" << g_app_->screenshot_counter_ << ".jpg";
    jtil::file_io::SaveArrayToFile<uint16_t>(depth, src_dim, ss.str());
    std::cout << "Depth saved to file " << ss.str() << std::endl;
    g_app_->screenshot_counter_++;
    g_app_->kinect_[cur_kinect]->unlockData();
  }

  void App::greyscaleScreenshotCB() {
    int cur_kinect = true;
    bool sync_ir_stream = false;
    GET_SETTING("sync_ir_stream", bool, sync_ir_stream);
    GET_SETTING("cur_kinect", int, cur_kinect);
    g_app_->kinect_[cur_kinect]->lockData();
    const uint8_t* rgb_src;
    if (!sync_ir_stream) {
      rgb_src = g_app_->kinect_[cur_kinect]->rgb();
    } else {
      rgb_src = g_app_->kinect_[cur_kinect]->ir();
    }
    uint8_t* grey = new uint8_t[src_width * src_height];
    for (uint32_t i = 0; i < src_dim; i++) {
      grey[i] = (uint8_t)(((uint16_t)rgb_src[i*3] + (uint16_t)rgb_src[i*3+1] + 
        (uint16_t)rgb_src[i*3+2]) / 3);
    }
    std::stringstream ss;
    ss << "greyscale_screenshot" << g_app_->screenshot_counter_ << ".jpg";
    jtil::renderer::Texture::saveGreyscaleToFile(ss.str(), grey, src_width, 
      src_height, true);
    std::cout << "Greyscale saved to file " << ss.str() << std::endl;
    const uint16_t* depth =  g_app_->kinect_[cur_kinect]->depth();
    ss.str("");
    ss << "depth_screenshot" << g_app_->screenshot_counter_ << ".jpg";
    jtil::file_io::SaveArrayToFile<uint16_t>(depth, src_dim, ss.str());
    std::cout << "Depth saved to file " << ss.str() << std::endl;
    g_app_->screenshot_counter_++;
    g_app_->kinect_[cur_kinect]->unlockData();
    SAFE_DELETE(grey);
  }

  int App::closeWndCB() {
    g_app_->app_running_ = false;
    return 0;
  }

  void App::resetScreenCB() {
    g_app_->registerNewRenderer();
  }

  void App::keyboardCB(int key, int scancode, int action, int mods) {
    switch (key) {
    case KEY_ESC:
      if (action == PRESSED) {
        requestShutdown();
      }
      break;
    default:
      break;
    }
  }
  
  void App::mousePosCB(double x, double y) {

  }
  
  void App::mouseButtonCB(int button, int action, int mods) {
 
  }
  
  void App::mouseWheelCB(double xoffset, double yoffset) {

  }

  void App::syncKinectData(const uint32_t index){
    int cur_kinect = 0, kinect_output = 0;
    GET_SETTING("cur_kinect", int, cur_kinect);
    GET_SETTING("kinect_output", int, kinect_output);
    if (index == static_cast<uint32_t>(cur_kinect)) {
      new_data_ |= kdata_[index]->syncWithKinect(kinect_[index], 
        kinect_output == OUTPUT_HAND_DETECTOR_DEPTH, render_labels_);
    } else {
      kdata_[index]->syncWithKinect(kinect_[index],
        kinect_output == OUTPUT_HAND_DETECTOR_DEPTH);
    }

    // Signal that we're done
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

  void App::saveKinectData(const uint32_t index) {
    bool continuous_cal_snapshot;
    GET_SETTING("continuous_cal_snapshot", bool, continuous_cal_snapshot);

    kdata_[index]->saveSensorData(continuous_cal_snapshot, index);

    // Signal that we're done
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

}  // namespace app
