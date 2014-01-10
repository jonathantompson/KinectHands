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
using namespace jtil::renderer;
using namespace jtil::image_util;
using namespace jtil::threading;

namespace app {

  App* App::g_app_ = NULL;
  uint64_t App::screenshot_counter_ = 0;
#if defined(_WIN32)
  DebugBuf* App::debug_buf = NULL;
  std::streambuf* old_cout_buf = NULL;
#endif

  App::App() {
    app_running_ = false;
    clk_ = NULL;
    for (uint32_t i = 0; i < MAX_NUM_KINECTS; i++) {
      kinect_[i] = NULL;
    }
    depth_tex_ = NULL;
    tp_ = NULL;
    threads_finished_ = false;
    data_save_cbs_ = NULL;
    depth_frame_number_ = 0;
    num_kinects_ = 0;
  }

  App::~App() {
    for (uint32_t i = 0; i < MAX_NUM_KINECTS; i++) {
      if (kinect_[i]) {
        kinect_[i]->shutdownKinect(); 
      }
      SAFE_DELETE(kinect_[i]);
    }
    SAFE_DELETE(clk_);
    SAFE_DELETE(depth_tex_);
    SAFE_DELETE(data_save_cbs_);
    tp_->stop();
    SAFE_DELETE(tp_);
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
    Renderer::InitRenderer();
    registerNewRenderer();

    clk_ = new Clk();
    frame_time_ = clk_->getTime();
    app_running_ = true;

    // Find and initialize all kinects up to MAX_NUM_KINECTS
    jtil::data_str::Vector<std::string> ids;
    KinectInterface::getDeviceIDs(ids);
    num_kinects_ = ids.size();
    if (num_kinects_ == 0) {
      throw std::wruntime_error("App::init() - ERROR: Found no OpenNI "
        "compatible devices!");
    }
    for (uint32_t i = 0; i < num_kinects_ && i < MAX_NUM_KINECTS; i++) {
      kinect_[i] = new KinectInterface(ids[i]);
    }
    int cur_kinect;
    GET_SETTING("cur_kinect", int, cur_kinect);
    if (cur_kinect >= (int)num_kinects_) {
      cur_kinect = num_kinects_ - 1;
      SET_SETTING("cur_kinect", int, cur_kinect);
    }

    tp_ = new ThreadPool(NUM_APP_WORKER_THREADS);
    data_save_cbs_ = new VectorManaged<Callback<void>*>(num_kinects_);
    for (uint32_t i = 0; i < num_kinects_; i++) {
      data_save_cbs_->pushBack(MakeCallableMany(&App::saveKinectData, this, i));
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
    // TODO: Get the REAL FOV...
    float fov_vert_deg = 80;
    float view_plane_near = -1;
    float view_plane_far = -5000;
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

  void App::run() {
    while (app_running_) {
      frame_time_prev_ = frame_time_;
      frame_time_ = clk_->getTime();
      double dt = frame_time_ - frame_time_prev_;

      int kinect_output = 0, cur_kinect = 0, label_type_enum = 0;
      GET_SETTING("cur_kinect", int, cur_kinect);
      GET_SETTING("kinect_output", int, kinect_output);

      bool new_data = false;
      if (kinect_[cur_kinect]->depth_frame_number() > depth_frame_number_) {
        new_data = true;
      }

      // Create the image data (in RGB)
      if (new_data) {
        kinect_[cur_kinect]->lockData();
        switch (kinect_output) {
        case OUTPUT_RGB:
          // TODO: Set this properly
        case OUTPUT_RGB_REGISTERED:
          // TODO: Set this properly
        case OUTPUT_BLUE:
          {
            // Set to 0.15f, 0.15f, 0.3f
            for (uint32_t i = 0; i < depth_dim; i++) {
              depth_im_[i*3] = 38;
              depth_im_[i*3+1] = 38;
              depth_im_[i*3+2] = 77;
            }
          }
          break;
        case OUTPUT_DEPTH: 
          {
            const uint16_t* depth = kinect_[cur_kinect]->depth();
            // TODO: Set this properly
            for (uint32_t i = 0; i < depth_dim; i++) {
              const uint8_t val = (depth[i] / 2) % 255;
              depth_im_[i*3] = val;
              depth_im_[i*3+1] = val;
              depth_im_[i*3+2] = val;
            }
          }
          break;
        case OUTPUT_DEPTH_ALL_VIEWS:
          {
            const uint32_t n_tiles_x = 
              (uint32_t)std::ceil(sqrtf((float)MAX_NUM_KINECTS));
            const uint32_t n_tiles_y = MAX_NUM_KINECTS / n_tiles_x;
            const uint32_t tile_res_x = depth_w / n_tiles_x;
            const uint32_t tile_res_y = depth_h / n_tiles_y;
            uint32_t k = 0;
            for (uint32_t vtile = 0; vtile < n_tiles_y; vtile++) {
              for (uint32_t utile = 0; utile < n_tiles_x; utile++, k++) {
                // Just point sample the kinects (no need to get fancy)
                uint32_t v_dst = vtile * tile_res_y;
                uint32_t v_src = 0;
                for (; v_dst < ((vtile + 1) * tile_res_y) && 
                  v_src < depth_h; v_dst++, v_src+=n_tiles_y) {
                  uint32_t u_dst = utile * tile_res_x;
                  uint32_t u_src = 0;
                  for (; u_dst < ((utile + 1) * tile_res_x) && 
                    u_src < depth_w; u_dst++, u_src+=n_tiles_x) {
                    uint32_t i_src = v_src * depth_w + u_src;
                    uint32_t i_dst = v_dst * depth_w + u_dst;
                    // TODO: Set this properly
                    const uint8_t val = 255;
                    depth_im_[i_dst*3] = val;
                    depth_im_[i_dst*3+1] = val;
                    depth_im_[i_dst*3+2] = val;
                  }
                }
              }
            }
          }
          break;
        case OUTPUT_DEPTH_RAINBOW:
          {
            const uint16_t* depth = kinect_[cur_kinect]->depth();
            for (uint32_t i = 0; i < depth_dim; i++) {
              uint32_t nColIndex = (uint32_t)(depth[i] % 256);
              depth_im_[i*3] = rainbowPalletR[nColIndex];
              depth_im_[i*3+1] = rainbowPalletG[nColIndex];
              depth_im_[i*3+2] = rainbowPalletB[nColIndex];
            }
          }
          break;
        }  // switch (kinect_output)

        // Update the frame number and timestamp
        depth_frame_number_ = kinect_[cur_kinect]->depth_frame_number();
        depth_frame_time_ = kinect_[cur_kinect]->depth_frame_time();
        kinect_[cur_kinect]->unlockData();
        
        // Update the kinect FPS string
        std::stringstream ss;
        for (uint32_t i = 0; i < num_kinects_; i++) {
          ss << "K" << i << ": " << kinect_[i]->kinect_fps_str() << "fps, ";
        }
        Renderer::g_renderer()->ui()->setTextWindowString("kinect_fps_wnd",
          ss.str().c_str());

        // Sync the data with the correct texture and set the correct
        // background texture
        bool stretch_image;
        GET_SETTING("stretch_image", bool, stretch_image);
        Renderer::g_renderer()->setBackgroundTextureStrech(stretch_image);
        switch (kinect_output) {
        case OUTPUT_RGB:
        case OUTPUT_RGB_REGISTERED:
          // TODO: Set these properly
        case OUTPUT_DEPTH:
        case OUTPUT_DEPTH_ALL_VIEWS:
        case OUTPUT_DEPTH_RAINBOW:
        case OUTPUT_BLUE:
          jtil::image_util::FlipImageVertInPlace<uint8_t>(depth_im_,
            depth_w, depth_h, 3);
          depth_tex_->flagDirty();
          Renderer::g_renderer()->setBackgroundTexture(depth_tex_);
          break;
        }
      }  // if (new_data)

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

      // Save the frame to file if we have been asked to:
      bool continuous_snapshot;
      GET_SETTING("continuous_snapshot", bool, continuous_snapshot);
      if (continuous_snapshot) {
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
    std::stringstream ss;

    memset(depth_im_, 0, sizeof(depth_im_[0]) * depth_dim * 3);
    depth_tex_ = new Texture(GL_RGB, depth_w, depth_h, GL_RGB, 
      GL_UNSIGNED_BYTE, (unsigned char*)depth_im_, false,
      TextureWrapMode::TEXTURE_CLAMP, TextureFilterMode::TEXTURE_LINEAR, 
      false);
    
    // Transfer ownership of the texture to the renderer
    Renderer::g_renderer()->setBackgroundTexture(depth_tex_);

    ui::UI* ui = Renderer::g_renderer()->ui();
    ui->addHeadingText("Kinect Settings:");

    // kinect_output selections
    ui->addSelectbox("kinect_output", "Kinect Output");
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_RGB, "RGB / IR"));
    ui->addSelectboxItem("kinect_output",
      ui::UIEnumVal(OUTPUT_RGB_REGISTERED, "RGB Registered"));
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_DEPTH, "Depth"));
    ui->addSelectboxItem("kinect_output",
      ui::UIEnumVal(OUTPUT_DEPTH_ALL_VIEWS, "Depth (all views)"));
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_DEPTH_RAINBOW, "Depth Rainbow"));
    ui->addSelectboxItem("kinect_output",
      ui::UIEnumVal(OUTPUT_BLUE, "Blue Background"));

    ui->addCheckbox("render_kinect_fps", "Render Kinect FPS");
    ui->addCheckbox("continuous_snapshot", "Save continuous video stream");

    ui->addButton("screenshot_button", "RGB Screenshot", 
      App::screenshotCB);

    ui->addSelectbox("cur_kinect", "Current Kinect");
    for (uint32_t i = 0; i < MAX_NUM_KINECTS; i++) {
      ss.str("");
      ss << "device " << i;
      ui->addSelectboxItem("cur_kinect", ui::UIEnumVal(i, ss.str().c_str()));
    }

    ui->addCheckbox("stretch_image", "Stretch Image");

    ui->createTextWindow("kinect_fps_wnd", " ");
    jtil::math::Int2 pos(400, 0);
    ui->setTextWindowPos("kinect_fps_wnd", pos);

    /*
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
    */
  }

  void App::screenshotCB() {
    // TODO: Finish this
    /*
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
    */
    std::cout << "Screenshot not yet added back." << std::endl;
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

  void App::saveKinectData(const uint32_t index) {
    // TODO: Fix this
    // kdata_[index]->saveSensorData(continuous_cal_snapshot, index);
    std::cout << "Save data not yet added back." << std::endl;

    // Signal that we're done
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

}  // namespace app
