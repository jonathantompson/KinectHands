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
    rgb_tex_ = NULL;
    tp_ = NULL;
    threads_finished_ = false;
    data_save_cbs_ = NULL;
    depth_frame_number_ = 0;
    num_kinects_ = 0;
    for (uint32_t i = 0; i < MAX_NUM_KINECTS; i++) {
      kinect_last_saved_depth_time_[i] = 0;
    }
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
    SAFE_DELETE(rgb_tex_);
    SAFE_DELETE(data_save_cbs_);
    if (tp_) {
      tp_->stop();
    }
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
    time_since_start_ = 0.0;
    app_running_ = true;

    // Find and initialize all kinects up to MAX_NUM_KINECTS
    jtil::data_str::VectorManaged<const char*> ids;
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

    bool pause_stream = false;
    SET_SETTING("pause_stream", bool, pause_stream);
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

    // Set the camera to the camera parameters
    float view_plane_near = -1;
    float view_plane_far = -5000;
    SET_SETTING("view_plane_near", float, view_plane_near);
    SET_SETTING("view_plane_far", float, view_plane_far);
    SET_SETTING("fov_deg", float, depth_vfov);

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
      time_since_start_ += dt;

      int kinect_output = 0, cur_kinect = 0, label_type_enum = 0;
      int background_color = 0;
      bool pause_stream, render_point_cloud;
      GET_SETTING("cur_kinect", int, cur_kinect);
      GET_SETTING("kinect_output", int, kinect_output);
      GET_SETTING("pause_stream", bool, pause_stream);
      GET_SETTING("render_point_cloud", bool, render_point_cloud);
      GET_SETTING("background_color", int, background_color);

      bool new_data = false;
      if (kinect_[cur_kinect]->depth_frame_number() > depth_frame_number_) {
        new_data = true;
      }

      Float3 rgb_background;
      switch (background_color) {
      case BLUE_BACKGROUND:
        rgb_background.set(0.15f, 0.15f, 0.3f);
        break;
      case LBLUE_BACKGROUND:
        rgb_background.set(0.15f * 2.0f, 0.15f * 2.0f, 0.3f * 2.0f);
        break;
      case WHITE_BACKGROUND:
        rgb_background.set(1.0f, 1.0f, 1.0f);
        break;
      default:
        throw std::wruntime_error("App::run() - background_color enum invalid");
      }
      SET_SETTING("clear_color", Float3, rgb_background);

      // Create the image data (in RGB)
      if (new_data && !pause_stream) {
        kinect_[cur_kinect]->lockData();
        switch (kinect_output) {
        case OUTPUT_RGB:
          {
            const uint8_t* rgb = kinect_[cur_kinect]->rgb();
            memcpy(rgb_im_, rgb, sizeof(rgb_im_[0]) * rgb_dim * 3);
          }
          break;
        case OUTPUT_BLUE:
          {
            uint8_t rgb[3];
            rgb[0] = (uint8_t)(rgb_background[0] * 255.0f);
            rgb[1] = (uint8_t)(rgb_background[1] * 255.0f);
            rgb[2] = (uint8_t)(rgb_background[2] * 255.0f);
            for (uint32_t i = 0; i < depth_dim; i++) {
              depth_im_[i*3] = rgb[0];
              depth_im_[i*3+1] = rgb[1];
              depth_im_[i*3+2] = rgb[2];
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
                // NOTE: We're only locking the first Kinect so we might get
                // screen tearing or strange artifacts!
                const uint16_t* depth = kinect_[k]->depth();
                for (; v_dst < ((vtile + 1) * tile_res_y) && 
                  v_src < depth_h; v_dst++, v_src+=n_tiles_y) {
                  uint32_t u_dst = utile * tile_res_x;
                  uint32_t u_src = 0;
                  for (; u_dst < ((utile + 1) * tile_res_x) && 
                    u_src < depth_w; u_dst++, u_src+=n_tiles_x) {
                    uint32_t i_src = v_src * depth_w + u_src;
                    uint32_t i_dst = v_dst * depth_w + u_dst;
                    const uint8_t val = (depth[i_src] / 2) % 255;
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
          case OUTPUT_DEPTH_COLORED:
          {
            memcpy(depth_im_, kinect_[cur_kinect]->depth_colored(), 
              sizeof(depth_im_[0]) * depth_dim * 3);
          }
          break;
        }  // switch (kinect_output)

        if (render_point_cloud) {
          // Copy the data but don't resync the openGL geometry
          const uint8_t* rgb = kinect_[cur_kinect]->depth_colored();
          const XYZPoint* xyz = kinect_[cur_kinect]->xyz();
          for (uint32_t i = 0; i < depth_dim; i++) {
            geom_pts_->col()[i].set((float)rgb[i*3] / 255.0f,
              (float)rgb[i*3+1] / 255.0f, (float)rgb[i*3+2] / 255.0f);
            geom_pts_->pos()[i].set(xyz[i].x, xyz[i].y, xyz[i].z);
          }
        }

        // Update the frame number and timestamp
        depth_frame_number_ = kinect_[cur_kinect]->depth_frame_number();
        depth_frame_time_ = kinect_[cur_kinect]->depth_frame_time();

        kinect_[cur_kinect]->unlockData();
        
        // resync the openGL geometry
        if (render_point_cloud) {
          geom_pts_->resync();  // TODO: Figure out why this doesn't work.
        }

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
          jtil::image_util::FlipImageVertInPlace<uint8_t>(rgb_im_,
            rgb_w, rgb_h, 3);
          rgb_tex_->flagDirty();
          Renderer::g_renderer()->setBackgroundTexture(rgb_tex_);
          break;
        case OUTPUT_DEPTH:
        case OUTPUT_DEPTH_ALL_VIEWS:
        case OUTPUT_DEPTH_RAINBOW:
        case OUTPUT_BLUE:
        case OUTPUT_DEPTH_COLORED:
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

      geom_inst_pts_->render() = render_point_cloud;

      Renderer::g_renderer()->

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
    renderer::Camera* camera = Renderer::g_renderer()->camera();
    
    // Update the mouse position
    mouse_pos_old_.set(mouse_pos_);
    bool in_screen = Renderer::g_renderer()->getMousePosition(mouse_pos_);
    bool left_mouse_button = Renderer::g_renderer()->getMouseButtonStateLeft();
    
    // Rotate the camera if user wants to
    if (!math::Double2::equal(mouse_pos_, mouse_pos_old_) && 
        in_screen && left_mouse_button) {
      float dx = static_cast<float>(mouse_pos_[0] - mouse_pos_old_[0]);
      float dy = static_cast<float>(mouse_pos_[1] - mouse_pos_old_[1]);
      float camera_speed_rotation;
      GET_SETTING("camera_speed_rotation", float, camera_speed_rotation);
      camera->rotateCamera(dx * camera_speed_rotation,
                           dy * camera_speed_rotation);
    }
    
    // Move the camera if the user wants to
    Float3 cur_dir(0.0f, 0.0f, 0.0f);
    bool W = Renderer::g_renderer()->getKeyState(static_cast<int>('W'));
    bool A = Renderer::g_renderer()->getKeyState(static_cast<int>('A'));
    bool S = Renderer::g_renderer()->getKeyState(static_cast<int>('S'));
    bool D = Renderer::g_renderer()->getKeyState(static_cast<int>('D'));
    bool Q = Renderer::g_renderer()->getKeyState(static_cast<int>('Q'));
    bool E = Renderer::g_renderer()->getKeyState(static_cast<int>('E'));
    bool LShift = Renderer::g_renderer()->getKeyState(KEY_LSHIFT);
    if (W) {
      cur_dir[2] -= 1.0f;
    } 
    if (S) {
      cur_dir[2] += 1.0f;
    }
    if (A) {
      cur_dir[0] -= 1.0f;
    }
    if (D) {
      cur_dir[0] += 1.0f;
    }
    if (Q) {
      cur_dir[1] -= 1.0f;
    }
    if (E) {
      cur_dir[1] += 1.0f;
    }
    if (!(cur_dir[0] == 0.0f && cur_dir[1] == 0.0f && cur_dir[2] == 0.0f)) {
      cur_dir.normalize();
      float camera_speed = 1.0f;
      if (LShift) {
        GET_SETTING("camera_speed_fast", float, camera_speed);
      } else {
        GET_SETTING("camera_speed", float, camera_speed);        
      }
      Float3::scale(cur_dir, camera_speed * static_cast<float>(dt));
      camera->moveCamera(cur_dir);      
    }
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
    memset(rgb_im_, 0, sizeof(rgb_im_[0]) * rgb_dim * 3);
    rgb_tex_ = new Texture(GL_RGB, rgb_w, rgb_h, GL_RGB, 
      GL_UNSIGNED_BYTE, (unsigned char*)rgb_im_, false,
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
      ui::UIEnumVal(OUTPUT_DEPTH, "Depth"));
    ui->addSelectboxItem("kinect_output",
      ui::UIEnumVal(OUTPUT_DEPTH_ALL_VIEWS, "Depth (all views)"));
    ui->addSelectboxItem("kinect_output", 
      ui::UIEnumVal(OUTPUT_DEPTH_RAINBOW, "Depth Rainbow"));
    ui->addSelectboxItem("kinect_output",
      ui::UIEnumVal(OUTPUT_DEPTH_COLORED, "Depth Colored"));
    ui->addSelectboxItem("kinect_output",
      ui::UIEnumVal(OUTPUT_BLUE, "Blank"));

    ui->addCheckbox("render_kinect_fps", "Render Kinect FPS");
    ui->addCheckbox("continuous_snapshot", "Save continuous video stream");

    ui->addButton("screenshot_button", "RGB Screenshot", 
      App::screenshotCB);
    ui->addButton("reset_camera_button", "Reset Camera Position", 
      App::resetCameraCB);

    ui->addSelectbox("cur_kinect", "Current Kinect");
    for (uint32_t i = 0; i < MAX_NUM_KINECTS; i++) {
      ss.str("");
      ss << "device " << i;
      ui->addSelectboxItem("cur_kinect", ui::UIEnumVal(i, ss.str().c_str()));
    }

    ui->addCheckbox("stretch_image", "Stretch Image");
    ui->addCheckbox("pause_stream", "Pause Stream");
    ui->addCheckbox("render_point_cloud", "Render Point Cloud");

    ui->addSelectbox("background_color", "Background");
    ui->addSelectboxItem("background_color", 
      ui::UIEnumVal(BLUE_BACKGROUND, "Blue"));
    ui->addSelectboxItem("background_color", 
      ui::UIEnumVal(LBLUE_BACKGROUND, "Light Blue"));
    ui->addSelectboxItem("background_color", 
      ui::UIEnumVal(WHITE_BACKGROUND, "White"));

    ui->createTextWindow("kinect_fps_wnd", " ");
    jtil::math::Int2 pos(400, 0);
    ui->setTextWindowPos("kinect_fps_wnd", pos);

    LightDir* light_dir = new LightDir();
    light_dir->dir_world().set(0, 0, -1);
    Renderer::g_renderer()->addLight(light_dir);

    geom_inst_pts_ = 
      Renderer::g_renderer()->geometry_manager()->createDynamicGeometry(
      "PointCloud");
    Renderer::g_renderer()->scene_root()->addChild(geom_inst_pts_);
    geom_inst_pts_->mat().leftMultScale(1000.0f, 1000.0f, 1000.0f);
    geom_pts_ = geom_inst_pts_->geom();
    geom_pts_->primative_type() = VERT_POINTS;
    float point_cloud_size;
    GET_SETTING("point_cloud_size", float, point_cloud_size);
    geom_inst_pts_->point_line_size() = point_cloud_size;
    geom_inst_pts_->apply_lighting() = false;
    geom_pts_->addVertexAttribute(VERTATTR_POS);
    geom_pts_->addVertexAttribute(VERTATTR_COL);
    geom_pts_->pos().capacity(depth_dim);
    geom_pts_->pos().resize(depth_dim);
    geom_pts_->col().capacity(depth_dim);
    geom_pts_->col().resize(depth_dim);
    for (uint32_t i = 0; i < depth_dim; i++) {
      geom_pts_->pos()[i].set(0,0,-std::numeric_limits<float>::infinity());
      geom_pts_->col()[i].set(0,0,0);
    }
    geom_pts_->sync();

    geom_inst_lines_ = 
      Renderer::g_renderer()->geometry_manager()->createDynamicGeometry(
      "Skeletons");
    Renderer::g_renderer()->scene_root()->addChild(geom_inst_lines_);
    geom_inst_lines_->mat().leftMultScale(1000.0f, 1000.0f, 1000.0f);
    geom_lines_ = geom_inst_lines_->geom();
    geom_lines_->primative_type() = VERT_LINES;
    float skeleton_line_width;
    GET_SETTING("skeleton_line_width", float, skeleton_line_width);
    geom_inst_lines_->point_line_size() = skeleton_line_width;
    geom_inst_lines_->apply_lighting() = false;
    geom_lines_->addVertexAttribute(VERTATTR_POS);
    geom_lines_->addVertexAttribute(VERTATTR_COL);
    geom_lines_->pos().capacity(10*2);
    geom_lines_->pos().resize(10*2);
    geom_lines_->col().capacity(10*2);
    geom_lines_->col().resize(10*2);
    for (uint32_t i = 0; i < 10; i++) {
      geom_lines_->pos()[i*2].set(0,0,0);  // TODO: Make this INF
      geom_lines_->col()[i*2].set(1,1,1);
      geom_lines_->pos()[i*2+1].set(1,1,1);
      geom_lines_->col()[i*2+1].set(0,0,0);
    }
    geom_lines_->sync();
  }

  void App::screenshotCB() {
    int cur_kinect = 0;
    GET_SETTING("cur_kinect", int, cur_kinect);
    g_app_->kinect_[cur_kinect]->lockData();
    const uint8_t* rgb_src;
    rgb_src = g_app_->kinect_[cur_kinect]->rgb();
    std::stringstream ss;
    ss << "rgb_screenshot" << g_app_->screenshot_counter_ << ".jpg";
    jtil::renderer::Texture::saveRGBToFile(ss.str(), rgb_src, rgb_w, 
      rgb_h, true);
    std::cout << "RGB saved to file " << ss.str() << std::endl;
    const uint16_t* depth =  g_app_->kinect_[cur_kinect]->depth();
    ss.str("");
    ss << "depth_screenshot" << g_app_->screenshot_counter_ << ".bin";
    jtil::file_io::SaveArrayToFile<uint16_t>(depth, depth_dim, ss.str());
    std::cout << "Depth saved to file " << ss.str() << std::endl;
    g_app_->screenshot_counter_++;
    g_app_->kinect_[cur_kinect]->unlockData();
  }

  void App::resetCameraCB() {
    Renderer::g_renderer()->camera()->eye_pos_world().zeros();
    Renderer::g_renderer()->camera()->eye_rot().identity();
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

  // Save's data for a single kinect.  This is run in parallel.
  void App::saveKinectData(const uint32_t i) {

    // Save the depth and colored depth together
    if (kinect_[i]->depth_frame_time() > kinect_last_saved_depth_time_[i]) {
      // Make a local copy of the data so we do as little work with the lock
      // taken as possible.
      kinect_[i]->lockData();
      uint8_t* depth_rgb = 
        (uint8_t*)&tmp_data1_[i * kinect_interface::depth_arr_size_bytes * 2];
      uint8_t* depth_rgb_compressed = 
        (uint8_t*)&tmp_data2_[i * kinect_interface::depth_arr_size_bytes * 2];
      memcpy(depth_rgb, kinect_[i]->depth(), depth_arr_size_bytes);
      kinect_[i]->unlockData();

      char filename[256];
      // Kinect time stamp is in units of 100ns (or 0.1us)
      int64_t kinect_time_us = kinect_[i]->depth_frame_time() / 10;
      int64_t app_time_us = (int64_t)(time_since_start_ * 1.0e6);
      if (kinect_time_us > 1e11 || app_time_us > 1e11) {
        throw std::wruntime_error("App::saveKinectData() - App has been "
          "running too long to save data.  Please restart.");
      }
      snprintf(filename, 255, "im_K%012I64d_A%012I64d.bin", kinect_time_us, 
        app_time_us);

      // Compress the array
      static const int compression_level = 1;  // 1 fast, 2 better compression
      int compressed_length = fastlz_compress_level(compression_level,
        (void*)depth_rgb, depth_arr_size_bytes, (void*)depth_rgb_compressed);

      // Now save the array to file
#ifdef _WIN32
      _mkdir("./data/hand_depth_data/");  // Silently fails if dir exists
      std::string full_filename = "./data/hand_depth_data/" + 
        std::string(filename);
#endif
#ifdef __APPLE__
      std::string full_path = std::string("./data/hand_depth_data/");
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
      std::string full_filename = full_path + ss.str();
#endif
      // Save the file
      std::ofstream file(full_filename.c_str(), std::ios::out | std::ios::binary);
      if (!file.is_open()) {
        throw std::runtime_error(std::string("error opening file:") + filename);
      }
      // file.write((const char*)depth_rgb_data_, compressed_length);
      file.write((const char*)depth_rgb_compressed, compressed_length);
      file.close();
      kinect_last_saved_depth_time_[i] = kinect_[i]->depth_frame_time();
    }

    // Signal that we're done
    std::unique_lock<std::mutex> ul(thread_update_lock_);
    threads_finished_++;
    not_finished_.notify_all();  // Signify that all threads might have finished
    ul.unlock();
  }

}  // namespace app
