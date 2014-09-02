//
//  main_data.cpp
//
//  Takes the coeffient values from ModelFit and creates data for the convnet
//  classifier.
//

#if defined(WIN32) || defined(_WIN32)
  #include <windows.h>
  #include <tchar.h> 
  #include <stdio.h>
  #include <strsafe.h>
  #pragma comment(lib, "User32.lib")
#else
  #include <sys/types.h>
  #include <dirent.h>
#endif

#if defined(_WIN32) && defined(_DEBUG)
  #include <Windows.h>
  #include <crtdbg.h>  // for _CrtSetDbgFlag
#endif

#include <stdlib.h>
#include <cmath>
#include <thread>
#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>
#include <algorithm>

#include "renderer/open_gl_common.h"
#include "renderer/renderer.h"
#include "renderer/geometry/geometry.h"
#include "renderer/colors.h"
#include "renderer/camera/camera.h"
#include "renderer/lights/light_dir.h"
#include "renderer/texture/texture_renderable.h"
#include "renderer/texture/texture.h"
#include "renderer/geometry/geometry_colored_mesh.h"
#include "renderer/geometry/geometry_manager.h"
#include "renderer/geometry/geometry_colored_points.h"
#include "jtil/windowing/window.h"
#include "jtil/renderer/texture/texture.h"
#include "jtil/windowing/window_settings.h"
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector.h"
#include "jtil/clk/clk.h"
#include "jtil/string_util/string_util.h"
#include "kinect_interface_primesense/hand_net/hand_model_coeff.h"
#include "model_fit/hand_geometry_mesh.h"
#include "model_fit/model_renderer.h"
#include "model_fit/model_fit.h"
#include "kinect_interface_primesense/depth_images_io.h"
#include "kinect_interface_primesense/open_ni_funcs.h"
#include "renderer/gl_state.h"
#include "kinect_interface_primesense/hand_net/hand_net.h"  // for HandCoeffConvnet
#include "kinect_interface_primesense/hand_net/hand_image_generator.h"
#include "kinect_interface_primesense/hand_detector/hand_detector.h"
#include "jtil/file_io/file_io.h"
#include "jtil/threading/thread_pool.h"
#include "jtil/debug_util/debug_util.h"
#include "jtil/data_str/hash_funcs.h"
#include "jtil/video/video_stream.h"
#include "jtil/renderer/colors/colors.h"
#include "jtil/math/math_base.h"  // for NextPrime
#include "jtorch/jtorch.h"

using namespace std;
using namespace jtil::math;
using namespace jtil::data_str;
using namespace jtil::threading;
using namespace kinect_interface_primesense::hand_net;
using namespace kinect_interface_primesense::hand_detector;
using namespace kinect_interface_primesense;
using namespace model_fit;
using namespace jtil::file_io;
using namespace jtil::string_util;
using namespace jtil::windowing;
using renderer::Renderer;
using renderer::Geometry;
using renderer::GeometryManager;
using renderer::GeometryColoredMesh;
using renderer::GeometryColoredPoints;
using renderer::Texture;
using renderer::TEXTURE_WRAP_MODE;
using renderer::TEXTURE_FILTER_MODE;
using renderer::GLState;
using hand_net::HandNet;
using hand_net::HandCoeffConvnet;

// *************************************************************
// ******************* CHANGEABLE PARAMETERS *******************
// *************************************************************
#define BACKUP_HDD
// Training Set
const uint32_t num_im_dirs = 11;
string im_dirs[num_im_dirs] = {
  string("hand_depth_data_2013_05_01_1/"),  // JONATHAN
  string("hand_depth_data_2013_05_03_1/"),  // JONATHAN
  string("hand_depth_data_2013_05_06_1/"),  // JONATHAN
  string("hand_depth_data_2013_05_06_2/"),  // JONATHAN
  string("hand_depth_data_2013_05_19_1/"),  // JONATHAN
  string("hand_depth_data_2013_05_19_2/"),  // JONATHAN
  string("hand_depth_data_2013_06_15_1/"),  // JONATHAN
  string("hand_depth_data_2013_06_15_2/"),  // JONATHAN
  string("hand_depth_data_2013_06_15_3/"),  // JONATHAN
  string("hand_depth_data_2013_06_15_4/"),  // JONATHAN
  string("hand_depth_data_2013_06_15_5/")   // JONATHAN
};

// Test Set
/*
const uint32_t num_im_dirs = 2;
string im_dirs[num_im_dirs] = {
  string("hand_depth_data_2013_05_08_1/"),  // JONATHAN
  string("hand_depth_data_2013_05_06_3/"),  // MURPHY
};
*/

#define DST_IM_DIR_BASE string("") 

#define SAVE_FILES  // Save the database files

#if defined(SAVE_FILES)
  #define DESIRED_PLAYBACK_FPS 100000.0f
#else
  #define DESIRED_PLAYBACK_FPS 30.0f  // fps
#endif
#define NUM_WORKER_THREADS 6 

#define NUM_KINECTS 3

#define HM_SIZE 18*2  // Width and height of each heat map
#define HMW 4  // Number of heat maps horizontally
#define HMH 1  // Number of heat maps vertically
#define HMSTD 1.0f

// *************************************************************
// *************** END OF CHANGEABLE PARAMETERS ****************
// *************************************************************
#define HM_TEX_WIDTH ((HM_SIZE * HMW) + (HMW + 1))  // to accomodate borders
#define HM_TEX_HEIGHT ((HM_SIZE * HMH) + (HMH + 1))  // to accomodate borders

#define FRAME_TIME (1.0f / DESIRED_PLAYBACK_FPS)
// Some image defines, you shouldn't have to change these
#if defined(__APPLE__)
  #error "Apple is not yet supported!"
#else
  #ifdef BACKUP_HDD
    #define DATA_ROOT string("F:/hand_data/")  // Work computer
    //#define DATA_ROOT string("G:/hand_data/")  // Home computer
  #else
    #define DATA_ROOT string("./../data/")
  #endif
  //#define DST_DATA_ROOT string("./../data/")  // Save locally
  #define DST_DATA_ROOT string("F:/hand_data/dataset/train/")  // Save directly to portable hdd
#endif
#define DST_IM_DIR (DST_DATA_ROOT + DST_IM_DIR_BASE)

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif
#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

const bool fit_left = false;
const bool fit_right = true; 
const uint32_t num_hands = (fit_left ? 1 : 0) + (fit_right ? 1 : 0);

jtil::clk::Clk* clk = NULL;
double t1, t0;

// The main window and basic rendering system
Window* wnd = NULL;
WindowSettings settings;
Renderer* render = NULL;
bool is_running = true;
bool continuous_playback = false;
bool found_hand = false;
HandGeometryMesh* model[2];

// Hand modelNUM_PARAMETERS
HandModelCoeff* l_hand = NULL;  // Not using this yet
HandModelCoeff* r_hand = NULL;
ModelRenderer* hand_renderer = NULL;
uint8_t label[src_dim];

HandNet* hn = NULL;

// Kinect Image data
DepthImagesIO* image_io = NULL;
Vector<Triple<char*, int64_t, int64_t>> im_files[NUM_KINECTS];  // filename, kinect time, global time
Vector<uint32_t> file_dir_indices[NUM_KINECTS];  // directory each file came from
float cur_xyz_data[src_dim*3];
float cur_uvd_data[src_dim*3];
int16_t cur_depth_data[src_dim*3];
float cur_synthetic_depth_data[src_dim*4];
uint8_t cur_label_data[src_dim];
uint8_t cur_image_rgb[src_dim*3];
uint8_t cur_depth_rgb[src_dim*3];  // Used when exporting to PNG
int32_t cur_image = 0;
int32_t cur_kinect = 0;
GeometryColoredPoints* geometry_points= NULL;
bool render_depth = true;
int playback_step = 1;
Texture* tex = NULL;
Texture* tex_hm = NULL;
uint8_t tex_data_hpf[HN_IM_SIZE * HN_IM_SIZE * 3];
uint8_t tex_data_depth[HN_IM_SIZE * HN_IM_SIZE * 3];
uint8_t tex_data[HN_IM_SIZE * HN_IM_SIZE * 3 * 2];
uint8_t hm_tex_data[HM_TEX_WIDTH * HM_TEX_HEIGHT * 4];  // RGBA (allows odd widths)
float hm_data[HM_SIZE * HM_SIZE];
OpenNIFuncs openni_funcs;
Float4x4 camera_view[NUM_KINECTS];
const uint32_t num_coeff_fit = HAND_NUM_COEFF;
Float4x4 old_camera_view;

// Decision forests
HandDetector* hand_detect = NULL;

// Hand Image Generator for the convnet
HandImageGenerator* hand_image_generator_ = NULL;
float blank_coeff[HandCoeffConvnet::HAND_NUM_COEFF_CONVNET];
float coeff_convnet[HandCoeffConvnet::HAND_NUM_COEFF_CONVNET];  // UVD positions in hand image space
float uvd_gt[HAND_NUM_COEFF_UVD];  // UVD positions in 640x480 space

// Multithreading
ThreadPool* tp;

// Video stream
jtil::video::VideoStream* video_stream = NULL;
const uint32_t video_frame_rate = 30;
uint8_t* screendat = NULL;

void quit() {
  tp->stop();
  delete tp;
  for (uint32_t j = 0; j < NUM_KINECTS; j++) {
    for (uint32_t i = 0; i < im_files[j].size(); i++) {
      SAFE_DELETE_ARR(im_files[j][i].first);
    }
  }
  delete image_io;
  delete clk;
  delete render;
  if (l_hand) {
    delete l_hand;
  }
  if (r_hand) {
    delete r_hand;
  }
  delete tex;
  delete tex_hm;
  delete hand_renderer;
  delete model[0];
  delete wnd;
  delete geometry_points;
  delete hand_detect;
  delete hand_image_generator_;
  if (hn) {
    delete hn;
  }
  SAFE_DELETE(video_stream);
  SAFE_DELETE_ARR(screendat);
  Texture::shutdownTextureSystem();
  GLState::shutdownGLState();
  Window::killWindowSystem();
  jtorch::ShutdownJTorch();
  exit(0);
}

uint32_t cur_buff_size = 0;
void saveFrameToVideoStream(uint32_t num_frames) {
  if ((int)cur_buff_size < settings.width * settings.height * 3) {
    SAFE_DELETE_ARR(screendat);
    cur_buff_size = settings.width * settings.height * 3;
    screendat = new uint8_t[cur_buff_size];
  }
  if (video_stream) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glReadPixels(0, 0, settings.width, settings.height, GL_BGR_EXT, 
      GL_UNSIGNED_BYTE, screendat);
    for (uint32_t i = 0; i < num_frames; i++) {
      video_stream->addBGRFrame(screendat);
    }
    std::cout << num_frames << " frames saved to file" << std::endl;
  }
}

uint32_t findClosestFrame(const uint32_t i_kinect) {
  if (i_kinect == 0) {
    return cur_image;
  }
  int64_t src_timestamp = im_files[0][cur_image].second;
  int32_t i_start = 0;
  int32_t i_end = (int32_t)im_files[i_kinect].size();
  int32_t frame = i_start;
  int64_t min_delta_t = std::abs(src_timestamp - im_files[i_kinect][frame].second);
  for (int32_t i = i_start + 1; i < i_end; i++) {
    int64_t delta_t = std::abs(src_timestamp - im_files[i_kinect][i].second);
    if (delta_t < min_delta_t) {
      min_delta_t = delta_t;
      frame = i;
    }
  }
  return (uint32_t)frame;
}

void loadCurrentImage() {
  uint32_t i_match = findClosestFrame(cur_kinect);
  uint32_t i_match_k0 = findClosestFrame(0);
  //uint32_t i_match = cur_image;

  char* file = im_files[cur_kinect][i_match].first;
  
  string DIR = DATA_ROOT + im_dirs[file_dir_indices[cur_kinect][i_match]];
  string full_filename = DIR + string(file);
  //std::cout << "loading image: " << full_filename << std::endl;

  for (uint32_t i = 0; i < NUM_KINECTS; i++) {
    std::stringstream ss;
    try {
      ss << DIR << "calibration_data" << i << ".bin";
      LoadArrayFromFile<float>(camera_view[i].m, 16, ss.str());
    } catch (std::wruntime_error e) {
      camera_view[i].identity();
      std::cout << "WARNING: " << ss.str() << " doesn't exist.  ";
      std::cout << "Using Identity camera matrix." << std::endl;
      system("PAUSE");
    }
  }

  Float4x4 camera_view_inv, cur_view;
  Float4x4::inverse(camera_view_inv, camera_view[cur_kinect]);
  Float4x4::mult(cur_view, old_camera_view, camera_view_inv);
  hand_renderer->camera(cur_kinect)->set_view_mat_directly = true;
  hand_renderer->camera(cur_kinect)->view()->set(cur_view);
  hand_renderer->camera(cur_kinect)->updateProjection();

  // Load in the image
  image_io->LoadCompressedImage(full_filename, 
    cur_depth_data, cur_label_data, cur_image_rgb);
  openni_funcs.ConvertDepthImageToProjective((uint16_t*)cur_depth_data, 
    cur_uvd_data);
  openni_funcs.convertDepthToWorldCoordinates(cur_uvd_data, cur_xyz_data, 
    src_dim);
  openni_funcs.convertWorldToDepthCoordinates(cur_xyz_data, cur_uvd_data,
    src_dim);  // For testing... 
  found_hand = hand_detect->findHandLabels(cur_depth_data, 
    cur_xyz_data, HDLabelMethod::HDFloodfill, label);

  // Load in the coeffs
  string src_file = im_files[0][i_match_k0].first;  // Load the coeffs for the 0th kinect only
  r_hand->loadFromFile(DIR, string("coeffr_") + src_file);
  cout << "loaded image " << cur_image+1 << " of " << im_files[0].size() <<
    ", kinect " << (cur_kinect+1) << " of " << NUM_KINECTS << ": " << file << endl;

  if (!found_hand) {
    // skip this data point
    std::cout << "Warning couldn't find hand in " << im_files[cur_kinect][i_match].first;
    std::cout << std::endl;
  }

  // Create the downsampled hand image, background is at 1 and hand is
  // from 0 --> 1.  Also calculates the convnet input.

  model[0]->setRendererAttachement(false);
  HandGeometryMesh::setCurrentStaticHandProperties(r_hand->coeff());
  model[0]->updateMatrices(r_hand->coeff());

  // Render and get the synthetic depth image
  GLState::glsViewport(0, 0, 640, 480);
  float coeff[num_hands * num_coeff_fit];
  memcpy(coeff, r_hand->coeff(), sizeof(coeff[0])*num_coeff_fit);
  hand_renderer->drawDepthMap(coeff, num_coeff_fit, (PoseModel**)model, 
    num_hands, cur_kinect, false);
  hand_renderer->extractDepthMap(cur_synthetic_depth_data);
  GLState::glsViewport(0, 0, wnd->width(), wnd->height());

  hand_image_generator_->calcHandImage(cur_depth_data, label);

  // Correctly modify the coeff values to those that are learnable by the
  // convnet (for instance angles are bad --> store cos(x), sin(x) instead)
  model[0]->handCoeff2CoeffConvnet(r_hand, coeff_convnet,
    hand_image_generator_->hand_pos_wh(), hand_image_generator_->uvd_com(),
    *hand_renderer->camera(cur_kinect)->proj(), *hand_renderer->camera(cur_kinect)->view());
  model[0]->handCoeff2UVD(r_hand, uvd_gt,
    hand_image_generator_->hand_pos_wh(), hand_image_generator_->uvd_com(),
    *hand_renderer->camera(cur_kinect)->proj(), *hand_renderer->camera(cur_kinect)->view());
}

void saveFrame() {
#if defined(SAVE_FILES)
  uint32_t i_match = findClosestFrame(cur_kinect);
  //uint32_t i_match = cur_image;

  string DIR = DATA_ROOT + im_dirs[file_dir_indices[cur_kinect][i_match]];
  uint32_t IM_DIR_hash = HashString(MAX_UINT32, DIR);

  std::stringstream ss;

  // Save the RGB out to a PNG
  char filename[256];
  snprintf(filename, 255, "rgb_%d_%07d.png", cur_kinect+1, cur_image+1);
  Texture::saveRGBToFile(DST_IM_DIR + filename, cur_image_rgb, src_width, src_height, 
    true);
  // Save the depth out to a PNG by packing the 16 bits into the G and B
  // channels
  for (uint32_t i = 0; i < src_dim; i++) {
    cur_depth_rgb[i*3] = 0;
    cur_depth_rgb[i*3+1] = (uint8_t)((cur_depth_data[i] & (int16_t)0x7F00) >> 8);
    cur_depth_rgb[i*3+2] = (uint8_t)(cur_depth_data[i] & (int16_t)0xFF);
  }
  snprintf(filename, 255, "depth_%d_%07d.png", cur_kinect+1, cur_image+1);
  Texture::saveRGBToFile(DST_IM_DIR + filename, cur_depth_rgb, src_width, src_height, 
    true);
  // Save out the synthetic depth
  for (uint32_t i = 0; i < src_dim; i++) {
    int16_t val = static_cast<int16_t>(cur_synthetic_depth_data[i]);
    cur_depth_rgb[i*3] = 0;
    cur_depth_rgb[i*3+1] = (uint8_t)((val & (int16_t)0x7F00) >> 8);
    cur_depth_rgb[i*3+2] = (uint8_t)(val & (int16_t)0xFF);
  }
  snprintf(filename, 255, "synthdepth_%d_%07d.png", cur_kinect+1, cur_image+1);
  Texture::saveRGBToFile(DST_IM_DIR + filename, cur_depth_rgb, src_width, src_height, 
    true);

  if (cur_kinect == 0) {
    // Save out the ground truth locations
    snprintf(filename, 255, "uvd_%d_%07d.bin", cur_kinect+1, cur_image+1);
    SaveArrayToFile<float>(uvd_gt,
      HAND_NUM_COEFF_UVD, DST_IM_DIR + filename);
  } else {
    // Otherwise, the easiest way to do this is to save out the 2 camera
    // matrices in Matlab (for camera 1 and camera 2) and then do the
    // transformation there.
    float mats[16*2];
    memcpy(mats, camera_view[0].m, 16*sizeof(mats[0]));
    memcpy(&mats[16], camera_view[cur_kinect].m, 16*sizeof(mats[0]));
    snprintf(filename, 255, "mat1matk_%d_%07d.bin", cur_kinect+1, cur_image+1);
    SaveArrayToFile<float>(mats, 16*2, DST_IM_DIR + filename);
  }
#endif
}

void keyboardCB(int key, int scancode, int action, int mods) {
  string full_im_filename;
  string full_hpf_im_filename;
  string r_coeff_file;
  string l_coeff_file;
  string src_file;

  if (action == RELEASED) {
    switch (key) {
    case 'P':
    case 'p':
      continuous_playback = !continuous_playback;
      break;
    case KEY_KP_ADD:
      if (cur_image < (int32_t)im_files[0].size() - 1) {
        cur_image++;
      }
      break;
    case KEY_KP_SUBTRACT:
      if (cur_image > 0) {
        cur_image--;
      }
      break;
    case '0':
      if (cur_image < (int32_t)im_files[0].size() - 100) {
        cur_image += 100;
      } else {
        cur_image = (int32_t)im_files[0].size() - 1;
      }

      break;
    case '9':
      if (cur_image > 99) {
        cur_image -= 100;
      } else {
        cur_image = 0;
      }
      break;
    case 'Q':
    case 'q':
    case KEY_ESC:
      is_running = false;
      break;
  case 'z':
  case 'Z':
    if (action == RELEASED) {
      if (video_stream) {
        SAFE_DELETE(video_stream);
        std::cout << "Video stream closed" << std::endl;
      } else {
        std::wstring file = L"model_fit_data_video.avi";
        video_stream = new jtil::video::VideoStream(settings.width,
          settings.height, 3, video_frame_rate, file);
        std::cout << "Video stream opened: " << 
          jtil::string_util::ToNarrowString(file) << std::endl;
      }
    }
    break;
    }
    if (key == KEY_KP_ADD || key == KEY_KP_SUBTRACT || key == '0' || 
      key == '9') {
      loadCurrentImage();
    }
  }
}

void renderFrame() {
  ((HandGeometryMesh*)model[0])->setRendererAttachement(true);

  // Render the images for debugging
  const float* im = hand_image_generator_->hpf_hand_image_cpu();
  const float scale = 4.0f;
  for (uint32_t v = 0; v < HN_IM_SIZE; v++) {
    for (uint32_t u = 0; u < HN_IM_SIZE; u++) {
      float val = 0.5f + im[v * HN_IM_SIZE + u] / scale;  // Centered around 0.5
      val = std::min<float>(1.0f, std::max<float>(0.0f, val));  // 0 to 1
      uint8_t cval = (uint8_t)(255.0f * val);
      uint32_t idst = (HN_IM_SIZE-v-1) * HN_IM_SIZE + u;
      tex_data_hpf[idst * 3] = cval;
      tex_data_hpf[idst * 3 + 1] = cval;
      tex_data_hpf[idst * 3 + 2] = cval;
    }
  }

  hand_image_generator_->annotateFeatsToHandImage(tex_data_hpf, coeff_convnet);

  im = hand_image_generator_->hand_image_cpu();
  const float gain = 3.0f;
  const float off = -gain * 0.5f + 0.5f;  // So that it is centered around 0.5 again
  for (uint32_t v = 0; v < HN_IM_SIZE; v++) {
    for (uint32_t u = 0; u < HN_IM_SIZE; u++) {
      int32_t val = (int32_t)((im[v * HN_IM_SIZE + u] * gain + off) * 255.0f);
      // Clamp the value from 0 to 255 (otherwise we'll get wrap around)
      uint8_t val8 = (uint8_t)std::min<int32_t>(std::max<int32_t>(val,0),255);
      uint32_t idst = (HN_IM_SIZE-v-1) * HN_IM_SIZE + u;
      // Texture needs to be flipped vertically and 0 --> 255
      tex_data_depth[idst * 3] = val8;
      tex_data_depth[idst * 3 + 1] = val8;
      tex_data_depth[idst * 3 + 2] = val8;
    }
  }

  for (uint32_t v = 0; v < HN_IM_SIZE; v++) {
    for (uint32_t u = 0; u < HN_IM_SIZE; u++) {
      uint32_t src_index = v * HN_IM_SIZE + u;
      uint32_t dst_index_depth = v * HN_IM_SIZE * 2 + u;
      tex_data[3 * dst_index_depth] = tex_data_depth[3 * src_index];
      tex_data[3 * dst_index_depth + 1] = tex_data_depth[3 * src_index + 1];
      tex_data[3 * dst_index_depth + 2] = tex_data_depth[3 * src_index + 2];
      uint32_t dst_index_hpf = v * HN_IM_SIZE * 2 + u + HN_IM_SIZE;
      tex_data[3 * dst_index_hpf] = tex_data_hpf[3 * src_index];
      tex_data[3 * dst_index_hpf + 1] = tex_data_hpf[3 * src_index + 1];
      tex_data[3 * dst_index_hpf + 2] = tex_data_hpf[3 * src_index + 2];
    }
  }

  tex->reloadData((unsigned char*)tex_data);

  render->renderFullscreenQuad(tex);

  wnd->swapBackBuffer();
}

using std::cout;
using std::endl;
Float4x4 mat_tmp;

int main(int argc, char *argv[]) {
  static_cast<void>(argc); static_cast<void>(argv);
#if defined(_DEBUG) && defined(_WIN32)
  jtil::debug::EnableMemoryLeakChecks();
  // jtil::debug::EnableAggressiveMemoryLeakChecks();
  // jtil::debug::SetBreakPointOnAlocation(159153);
#endif

  cout << "Usage:" << endl;
  cout << "p - Cycle through the frames continuously" << endl;
  cout << "+/- - Forward and back a frame" << endl;
  cout << "9/0 - Forward and back 100 frames" << endl;
  cout << "d - Delete frame (when loading processed images)" << endl;
  cout << "z - Open video stream" << endl;
  cout << "q/ESC - Quit" << endl;
  
  try {
    tp = new ThreadPool(NUM_WORKER_THREADS);

    clk = new jtil::clk::Clk();
    t1 = clk->getTime();
    t0 = t1;
    
    // Initialize Windowing system
    Window::initWindowSystem();
    Texture::initTextureSystem();

    jtorch::InitJTorch("../../jtorch");

    // Fill the settings structure
    settings.width = 640 * 2;
#ifdef VISUALIZE_HEAT_MAPS
    settings.height = 480 * 2;
#else
    settings.height = 640;
#endif
    settings.fullscreen = false;
    settings.title = string("Hand Fit Project");
    settings.gl_major_version = 4;
    settings.gl_minor_version = 0;
    settings.num_depth_bits = 24;
    settings.num_stencil_bits = 0;
    settings.num_rgba_bits = 8;
    settings.samples = 1;
    
    // Create the window so that we have a valid openGL context
    wnd = new Window(settings);
    GLState::initGLState();    
    
    // Create an instance of the renderer
    FloatQuat eye_rot; 
    eye_rot.identity();
    Float3 eye_pos(0, 0, 0);
    render = new Renderer();
    float fov_vert_deg = 360.0f * OpenNIFuncs::fVFOV_primesense_109 / 
      (2.0f * (float)M_PI);
    render->init(eye_rot, eye_pos, settings.width, settings.height,
      -HAND_MODEL_CAMERA_VIEW_PLANE_NEAR, -HAND_MODEL_CAMERA_VIEW_PLANE_FAR, 
      fov_vert_deg);

    model[0] = new HandGeometryMesh(HandType::RIGHT);
    
    // Load the Kinect data for fitting from file and process it
    image_io = new DepthImagesIO();
    for (uint32_t i = 0; i < NUM_KINECTS; i++) {
      //image_io->GetFilesInDirectory(im_files[i], DATA_ROOT + im_dirs[0], i);
      image_io->GetFilesInDirectories(im_files[i], file_dir_indices[i], im_dirs, 
        DATA_ROOT, num_im_dirs, i, NULL);
      if (im_files[i].size() == 0) {
        std::cout << "No Files found!" << std::endl;
  #if defined(WIN32) || defined(_WIN32)
        system("pause");
  #endif
        quit();
      } else {
        std::cout << "Loaded " << im_files[i].size() << " frames for kinect ";
        std::cout << i << std::endl;
      }
    }

    image_io->AlignKinects(im_files, NUM_KINECTS);

    // Attach callback functions for event handling
    wnd->registerKeyboardCB(keyboardCB);
    wnd->registerMousePosCB(NULL);
    wnd->registerMouseButCB(NULL);
    wnd->registerMouseWheelCB(NULL);
    
    // Create the hand data and attach it to the renderer for lighting
    if (num_hands != 1) {
      throw std::runtime_error("ERROR: This main routine only works with one "
        "hand (left or right)!");
    }
    hand_renderer = new ModelRenderer(NUM_KINECTS);
    r_hand = new HandModelCoeff(HandType::RIGHT);
    l_hand = new HandModelCoeff(HandType::LEFT);

    // We need to save the original camera view
    old_camera_view.set(*hand_renderer->camera(0)->view());

    std::cout << "Using a cropped source image of " << HN_SRC_IM_SIZE;
    std::cout << std::endl << "Final image size after processing is ";
    std::cout << (HN_IM_SIZE) << std::endl;

    hand_detect = new HandDetector(tp);
    std::cout << "Loading forest from " << "./../" << FOREST_DATA_FILENAME << 
      std::endl;
    hand_detect->init(src_width, src_height,"./../" + FOREST_DATA_FILENAME);
    hand_image_generator_ = new HandImageGenerator(HN_DEFAULT_NUM_CONV_BANKS);
    for (uint32_t i = 0; i < HandCoeffConvnet::HAND_NUM_COEFF_CONVNET; i++) {
      blank_coeff[i] = 0.0f;
    }

    loadCurrentImage();
    tex = new Texture(GL_RGB8, HN_IM_SIZE * 2, HN_IM_SIZE, GL_RGB, 
      GL_UNSIGNED_BYTE, (unsigned char*)tex_data, 
      TEXTURE_WRAP_MODE::TEXTURE_CLAMP, false,
      TEXTURE_FILTER_MODE::TEXTURE_NEAREST);
    tex_hm = new Texture(GL_RGBA8, HM_TEX_WIDTH, HM_TEX_HEIGHT, GL_RGBA, 
      GL_UNSIGNED_BYTE, (unsigned char*)hm_tex_data, 
      TEXTURE_WRAP_MODE::TEXTURE_CLAMP, false,
      TEXTURE_FILTER_MODE::TEXTURE_NEAREST);

    render->renderFrame(0);

    while (is_running) {
      renderFrame();
      saveFrame();

      if (continuous_playback) {
        t1 = clk->getTime();
        float dt = (float)(t1 - t0);
        if (dt < FRAME_TIME) {
          std::this_thread::sleep_for(
            std::chrono::microseconds((int)(1000000.0f * (FRAME_TIME - dt))));
        }
        t0 = t1;
        if (cur_image < (int32_t)im_files[0].size() - 1) {
          cur_image++;
        } else {
          if (cur_kinect == NUM_KINECTS-1) {
            // Otherwise we're all done!
            continuous_playback = false;
          } else {
            // Advance to the next kinect
            cur_kinect = cur_kinect + 1;
            cur_image = 0;
          }
        }
        saveFrameToVideoStream(1);
        loadCurrentImage();
      }
    }

    std::cout << "All done!" << std::endl;
    quit();

  } catch (std::runtime_error e) {
    printf("%s\n", e.what());
#if defined(WIN32) || defined(_WIN32)
    system("PAUSE");
#endif
  }
  
  return 0;
}
