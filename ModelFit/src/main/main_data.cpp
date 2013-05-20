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
#include "windowing/window.h"
#include "windowing/window_settings.h"
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector.h"
#include "jtil/clk/clk.h"
#include "jtil/string_util/string_util.h"
#include "kinect_interface/hand_net/hand_model_coeff.h"
#include "model_fit/hand_geometry_mesh.h"
#include "model_fit/model_renderer.h"
#include "model_fit/model_fit.h"
#include "kinect_interface/depth_images_io.h"
#include "kinect_interface/open_ni_funcs.h"
#include "renderer/gl_state.h"
#include "kinect_interface/hand_net/hand_net.h"  // for HandCoeffConvnet
#include "kinect_interface/hand_net/hand_image_generator.h"
#include "kinect_interface/hand_detector/hand_detector.h"
#include "jtil/file_io/file_io.h"
#include "jtil/threading/thread_pool.h"
#include "jtil/debug_util/debug_util.h"
#include "jtil/data_str/hash_funcs.h"
#include "jtil/math/math_base.h"  // for NextPrime
#include "jtorch/jtorch.h"

// *************************************************************
// ******************* CHANGEABLE PARAMETERS *******************
// *************************************************************
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_01_1/")  // Cal + Fit + Proc (5405)
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_03_1/")  // Cal + Fit + Proc (6533)
#define IM_DIR_BASE string("data/hand_depth_data_2013_05_06_1/")  // Cal + Fit (8709)
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_06_2/")  // Cal + Fit (8469)
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_06_3/")  // Cal + Fit (5815)
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_08_1/")  // Cal + Fit (2440) (Tr-data)
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_19_1/")  // Cal + Fit (5969)
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_19_2/")  // Cal + Fit (6781)  Total: 47681 

#define DST_IM_DIR_BASE string("data/hand_depth_data_processed_for_CN/") 
//#define DST_IM_DIR_BASE string("data/hand_depth_data_processed_for_CN_testset/") 

//#define LOAD_PROCESSED_IMAGES  // Load the images from the dst image directory
#define SAVE_FILES  // Only enabled when we're not loading processed images
//#define SAVE_SYNTHETIC_IMAGE  // Use portion of the screen governed by 
//                              // HandForests, but save synthetic data (only 
//                              // takes effect when not loading processed images)

#define SAVE_DEPTH_IMAGES  // Save the regular depth files --> Only when SAVE_FILES defined
#define SAVE_HPF_IMAGES  // Save the hpf files --> Only when SAVE_FILES defined

#if !defined(LOAD_PROCESSED_IMAGES) && defined(SAVE_FILES)
  #define DESIRED_PLAYBACK_FPS 100000.0f
#else
  #define DESIRED_PLAYBACK_FPS 30.0f  // fps
#endif
#define NUM_WORKER_THREADS 6

// *************************************************************
// *************** END OF CHANGEABLE PARAMETERS ****************
// *************************************************************

#define FRAME_TIME (1.0f / DESIRED_PLAYBACK_FPS)
// Some image defines, you shouldn't have to change these
#if defined(__APPLE__)
#define KINECT_HANDS_ROOT string("./../../../../../../../../../../")
#else
#define KINECT_HANDS_ROOT string("./../")
#endif
#define IM_DIR (KINECT_HANDS_ROOT + IM_DIR_BASE)
#define DST_IM_DIR (KINECT_HANDS_ROOT + DST_IM_DIR_BASE)

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif
#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

const bool fit_left = false;
const bool fit_right = true; 
const uint32_t num_hands = (fit_left ? 1 : 0) + (fit_right ? 1 : 0);

using namespace std;
using namespace jtil::math;
using namespace jtil::data_str;
using namespace jtil::threading;
using namespace kinect_interface::hand_net;
using namespace kinect_interface::hand_detector;
using namespace kinect_interface;
using namespace model_fit;
using namespace jtil::file_io;
using namespace jtil::string_util;
using renderer::Renderer;
using renderer::Geometry;
using renderer::GeometryManager;
using renderer::GeometryColoredMesh;
using renderer::GeometryColoredPoints;
using renderer::Texture;
using renderer::TEXTURE_WRAP_MODE;
using renderer::TEXTURE_FILTER_MODE;
using windowing::Window;
using windowing::WindowSettings;
using renderer::GLState;
using hand_net::HandNet;
using hand_net::HandCoeffConvnet;

jtil::clk::Clk* clk = NULL;
double t1, t0;

// The main window and basic rendering system
Window* wnd = NULL;
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

// Kinect Image data
uint32_t IM_DIR_hash = HashString(MAX_UINT32, IM_DIR);
DepthImagesIO* image_io = NULL;
Vector<Triple<char*, int64_t, int64_t>> im_files;  // filename, kinect time, global time
float cur_xyz_data[src_dim*3];
float cur_uvd_data[src_dim*3];
int16_t cur_depth_data[src_dim*3];
float cur_synthetic_depth_data[src_dim*4];
uint8_t cur_label_data[src_dim];
uint8_t cur_image_rgb[src_dim*3];
int32_t cur_image = 0;
GeometryColoredPoints* geometry_points= NULL;
bool render_depth = true;
int playback_step = 1;
Texture* tex = NULL;
uint8_t tex_data_hpf[HN_IM_SIZE * HN_IM_SIZE * 3];
uint8_t tex_data_depth[HN_IM_SIZE * HN_IM_SIZE * 3];
uint8_t tex_data[HN_IM_SIZE * HN_IM_SIZE * 3 * 2];
OpenNIFuncs openni_funcs;
Float4x4 camera_view;
const uint32_t num_coeff_fit = HAND_NUM_COEFF;

// Decision forests
HandDetector* hand_detect = NULL;

// Hand Image Generator for the convnet
HandImageGenerator* hand_image_generator_ = NULL;
float blank_coeff[HandCoeffConvnet::HAND_NUM_COEFF_CONVNET];
float coeff_convnet[HandCoeffConvnet::HAND_NUM_COEFF_CONVNET];

// Multithreading
ThreadPool* tp;

void quit() {
  tp->stop();
  delete tp;
  for (uint32_t i = 0; i < im_files.size(); i++) {
    SAFE_DELETE_ARR(im_files[i].first);
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
  delete hand_renderer;
  delete model[0];
  delete wnd;
  delete geometry_points;
  delete hand_detect;
  delete hand_image_generator_;
  Texture::shutdownTextureSystem();
  GLState::shutdownGLState();
  Window::killWindowSystem();
  exit(0);
}

void loadCurrentImage() {
  char* file = im_files[cur_image].first;
#ifdef LOAD_PROCESSED_IMAGES
  string DIR = DST_IM_DIR;
#else
  string DIR = IM_DIR;
#endif
  string full_filename = DIR + string(file);
  //std::cout << "loading image: " << full_filename << std::endl;

#ifdef LOAD_PROCESSED_IMAGES
  string full_hpf_im_filename = DIR + std::string("hpf_") + im_files[cur_image].first;

  // Just load the processed image directly
  const float* im = hand_image_generator_->hand_image_cpu();
  LoadArrayFromFile<float>(const_cast<float*>(im), HN_IM_SIZE * HN_IM_SIZE, 
    full_filename);
  im = hand_image_generator_->hpf_hand_image_cpu();
  LoadArrayFromFile<float>(const_cast<float*>(im), HN_IM_SIZE * HN_IM_SIZE, 
    full_hpf_im_filename);
  memset(cur_depth_data, 0, src_dim * sizeof(cur_depth_data[0]));
  memset(cur_label_data, 0, src_dim * sizeof(cur_label_data[0]));
  memset(cur_image_rgb, 0, 3 * src_dim * sizeof(cur_image_rgb[0]));
  memset(cur_xyz_data, 0, 3 * src_dim * sizeof(cur_xyz_data[0]));

  string src_file = im_files[cur_image].first;
  src_file = src_file.substr(10, src_file.length());
  string r_coeff_file = DST_IM_DIR + string("coeffr_") + src_file;
  LoadArrayFromFile<float>(coeff_convnet, 
    HandCoeffConvnet::HAND_NUM_COEFF_CONVNET, r_coeff_file);
  if (cur_image % 100 == 0) {
    cout << "loaded image " << cur_image+1 << " of " << im_files.size() << endl;
  }
#else
  // Load in the image
  image_io->LoadCompressedImage(full_filename, 
    cur_depth_data, cur_label_data, cur_image_rgb);
  openni_funcs.ConvertDepthImageToProjective((uint16_t*)cur_depth_data, 
    cur_uvd_data);
  openni_funcs.convertDepthToWorldCoordinates(cur_uvd_data, cur_xyz_data, 
    src_dim);
  found_hand = hand_detect->findHandLabels(cur_depth_data, 
    cur_xyz_data, HDLabelMethod::HDFloodfill, label);

  // Load in the coeffs
  string src_file = im_files[cur_image].first;
#ifdef LOAD_PROCESSED_IMAGES
  src_file = src_file.substr(10, src_file.length());
#endif
  r_hand->loadFromFile(DIR, string("coeffr_") + src_file);
  if (cur_image % 100 == 0) {
    cout << "loaded image " << cur_image+1 << " of " << im_files.size() << endl;
  }

  if (!found_hand) {
    // skip this data point
    std::cout << "Warning couldn't find hand in " << im_files[cur_image].first;
    std::cout << std::endl;
  }

  // Create the downsampled hand image, background is at 1 and hand is
  // from 0 --> 1.  Also calculates the convnet input.
#ifdef SAVE_HPF_IMAGES
  const bool create_hpf_images = true;
#else
  const bool create_hpf_images = false;
#endif

  model[0]->setRendererAttachement(false);
  HandGeometryMesh::setCurrentStaticHandProperties(r_hand->coeff());

#ifdef SAVE_SYNTHETIC_IMAGE
  GLState::glsViewport(0, 0, 640, 480);
  float coeff[num_hands * num_coeff_fit];
  memcpy(coeff, r_hand->coeff(), sizeof(coeff[0])*num_coeff_fit);
  hand_renderer->drawDepthMap(coeff, num_coeff_fit, (PoseModel**)model, 
    num_hands, 0, false);
  hand_renderer->extractDepthMap(cur_synthetic_depth_data);
  hand_image_generator_->calcHandImage(cur_depth_data, label,
    create_hpf_images, cur_synthetic_depth_data);
  GLState::glsViewport(0, 0, wnd->width(), wnd->height());
#else
  hand_image_generator_->calcHandImage(cur_depth_data, label,
    create_hpf_images);
#endif

  hand_renderer->camera(0)->updateProjection();
  // Correctly modify the coeff values to those that are learnable by the
  // convnet (for instance angles are bad --> store cos(x), sin(x) instead)
  model[0]->handCoeff2CoeffConvnet(r_hand, coeff_convnet,
    hand_image_generator_->hand_pos_wh(), hand_image_generator_->uvd_com(),
    *hand_renderer->camera(0)->proj(), *hand_renderer->camera(0)->view());
#endif
}

void saveFrame() {
#if defined(SAVE_FILES) && !defined(LOAD_PROCESSED_IMAGES)
  // Check that the current coeff doesn't have features points that are off 
  // screen --> Usually an indicator that the HandDetector messed up
  for (uint32_t i = 0; i < HandCoeffConvnet::HAND_NUM_COEFF_CONVNET && 
    found_hand; i+= FEATURE_SIZE) {
    if (coeff_convnet[i] < 0 || coeff_convnet[i] > 1) {
      std::cout << " Coeff is off screen.  Not saving files." << std::endl;
      found_hand = false;
    }
    if (coeff_convnet[i+1] < 0 || coeff_convnet[i+1] > 1) {
      std::cout << " Coeff is off screen.  Not saving files." << std::endl;
      found_hand = false;
    }
  }

  if (found_hand) {
    std::stringstream ss;
    
    // Save the cropped image to file:
#if defined(SAVE_DEPTH_IMAGES)
    ss << DST_IM_DIR << "processed_" << IM_DIR_hash << "_" << im_files[cur_image].first;
    SaveArrayToFile<float>(hand_image_generator_->hand_image_cpu(),
      hand_image_generator_->size_images(), ss.str());
#endif

    // Save the HPF images to file:
#if defined(SAVE_HPF_IMAGES)
    ss.str(string(""));
    ss << DST_IM_DIR << "hpf_processed_" << IM_DIR_hash << "_" << im_files[cur_image].first;
    SaveArrayToFile<float>(hand_image_generator_->hpf_hand_image_cpu(),
      hand_image_generator_->size_images(), ss.str());
#endif

    ss.str(string(""));
    ss << DST_IM_DIR << "coeffr_" << IM_DIR_hash << "_" << im_files[cur_image].first;
    SaveArrayToFile<float>(coeff_convnet,
      HandCoeffConvnet::HAND_NUM_COEFF_CONVNET, ss.str());

    // Don't save LHand data...  It's just blank anyway and will eat into our 
    // disk IO
    //ss.str(string(""));
    //ss << DST_IM_DIR << "coeffl_" << IM_DIR_hash << "_" << im_files[cur_image].first;
    //SaveArrayToFile<float>(blank_coeff,
    //  HandCoeffConvnet::HAND_NUM_COEFF_CONVNET, ss.str());
  }
#endif
}

void keyboardCB(int key, int action) {
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
      if (cur_image < (int32_t)im_files.size() - 1) {
        cur_image++;
      }
      break;
    case KEY_KP_SUBTRACT:
      if (cur_image > 0) {
        cur_image--;
      }
      break;
    case '0':
      if (cur_image < (int32_t)im_files.size() - 100) {
        cur_image += 100;
      } else {
        cur_image = (int32_t)im_files.size() - 1;
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
#ifdef LOAD_PROCESSED_IMAGES
  case 'D':
  case 'd':
#if defined(WIN32) || defined(_WIN32)
    full_im_filename = DST_IM_DIR + string(im_files[cur_image].first);
    full_hpf_im_filename = DST_IM_DIR + std::string("hpf_") + 
      im_files[cur_image].first;
    src_file = im_files[cur_image].first;
    src_file = src_file.substr(10, src_file.length());
    r_coeff_file = DST_IM_DIR + string("coeffr_") + src_file;
    //l_coeff_file = DST_IM_DIR + string("coeffl_") + src_file;

    if(!DeleteFile(full_im_filename.c_str()) ||
      !DeleteFile(r_coeff_file.c_str()) /*||
      !DeleteFile(l_coeff_file.c_str())*/ ||
      !DeleteFile(full_hpf_im_filename.c_str())) {
      cout << "Error deleting files: " << endl;
      cout << "    - " << full_im_filename.c_str() << endl;
      cout << "    - " << r_coeff_file.c_str() << endl;
      //cout << "    - " << l_coeff_file.c_str() << endl;
      cout << "    - " << full_hpf_im_filename.c_str() << endl;
      cout << endl;
    } else {
      cout << "Files deleted sucessfully: " << endl;
      cout << "    - " << full_im_filename.c_str() << endl;
      cout << "    - " << r_coeff_file.c_str() << endl;
      //cout << "    - " << l_coeff_file.c_str() << endl;
      cout << "    - " << full_hpf_im_filename.c_str() << endl;
      cout << endl;
      im_files.deleteAtAndShift((uint32_t)cur_image);
      loadCurrentImage();
    }
#else
    cout << "Delete function not implemented for Mac OS X" << endl;
#endif
    break;
#endif
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
  float min = im[0];
  float max = im[0];
  for (uint32_t i = 1; i < HN_IM_SIZE * HN_IM_SIZE; i++) {
    min = std::min<float>(min, im[i]);
    max = std::max<float>(max, im[i]); 
  }
  float range = max - min;
  for (uint32_t v = 0; v < HN_IM_SIZE; v++) {
    for (uint32_t u = 0; u < HN_IM_SIZE; u++) {
      uint32_t val = (uint32_t)((im[v * HN_IM_SIZE + u] - min)/range * 255.0f);
      // Clamp the value from 0 to 255 (otherwise we'll get wrap around)
      uint8_t val8 = (uint8_t)std::min<uint32_t>(std::max<uint32_t>(val,0),255);
      uint32_t idst = (HN_IM_SIZE-v-1) * HN_IM_SIZE + u;
      // Texture needs to be flipped vertically and 0 --> 255
      tex_data_hpf[idst * 3] = val8;
      tex_data_hpf[idst * 3 + 1] = val8;
      tex_data_hpf[idst * 3 + 2] = val8;
    }
  }

  hand_image_generator_->annotateFeatsToHandImage(tex_data_hpf, coeff_convnet);

  im = hand_image_generator_->hand_image_cpu();
  for (uint32_t v = 0; v < HN_IM_SIZE; v++) {
    for (uint32_t u = 0; u < HN_IM_SIZE; u++) {
      uint32_t val = (uint32_t)(im[v * HN_IM_SIZE + u] * 255.0f);
      // Clamp the value from 0 to 255 (otherwise we'll get wrap around)
      uint8_t val8 = (uint8_t)std::min<uint32_t>(std::max<uint32_t>(val,0),255);
      uint32_t idst = (HN_IM_SIZE-v-1) * HN_IM_SIZE + u;
      // Texture needs to be flipped vertically and 0 --> 255
      tex_data_depth[idst * 3] = val8;
      tex_data_depth[idst * 3 + 1] = val8;
      tex_data_depth[idst * 3 + 2] = val8;
    }
  }

  hand_image_generator_->annotateFeatsToHandImage(tex_data_depth, coeff_convnet);

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

uint32_t GetProcessedFilesInDirectory(
    jtil::data_str::Vector<Triple<char*, int64_t, int64_t>>& files_in_directory, 
    const string& directory, const uint32_t kinect_num) {
    std::vector<Triple<char*, int64_t, int64_t>> files;
#if defined(WIN32) || defined(_WIN32)
    // Prepare string for use with FindFile functions.  First, copy the
    // string to a buffer, then append '\*' to the directory name.
    TCHAR szDir[MAX_PATH];
    StringCchCopy(szDir, MAX_PATH, directory.c_str());
    std::stringstream ss;
    ss << "\\processed_*_hands" << kinect_num << "_*.bin";
    StringCchCat(szDir, MAX_PATH, ss.str().c_str());

    // Find the first file in the directory.
    WIN32_FIND_DATA ffd;
    HANDLE hFind = FindFirstFile(szDir, &ffd);
    if (hFind == INVALID_HANDLE_VALUE) {
      cout << "GetFilesInDirectory error getting dir info. Check that ";
      cout << "directory is not empty!" << endl;
      return 0;
    }

    do {
      if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
      } else {
        std::string cur_filename = ffd.cFileName;
        char* name = new char[cur_filename.length() + 1];
        strcpy(name, cur_filename.c_str());
        files.push_back(Triple<char*, int64_t, int64_t>(name, -1, -1));
      }
    } while (FindNextFile(hFind, &ffd) != 0);
    FindClose(hFind);

#else
    throw std::wruntime_error("Apple version needs updating!");
#endif
    // Now Parse the filenames and get their unique ID number
    for (uint32_t i = 0; i < files.size(); i++) {
      std::string str = files.at(i).first;
      size_t i0 = str.find_first_of("0123456789");  // Database hash start
      size_t i1 = (uint32_t)str.find_first_not_of("0123456789", i0);  // Database hash end
      size_t i2 = str.find_first_of("0123456789", i1+1);  // Kinect Number
      size_t i3 = str.find_first_of("0123456789", i2+1);  // Frame time start
      size_t i4 = (uint32_t)str.find_first_not_of("0123456789", i3);  // Frame time end

      if (i0 == string::npos || i1 == string::npos || i2 == string::npos || 
        i3 == string::npos || i4 == string::npos) {
        throw std::wruntime_error("DepthImagesIO::GetFilesInDirectory() - "
          "ERROR: Couldn't parse filename!");
      }
      std::string database_str = str.substr(i0, i1-i0);
      std::string frametime_str = str.substr(i3, i4-i3);

      int64_t database_num = jtil::string_util::Str2Num<int64_t>(database_str);
      int64_t frametime_num = jtil::string_util::Str2Num<int64_t>(frametime_str);
      files.at(i).second = database_num;
      files.at(i).third = frametime_num;
    }

    // Now sort the files by their unique id
    // Third argument is an inline lambda expression (comparison function)
    std::cout << "sorting image filenames by timestamp..." << std::endl;

    // Now sort by both dataset and frame number
    std::sort(files.begin(), files.end(), [](Triple<char*, int64_t, int64_t>& a,
      Triple<char*, int64_t, int64_t>& b) { return b.second > a.second ? true : b.third > a.third; });

    // Now copy them into the output array
    files_in_directory.capacity((uint32_t)files.size());
    for (uint32_t i = 0; i < files.size(); i++) {
      files_in_directory.pushBack(files.at(i));
    }

    return files_in_directory.size();
  };

using std::cout;
using std::endl;
Float4x4 mat_tmp;
WindowSettings settings;

int main(int argc, char *argv[]) {
  static_cast<void>(argc); static_cast<void>(argv);
#if defined(_DEBUG) && defined(_WIN32)
  jtil::debug::EnableMemoryLeakChecks();
  // jtil::debug::EnableAggressiveMemoryLeakChecks();
  // jtil::debug::SetBreakPointOnAlocation(3734);
#endif

  cout << "Usage:" << endl;
  cout << "p - Cycle through the frames continuously" << endl;
  cout << "+/- - Forward and back a frame" << endl;
  cout << "9/0 - Forward and back 100 frames" << endl;
  cout << "d - Delete frame (when loading processed images)" << endl;
  cout << "q/ESC - Quit" << endl;
  
  try {
    tp = new ThreadPool(NUM_WORKER_THREADS);

    clk = new jtil::clk::Clk();
    t1 = clk->getTime();
    t0 = t1;
    
    // Initialize Windowing system
    Window::initWindowSystem();
    Texture::initTextureSystem();

    jtorch::InitJTorch("../jtorch");

    // Fill the settings structure
    settings.width = 640 * 2;
    settings.height = 640;
    settings.fullscreen = false;
    settings.title = string("Hand Fit Project");
    settings.gl_major_version = 3;
    settings.gl_minor_version = 2;
    settings.num_depth_bits = 24;
    settings.num_stencil_bits = 0;
    settings.num_rgba_bits = 8;
    
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
#ifdef LOAD_PROCESSED_IMAGES
    GetProcessedFilesInDirectory(im_files, DST_IM_DIR, 0);
#else
    image_io->GetFilesInDirectory(im_files, IM_DIR, 0, NULL);
#endif

    // Attach callback functions for event handling
    wnd->registerKeyboardCB(keyboardCB);
    wnd->registerMousePosCB(NULL);
    wnd->registerMouseButtonCB(NULL);
    wnd->registerMouseWheelCB(NULL);
    wnd->registerCharacterInputCB(NULL);
    
    // Create the hand data and attach it to the renderer for lighting
    if (num_hands != 1) {
      throw std::runtime_error("ERROR: This main routine only works with one "
        "hand (left or right)!");
    }
    hand_renderer = new ModelRenderer(1);
    r_hand = new HandModelCoeff(HandType::RIGHT);
    l_hand = new HandModelCoeff(HandType::LEFT);

    try {
      LoadArrayFromFile<float>(camera_view.m, 16, IM_DIR + 
        "calibration_data0.bin");
    } catch (std::wruntime_error e) {
      std::cout << "WARNING: calibration_data0.bin doesn't exist.  ";
      std::cout << "Identity camera matrix." << std::endl;
    }
    Float4x4 old_view, camera_view_inv, cur_view;
    old_view.set(*hand_renderer->camera(0)->view());
    Float4x4::inverse(camera_view_inv, camera_view);
    Float4x4::mult(cur_view, old_view, camera_view_inv);
    hand_renderer->camera(0)->view()->set(cur_view);
    hand_renderer->camera(0)->set_view_mat_directly = true;

    std::cout << "Using a cropped source image of " << HN_SRC_IM_SIZE;
    std::cout << std::endl << "Final image size after processing is ";
    std::cout << (HN_IM_SIZE) << std::endl;

    hand_detect = new HandDetector(tp);
    hand_detect->init(src_width, src_height, KINECT_HANDS_ROOT +
      FOREST_DATA_FILENAME);
    hand_image_generator_ = new HandImageGenerator(HN_DEFAULT_NUM_CONV_BANKS);
    for (uint32_t i = 0; i < HandCoeffConvnet::HAND_NUM_COEFF_CONVNET; i++) {
      blank_coeff[i] = 0.0f;
    }

    loadCurrentImage();
    tex = new Texture(GL_RGB8, 2 * HN_IM_SIZE, HN_IM_SIZE, GL_RGB, 
      GL_UNSIGNED_BYTE, (unsigned char*)tex_data, 
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
        if (cur_image < (int32_t)im_files.size() - 1) {
          cur_image++;
          loadCurrentImage();
        } else {
          continuous_playback = false;
        }
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
