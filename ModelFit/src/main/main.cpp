//
//  main.cpp
//
//  A quick test of some mesh triangulation and deformation techniques
//

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
#include "renderer/geometry/geometry_colored_lines.h"
#include "renderer/gl_state.h"
#include "windowing/window.h"
#include "windowing/window_settings.h"
#include "model_fit/calibrate_geometry.h"
#include "model_fit/hand_geometry_mesh.h"
#include "model_fit/model_renderer.h"
#include "model_fit/model_fit.h"
#include "kinect_interface/hand_net/hand_model_coeff.h"
#include "kinect_interface/depth_images_io.h"
#include "kinect_interface/open_ni_funcs.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"
#include "kinect_interface/hand_detector/hand_detector.h"
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector.h"
#include "jtil/clk/clk.h"
#include "jtil/file_io/file_io.h"
#include "jtil/string_util/string_util.h"
#include "jtil/threading/thread_pool.h"
#include "jtil/math/icp.h"
#include "jtil/image_util/image_util.h"

#if defined(WIN32) || defined(_WIN32)  
  #define snprintf _snprintf_s
  #pragma warning( disable : 4099 )
#endif
#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

// CALIBRATION MODE ONLY SETTINGS:
// #define CALIBRATION_RUN
#define FILTER_SIZE 60  
#define CALIBRATION_MAX_FILES 100
#define ICP_PC_MODEL_DIST_THRESH 15  // mm
#define ICP_USE_POINTS_NEAR_MODEL false

#define PERFORM_ICP_FIT
#define USE_ICP_NORMALS
#define ICP_NUM_ITERATIONS 100
#define ICP_METHOD ICPMethod::BFGS_ICP
#define ICP_COS_NORM_THRESHOLD acosf((35.0f / 360.0f) * 2.0f * (float)M_PI);
#define ICP_MIN_DISTANCE_SQ 1.0f
#define ICP_MAX_DISTANCE_SQ 1600.0f  // 4cm ^ 2 = 40mm ^ 2
#define MAX_ICP_PTS 100000

// KINECT DATA
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_1/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_2_1/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_2_2/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_3/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_4/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_5/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_6/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_7/")  // Fit (Tr-data)

// PRIMESENSE DATA
//#define IM_DIR_BASE string("data/hand_depth_data/")
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_01_1/")  // Cal + Fit + Proc (5405)
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_03_1/")  // Cal + Fit + Proc (6533)
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_06_1/")  // Cal + Fit + Proc (8709)
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_06_2/")  // Cal + Fit + Proc (8469)
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_06_3/")  // Cal + Fit + Proc (5815)  Total: 34931 
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_08_1/")  // Cal (Tr-data) + Fit + Proc (2440)
#define IM_DIR_BASE string("data/hand_depth_data_2013_05_19_1/")  // Cal
//#define IM_DIR_BASE string("data/hand_depth_data_2013_05_19_2/")

//#define KINECT_DATA  // Otherwise Primesense 1.09 data
#define MAX_KINECTS 3
#define NUM_WORKER_THREADS 6

#if defined(__APPLE__)
  #define KINECT_HANDS_ROOT string("./../../../../../../../../../../")
#else
  #define KINECT_HANDS_ROOT string("./../")
#endif

#ifndef HAND_FIT
  #error "HAND_FIT is not defined in the preprocessor definitions!"
#endif

#define IM_DIR (KINECT_HANDS_ROOT + IM_DIR_BASE)
// #define LOAD_AND_SAVE_OLD_FORMAT_COEFFS
const bool fit_left = false;
const bool fit_right = true; 

using namespace std;
using namespace jtil::math;
using namespace jtil::data_str;
using namespace jtil::file_io;
using namespace jtil::image_util;
using namespace jtil::threading;
using namespace model_fit;
using namespace renderer;
using namespace windowing;
using namespace kinect_interface;
using namespace kinect_interface::hand_net;
using namespace kinect_interface::hand_detector;

jtil::clk::Clk* clk = NULL;
double t1, t0;

// The main window and basic rendering system
Window* wnd = NULL;
Renderer* render = NULL;
bool rotate_light = false;
int render_output = 1;  // 1 - color, 
                        // 2 - synthetic depth, 

// Camera class for handling view and proj matrices
const float mouse_speed_rotation = 0.005f;
const float camera_speed = 300.0f;
const float camera_run_mulitiplier = 10.0f;
Float3 cur_dir(0.0f, 0.0f, 0.0f);
Float3 delta_pos;
int mouse_x, mouse_y, mouse_x_prev, mouse_y_prev;
bool camera_rotate = false;
bool scale_coeff = false;
bool middle_down = false;
bool running = false;
bool shift_down = false;

// model
Float4x4 camera_view[MAX_KINECTS];
PoseModel** models;
int cur_kinect = 0;
int cur_icp_dst_kinect = 0;
uint32_t cur_icp_mat = 0;
int32_t last_icp_kinect = -1;
bool render_correspondances = true;
#if defined(CALIBRATION_RUN)
  CalibrateGeometryType cal_type = CalibrateGeometryType::ICOSAHEDRON; 
  const float max_icp_dist = GDT_MAX_DIST;
  const uint32_t num_models = 1;
  float** coeffs[MAX_KINECTS] = {NULL, NULL};  // coeffs[kinect][frame][coeff]
  const uint32_t num_coeff = CalibrateCoeff::NUM_PARAMETERS;
  const uint32_t num_coeff_fit = CAL_GEOM_NUM_COEFF;
  bool render_all_views = 0;
  int16_t** depth_database[MAX_KINECTS] = {NULL, NULL};  // depth[kinect][frame][pix]
  uint8_t** rgb_database[MAX_KINECTS] = {NULL, NULL};
  uint8_t** label_database[MAX_KINECTS] = {NULL, NULL};
  uint32_t num_model_fit_cameras = 1;
#else
  HandModelCoeff** l_hand_coeffs = NULL;  // Left Hand coefficients
  HandModelCoeff** r_hand_coeffs = NULL;  // Right hand coeffs
  const uint32_t num_models = (fit_left ? 1 : 0) + (fit_right ? 1 : 0);
  int hand_to_modify = fit_left ? 0 : 1;
  const uint32_t num_coeff = HandCoeff::NUM_PARAMETERS;
  const uint32_t num_coeff_fit = HAND_NUM_COEFF;
  uint32_t num_model_fit_cameras = MAX_KINECTS;
#endif
uint32_t cur_coeff = 0;
ModelFit* fit = NULL;
bool continuous_fit = false;  // fit frames continuously each frame
bool continuous_play = false;  // Play back recorded frames
bool render_models = true;
uint32_t coeff_src = 0;
float** coeff = NULL;  // Temp space only used when performing fit
float** prev_coeff = NULL; 

// Kinect Image data 
DepthImagesIO* image_io = NULL;
Vector<Triple<char*, int64_t, int64_t>> im_files[MAX_KINECTS];  // filename, kinect time, global time
float cur_xyz_data[MAX_KINECTS][src_dim*3];
float cur_norm_data[MAX_KINECTS][src_dim*3];
float cur_uvd_data[MAX_KINECTS][src_dim*3];
int16_t** cur_depth_data;  // Size: [MAX_KINECTS][src_dim*3]
uint8_t** cur_label_data;  // Size: [MAX_KINECTS][src_dim]
uint8_t cur_image_rgb[MAX_KINECTS][src_dim*3];
uint32_t cur_image = 0;
GeometryColoredPoints* geometry_points[MAX_KINECTS];
GeometryColoredLines* geometry_lines[MAX_KINECTS-1];  // For displaying ICP correspondances
float temp_xyz[3 * src_dim];
float temp_rgb[3 * src_dim];
bool render_depth = true;
int playback_step = 1;
OpenNIFuncs openni_funcs;
Texture* tex = NULL;
uint8_t tex_data[src_dim * 3];

// Decision forests
HandDetector* hand_detect = NULL;

// ICP
jtil::math::ICP icp;

// Multithreading
ThreadPool* tp;

void quit() {
  tp->stop();
  delete tp;
  for (uint32_t i = 0; i < num_models; i++) {
    models[i]->setRendererAttachement(true);  // Make sure Geometry manager deletes models
    SAFE_DELETE(models[i]);
  }
  SAFE_DELETE(hand_detect);
  SAFE_DELETE(tex);
  SAFE_DELETE_ARR(models);
  SAFE_DELETE_ARR(coeff);
  SAFE_DELETE_ARR(prev_coeff);
  SAFE_DELETE(image_io);
  SAFE_DELETE(clk);
  if (cur_depth_data) {
    for (uint32_t i = 0; i < MAX_KINECTS; i++) {
      SAFE_DELETE_ARR(cur_depth_data[i]);
    }
  }
  SAFE_DELETE_ARR(cur_depth_data);
  if (cur_label_data) {
    for (uint32_t i = 0; i < MAX_KINECTS; i++) {
      SAFE_DELETE_ARR(cur_label_data[i]);
    }
  }
  SAFE_DELETE_ARR(cur_label_data);
#if defined(CALIBRATION_RUN)
  for (uint32_t i = 0; i < MAX_KINECTS; i++) {
    if (coeffs[i]) {
      for (uint32_t j = 0; j < im_files[0].size(); j++) {
        SAFE_DELETE_ARR(coeffs[i][j]);
      }
      SAFE_DELETE_ARR(coeffs[i]);
    }
  }
  for (uint32_t k = 0; k < MAX_KINECTS; k++) {
    if (depth_database[k]) {
      for (uint32_t f = 0; f < im_files[k].size(); f++) {
        SAFE_DELETE_ARR(depth_database[k][f]);
      }
      SAFE_DELETE_ARR(depth_database[k]);
    }
    if (rgb_database[k]) {
      for (uint32_t f = 0; f < im_files[k].size(); f++) {
        SAFE_DELETE_ARR(rgb_database[k][f]);
      }
      SAFE_DELETE_ARR(rgb_database[k]);
    }
    if (label_database[k]) {
      for (uint32_t f = 0; f < im_files[k].size(); f++) {
        SAFE_DELETE_ARR(label_database[k][f]);
      }
      SAFE_DELETE_ARR(label_database[k]);
    }
  }

#else
  if (l_hand_coeffs) {
    for (uint32_t i = 0; i < im_files[0].size(); i++) { 
      delete l_hand_coeffs[i];
    }
    delete[] l_hand_coeffs;
  }
  if (r_hand_coeffs) {
    for (uint32_t i = 0; i < im_files[0].size(); i++) {
      delete r_hand_coeffs[i];
    }
    delete[] r_hand_coeffs;
  }
#endif
  SAFE_DELETE(render);
  SAFE_DELETE(fit);
  for (uint32_t k = 0; k < MAX_KINECTS; k++) {
    SAFE_DELETE(geometry_points[k]);
    for (uint32_t i = 0; i < im_files[k].size(); i++) {
      SAFE_DELETE_ARR(im_files[k][i].first);
    }
  }
  Texture::shutdownTextureSystem();
  GLState::shutdownGLState();
  SAFE_DELETE(wnd);
  Window::killWindowSystem();
  exit(0);
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

void loadCurrentImage(bool print_to_screen = true) {
  char* file = im_files[0][cur_image].first;
  string full_filename = IM_DIR + string(file);
  if (print_to_screen) {
    std::cout << "loading image: " << full_filename << std::endl;
    std::cout << "cur_image = " << cur_image << " of ";
    std::cout << im_files[0].size() << std::endl;
  }
#if defined(CALIBRATION_RUN)
  // Average the non-zero pixels over some temporal filter kernel
  int32_t start_index[MAX_KINECTS];
  int16_t cur_depth[FILTER_SIZE];
  for (int32_t k = 0; k < MAX_KINECTS; k++) {
    if (im_files[k].size() == 0) {
      start_index[k] = MAX_UINT32;
    } else {
      // find the correct file (with smallest timestamp difference) - 
      // Search in a window of radius 30 images
      start_index[k] = findClosestFrame(k);
    }
  }
  for (int32_t k = 0; k < MAX_KINECTS; k++) {
    for (int32_t i = 0; i < src_dim; i++) {
      int32_t filt = 0;
      for (int32_t f = start_index[k]; f < start_index[k] + FILTER_SIZE && 
        f < (int32_t)im_files[k].size(); f++, filt++) {
        cur_depth[filt] = depth_database[k][f][i];
      }
      // Now calculate the std and mean of the non-zero entries
      for ( ; filt < FILTER_SIZE; filt++) {
        cur_depth[filt] = GDT_MAX_DIST + 1;
      }
      float sum = 0;
      float sum_sqs = 0;
      float cnt = 0;
      for (int32_t filt = 0; filt < FILTER_SIZE; filt++) {
        if (cur_depth[filt] != 0 && cur_depth[filt] < GDT_MAX_DIST) {
          sum += (float)cur_depth[filt];
          sum_sqs += (float)cur_depth[filt] * (float)cur_depth[filt];
          cnt++;
        }
      }
      if (cnt < LOOSE_EPSILON) {
        cur_depth_data[k][i] = GDT_MAX_DIST + 1;
      } else {
        float mean = sum / cnt;
        float var = sum_sqs / cnt - (mean * mean);
        if (var > 100) {
          // Calculate a new mean of all the depth values IN FRONT of the mean
          sum = 0;
          cnt = 0;
          for (int32_t filt = 0; filt < FILTER_SIZE; filt++) {
            if (cur_depth[filt] < mean && cur_depth[filt] != 0) {
              sum += (float)cur_depth[filt];
              cnt++;
            }
          }
          mean = sum / cnt;
        }
        cur_depth_data[k][i] = (int16_t)mean;
      }
    }
    memcpy(cur_image_rgb[k], rgb_database[k][start_index[k]], 
      sizeof(cur_image_rgb[k][0]) * src_dim * 3);
    memset(cur_label_data[k], 0, sizeof(cur_label_data[k][0]) * src_dim);

#ifdef KINECT_DATA
      DepthImagesIO::convertSingleImageToXYZ(cur_xyz_data[k], cur_depth_data[k]);
#else
      openni_funcs.ConvertDepthImageToProjective((uint16_t*)cur_depth_data[k], 
        cur_uvd_data[k]);
      openni_funcs.convertDepthToWorldCoordinates(cur_uvd_data[k], cur_xyz_data[k], 
        src_dim);
#endif
  }
#else
  // Now load the other Kinect data
  for (uint32_t k = 0; k < MAX_KINECTS; k++) {
    if (im_files[k].size() == 0) {
      for (uint32_t j = 0; j < src_dim; j++) {
        cur_depth_data[k][j] = GDT_MAX_DIST;
      }
      memset(cur_label_data[k], 0, sizeof(cur_label_data[k][0]) * src_dim); 
      memset(cur_image_rgb[k], 0, sizeof(cur_image_rgb[k][0]) * src_dim * 3); 
    } else {
      // find the correct file (with smallest timestamp difference) - O(60)
      uint32_t i_match = findClosestFrame(k);
      // Now we've found the correct file, load it
      file = im_files[k][i_match].first;
      full_filename = IM_DIR + string(file);
      std::cout << "loading additional image: " << full_filename << std::endl;
      image_io->LoadCompressedImage(full_filename, 
        cur_depth_data[k], cur_label_data[k], cur_image_rgb[k]);
      memset(cur_label_data[k], 0, src_dim * sizeof(cur_label_data[k][0]));
#ifdef KINECT_DATA
      DepthImagesIO::convertSingleImageToXYZ(cur_xyz_data[k], cur_depth_data[k]);
#else
      openni_funcs.ConvertDepthImageToProjective((uint16_t*)cur_depth_data[k], 
        cur_uvd_data[k]);
      openni_funcs.convertDepthToWorldCoordinates(cur_uvd_data[k], cur_xyz_data[k], 
        src_dim);
#endif
    }
  }
#endif
}

void InitXYZPointsForRendering() {
  for (uint32_t k = 0; k < MAX_KINECTS; k++) {
    if (geometry_points[k]->synced()) {
      geometry_points[k]->unsyncVAO();
    }

    jtil::data_str::Vector<jtil::math::Float3>* vert = geometry_points[k]->vertices();
    jtil::data_str::Vector<jtil::math::Float3>* cols = geometry_points[k]->colors();

    float cur_col[3];
    for (uint32_t i = 0; i < src_dim; i++) {
      vert->at(i)->set(&cur_xyz_data[k][i*3]);
      if (cur_label_data[k][i] == 0) {
        if (k != 2) {
          cur_col[0] = 1.0f * static_cast<float>(cur_image_rgb[k][i*3]) / 255.0f;
        } else {
           cur_col[0] = 0.1f * static_cast<float>(cur_image_rgb[k][i*3]) / 255.0f;
        }
        cur_col[1] = 1.0f * static_cast<float>(cur_image_rgb[k][i*3+1]) / 255.0f;
        if (k != 1) {
          cur_col[2] = 1.0f * static_cast<float>(cur_image_rgb[k][i*3+2]) / 255.0f;
        } else {
          cur_col[2] = 0.1f * static_cast<float>(cur_image_rgb[k][i*3+2]) / 255.0f;
        }
      } else {
        cur_col[0] = std::max<float>(0.0f, (float)(cur_image_rgb[k][i*3]) / 255.0f - 0.1f);
        cur_col[1] = std::min<float>((float)(cur_image_rgb[k][i*3+1]) / 255.0f + 0.4f, 255.0f);
        cur_col[2] = std::max<float>(0.0f, (float)(cur_image_rgb[k][i*3+2]) / 255.0f - 0.1f);
      }
      cols->at(i)->set(cur_col);
    }
    
    geometry_points[k]->syncVAO();
  }
}


void saveCurrentCoeffs() {
#if defined(CALIBRATION_RUN)
  // Save both kinect coeffs in the same file
  string filename = IM_DIR + string("coeff_") + im_files[0][cur_image].first;
  float calb_data[MAX_KINECTS * num_coeff];
  for (uint32_t i = 0; i < MAX_KINECTS; i++) {
    memcpy(&calb_data[i * num_coeff], coeffs[i][cur_image], 
      sizeof(*calb_data) * num_coeff);
  }
  SaveArrayToFile<float>(calb_data, MAX_KINECTS * num_coeff, filename);
  cout << "hand data saved to file" << endl;
#else
  string r_hand_file = string("coeffr_") + im_files[0][cur_image].first;
  string l_hand_file = string("coeffl_") + im_files[0][cur_image].first;
  if (fit_right) {
    r_hand_coeffs[cur_image]->saveToFile(IM_DIR, r_hand_file);
  } else {
    r_hand_coeffs[cur_image]->saveBlankFile(IM_DIR, r_hand_file);
  }
  if (fit_left) {
    l_hand_coeffs[cur_image]->saveToFile(IM_DIR, l_hand_file);
  } else {
    l_hand_coeffs[cur_image]->saveBlankFile(IM_DIR, l_hand_file);
  }
  cout << "hand data saved to file" << endl;
#endif
}

void MouseButtonCB(int button, int action) {
  if (button == MOUSE_BUTTON_LEFT) {
    if (action == PRESSED) {
      camera_rotate = true;
    }
    if (action == RELEASED) {
      camera_rotate = false;
    }
  } else if (button == MOUSE_BUTTON_RIGHT) {
    if (action == PRESSED) {
      scale_coeff = true;
    }
    if (action == RELEASED) {
      scale_coeff = false;
    }
  } else if (button == MOUSE_BUTTON_MIDDLE) {
    if (action == PRESSED) {
      middle_down = true;
    }
    if (action == RELEASED) {
      middle_down = false;
    }
  }
}

void MousePosCB(int x, int y) {
  mouse_x_prev = mouse_x;
  mouse_y_prev = mouse_y;
  mouse_x = x;
  mouse_y = y;
  if (camera_rotate) {
    int dx = mouse_x - mouse_x_prev;
    int dy = mouse_y - mouse_y_prev;
    float theta_x = dx * mouse_speed_rotation;
    float theta_y = dy * mouse_speed_rotation;
    render->camera()->rotateCamera(theta_x, theta_y);
  }
  if (scale_coeff) {
    int dy = mouse_y - mouse_y_prev;
    float theta_y = dy * mouse_speed_rotation;
    if (/*cur_coeff >= 0 &&*/ cur_coeff <= 2) {
      theta_y *= 50.0f;
    }
#if defined(CALIBRATION_RUN)
    if (shift_down) {
      if (cal_type == BOX) {
        Float3& box_size = ((CalibrateGeometry*)models[0])->box_size();
        box_size[cur_coeff % 3] -= theta_y;
        ((CalibrateGeometry*)models[0])->updateSize();
        cout << "cur_coeff " << cur_coeff % 3;
        cout << " --> " << box_size[cur_coeff % 3] << endl;
      } else if (cal_type == ICOSAHEDRON) {
        ((CalibrateGeometry*)models[0])->icosahedron_scale() -= 0.001f * theta_y;
        ((CalibrateGeometry*)models[0])->updateSize();
        cout << "icosahedron_scale --> " << 
          ((CalibrateGeometry*)models[0])->icosahedron_scale() << endl;
      }
    } else {
      coeffs[cur_kinect][cur_image][cur_coeff] -= theta_y;
      cout << "cur_coeff " << cur_coeff;
      cout << " --> " << coeffs[cur_kinect][cur_image][cur_coeff] << endl;
    }
#else
    float coeff_val;
    if (cur_coeff == HandCoeff::SCALE) {
      // We must set both scales at once
      coeff_val = l_hand_coeffs[cur_image]->getCoeff(cur_coeff);
      l_hand_coeffs[cur_image]->setCoeff(cur_coeff, coeff_val - theta_y);
      r_hand_coeffs[cur_image]->setCoeff(cur_coeff, coeff_val - theta_y);
    } else if (hand_to_modify == 0) {
      coeff_val = l_hand_coeffs[cur_image]->getCoeff(cur_coeff);
      l_hand_coeffs[cur_image]->setCoeff(cur_coeff, coeff_val - theta_y);
    } else {
      coeff_val = r_hand_coeffs[cur_image]->getCoeff(cur_coeff);
      r_hand_coeffs[cur_image]->setCoeff(cur_coeff, coeff_val - theta_y);
    }
    cout << "cur_coeff " << kinect_interface::hand_net::HandCoeffToString(cur_coeff);
    cout << " --> " << coeff_val - theta_y << endl;
#endif
  }
  if (middle_down) {
    int dy = mouse_y - mouse_y_prev;
    float theta_y = dy * mouse_speed_rotation;
    Float3 trans(0, 0, 0);
    trans[cur_coeff % 3] -= theta_y;
    camera_view[cur_kinect].leftMultTranslation(trans);

    // Now save the results to file
    std::stringstream ss;
    ss << IM_DIR << "calibration_data" << cur_kinect << ".bin";
    SaveArrayToFile<float>(camera_view[cur_kinect].m, 16, ss.str());
    std::cout << "Calibration data saved to " << ss.str() << endl;
  }
}

void renderFrame(float dt);
void fitFrame(bool seed_with_last_frame, bool query_only);
double continuous_fit_timer_start;
double continuous_play_timer_start;
const double continuous_play_frame_time = 1.0/15.0;
string full_im_filename;
string r_coeff_file;
string l_coeff_file;
string new_full_im_filename;
string new_r_coeff_file;
string new_l_coeff_file;
void KeyboardCB(int key, int action) {
  int repeat = 1;

  switch (key) {
    case KEY_LSHIFT:
      if (action == PRESSED) {
        running = true;
        shift_down = true;
      } else {
        running = false;
        shift_down = false;
      }
      break;
    case KEY_ESC:
      if (action == RELEASED) {
        quit();
      }
      break;
    case static_cast<int>('w'):
    case static_cast<int>('W'):
      if (action == PRESSED) {
        cur_dir[2] += 1;
      } else {
        cur_dir[2] -= 1;
      }
      break;
    case static_cast<int>('s'):
    case static_cast<int>('S'):
      if (action == PRESSED) {
        cur_dir[2] -= 1;
      } else {
        cur_dir[2] += 1;
      }
      break;
    case static_cast<int>('a'):
    case static_cast<int>('A'):
      if (action == PRESSED) {
        cur_dir[0] += 1;
      } else {
        cur_dir[0] -= 1;
      }
      break;
    case static_cast<int>('d'):
    case static_cast<int>('D'):
      if (action == PRESSED) {
        cur_dir[0] -= 1;
      } else {
        cur_dir[0] += 1;
      }
      break;
    case static_cast<int>('q'):
    case static_cast<int>('Q'):
      if (action == PRESSED) {
        cur_dir[1] += 1;
      } else {
        cur_dir[1] -= 1;
      }
      break;
    case static_cast<int>('e'):
    case static_cast<int>('E'):
      if (action == PRESSED) {
        cur_dir[1] -= 1;
      } else {
        cur_dir[1] += 1;
      }
      break;
    case static_cast<int>('r'):
    case static_cast<int>('R'):
      if (action == RELEASED) {
        rotate_light = !rotate_light;
      }
      break;
    case static_cast<int>('t'):
    case static_cast<int>('T'):
      if (action == RELEASED) {
        render->wireframe = !render->wireframe;
      }
      break;
    case static_cast<int>('b'):
    case static_cast<int>('B'):
      if (action == RELEASED) {
        render->render_bounding_spheres = !render->render_bounding_spheres;
      }
      break;
    case static_cast<int>('1'):
    case static_cast<int>('2'):
    case static_cast<int>('3'):
    case static_cast<int>('4'):
    case static_cast<int>('5'):
    case static_cast<int>('6'):
    case static_cast<int>('7'):
      if (action == RELEASED && !shift_down) {
        render_output = key - static_cast<int>('1') + 1;
      }
#ifndef CALIBRATION_RUN
      if (action == RELEASED && shift_down && cur_image > 0) {
        switch (key) {
          case static_cast<int>('5'):
            std::cout << "copying thumb from previous frame..." << std::endl;
            if (fit_left) {
              l_hand_coeffs[cur_image]->setCoeff(HandCoeff::THUMB_THETA, 
                l_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::THUMB_THETA));
              l_hand_coeffs[cur_image]->setCoeff(HandCoeff::THUMB_PHI, 
                l_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::THUMB_PHI));
              l_hand_coeffs[cur_image]->setCoeff(HandCoeff::THUMB_K1_PHI, 
                l_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::THUMB_K1_PHI));
              l_hand_coeffs[cur_image]->setCoeff(HandCoeff::THUMB_K1_THETA, 
                l_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::THUMB_K1_THETA));
              l_hand_coeffs[cur_image]->setCoeff(HandCoeff::THUMB_K2_PHI, 
                l_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::THUMB_K2_PHI));
            }
            if (fit_right) {
              r_hand_coeffs[cur_image]->setCoeff(HandCoeff::THUMB_THETA, 
                r_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::THUMB_THETA));
              r_hand_coeffs[cur_image]->setCoeff(HandCoeff::THUMB_PHI, 
                r_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::THUMB_PHI));
              r_hand_coeffs[cur_image]->setCoeff(HandCoeff::THUMB_K1_PHI, 
                r_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::THUMB_K1_PHI));
              r_hand_coeffs[cur_image]->setCoeff(HandCoeff::THUMB_K1_THETA, 
                r_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::THUMB_K1_THETA));
              r_hand_coeffs[cur_image]->setCoeff(HandCoeff::THUMB_K2_PHI, 
                r_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::THUMB_K2_PHI));
            }
            break;
          case static_cast<int>('1'):
          case static_cast<int>('2'):
          case static_cast<int>('3'):
          case static_cast<int>('4'):
            uint32_t finger = (int)key - (int)'1';
            uint32_t off = FINGER_NUM_COEFF * finger;
            std::cout << "copying finger " << finger;
            std::cout << " from previous frame..." << std::endl;
            if (fit_left) {
              l_hand_coeffs[cur_image]->setCoeff(HandCoeff::F0_ROOT_PHI + off, 
                l_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::F0_ROOT_PHI + off));
              l_hand_coeffs[cur_image]->setCoeff(HandCoeff::F0_ROOT_THETA + off, 
                l_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::F0_ROOT_THETA + off));
              l_hand_coeffs[cur_image]->setCoeff(HandCoeff::F0_PHI + off, 
                l_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::F0_PHI + off));
              l_hand_coeffs[cur_image]->setCoeff(HandCoeff::F0_THETA + off, 
                l_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::F0_THETA + off));
              l_hand_coeffs[cur_image]->setCoeff(HandCoeff::F0_KNUCKLE_MID + off, 
                l_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::F0_KNUCKLE_MID + off));
              l_hand_coeffs[cur_image]->setCoeff(HandCoeff::F0_KNUCKLE_END + off, 
                l_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::F0_KNUCKLE_END + off));
            }
            if (fit_right) {
              r_hand_coeffs[cur_image]->setCoeff(HandCoeff::F0_ROOT_PHI + off, 
                r_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::F0_ROOT_PHI + off));
              r_hand_coeffs[cur_image]->setCoeff(HandCoeff::F0_ROOT_THETA + off, 
                r_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::F0_ROOT_THETA + off));
              r_hand_coeffs[cur_image]->setCoeff(HandCoeff::F0_PHI + off, 
                r_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::F0_PHI + off));
              r_hand_coeffs[cur_image]->setCoeff(HandCoeff::F0_THETA + off, 
                r_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::F0_THETA + off));
              r_hand_coeffs[cur_image]->setCoeff(HandCoeff::F0_KNUCKLE_MID + off, 
                r_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::F0_KNUCKLE_MID + off));
              r_hand_coeffs[cur_image]->setCoeff(HandCoeff::F0_KNUCKLE_END + off, 
                r_hand_coeffs[cur_image-1]->getCoeff(HandCoeff::F0_KNUCKLE_END + off));
            }
            break;
        }
        saveCurrentCoeffs();
      }
#endif
      break;
    case KEY_KP_ADD:
      if (action == RELEASED) {
        cur_image = cur_image < static_cast<uint32_t>(im_files[0].size())-1 ? 
          cur_image+1 : cur_image;
        loadCurrentImage();
        InitXYZPointsForRendering();
      }
      break;
    case KEY_KP_SUBTRACT:
      if (action == RELEASED) {
        cur_image = cur_image > 0 ? cur_image-1 : 0;
        loadCurrentImage();
        InitXYZPointsForRendering();
      }
      break;
    case static_cast<int>('0'):
      if (action == RELEASED) {
        cur_image = cur_image + 100;
        if (cur_image >= im_files[0].size()) {
          cur_image = im_files[0].size()-1;
        }
        loadCurrentImage();
        InitXYZPointsForRendering();
      }
      break;
    case static_cast<int>('9'):
      if (action == RELEASED) {
        cur_image = cur_image >= 100 ? cur_image-100 : 0;
        loadCurrentImage();
        InitXYZPointsForRendering();
      }
      break;
    case static_cast<int>('}'):
    case static_cast<int>(']'):
      if (action == RELEASED) {
        cur_coeff = (cur_coeff + 1) % num_coeff;
#ifdef CALIBRATION_RUN
        cout << "cur_coeff = " << cur_coeff << std::endl;
#else
        cout << "cur_coeff = " << kinect_interface::hand_net::HandCoeffToString(cur_coeff); 
        if (hand_to_modify == 0) {
          cout << " = " << l_hand_coeffs[cur_image]->getCoeff(cur_coeff);
        } else {
          cout << " = " << r_hand_coeffs[cur_image]->getCoeff(cur_coeff);
        }
        cout << std::endl;
#endif
      }
      break;
    case static_cast<int>('{'):
    case static_cast<int>('['):
      if (action == RELEASED) {
        cur_coeff = cur_coeff != 0 ? (cur_coeff - 1) : num_coeff - 1;
#ifdef CALIBRATION_RUN
        cout << "cur_coeff = " << cur_coeff << std::endl;
#else
        cout << "cur_coeff = " << kinect_interface::hand_net::HandCoeffToString(cur_coeff); 
        if (hand_to_modify == 0) {
          cout << " = " << l_hand_coeffs[cur_image]->getCoeff(cur_coeff);
        } else {
          cout << " = " << r_hand_coeffs[cur_image]->getCoeff(cur_coeff);
        }
        cout << std::endl;
#endif
      }
      break;
    case static_cast<int>('f'):
    case static_cast<int>('F'):
      if (action == RELEASED) {
        cout << "Fitting model..." << endl;
        fitFrame(false, false);
      }
      break;
    case static_cast<int>('g'):
    case static_cast<int>('G'):
      if (action == RELEASED) {
        continuous_fit = !continuous_play && !continuous_fit;
        if (continuous_fit) {
          fit->resetFuncEvalCount();
          continuous_fit_timer_start = clk->getTime();
        }
      }
      break;
    case static_cast<int>('p'):
    case static_cast<int>('P'):
      if (action == RELEASED) {
        continuous_play = !continuous_fit && !continuous_play;
      }
      break;
    case static_cast<int>('l'):
    case static_cast<int>('L'):
      if (action == RELEASED) {
        cur_image = 0;
        loadCurrentImage();
        InitXYZPointsForRendering();
      }
      break;
    case static_cast<int>('o'):
    case static_cast<int>('O'): 
      if (action == RELEASED) {
        playback_step = playback_step + 1;
        playback_step = playback_step == 11 ? 1 : playback_step;
        cout << "playback_step = " << playback_step << endl;
      }
      break;
    case static_cast<int>('c'):
    case static_cast<int>('C'): 
      if (action == RELEASED) {
        if (cur_kinect == cur_icp_dst_kinect) {
          std::cout << "Source and destination kinects are the same!" << std::endl;
          break;
        }
        uint32_t k_dst = cur_icp_dst_kinect;  // This point cloud wont move
        uint32_t k_src = cur_kinect;  // 
        CalcNormalImage(cur_norm_data[k_src], cur_xyz_data[k_src], src_width, 
          src_height, 50, SimpleNormalApproximation);
        CalcNormalImage(cur_norm_data[k_dst], cur_xyz_data[k_dst], src_width, 
          src_height, 50, SimpleNormalApproximation);

        // Since k_dst wont move, we can save it's data to file.
        std::stringstream ss;
        ss << IM_DIR << "calibration_data" << k_dst << ".bin";
        SaveArrayToFile<float>(camera_view[k_dst].m, 16, ss.str());
        std::cout << "Calibration data saved to " << ss.str() << endl;

        Vector<float> pc1_src, pc2_src, npc1_src, npc2_src;
#ifdef CALIBRATION_RUN
        bool use_points_near_model = ICP_USE_POINTS_NEAR_MODEL;
#else
        bool use_points_near_model = false;
#endif
        if (use_points_near_model) {
#ifdef CALIBRATION_RUN
          // Collect the points that are just near the fitted model
          CalibrateGeometry* cal_model = (CalibrateGeometry*)models[0];
          cal_model->findPointsCloseToModel(pc1_src, npc1_src, 
            cur_xyz_data[k_dst], cur_norm_data[k_dst], coeffs[k_dst][cur_image], 
            ICP_PC_MODEL_DIST_THRESH);
          cal_model->findPointsCloseToModel(pc2_src, npc2_src, 
            cur_xyz_data[k_src], cur_norm_data[k_src], coeffs[k_src][cur_image], 
            ICP_PC_MODEL_DIST_THRESH);
#endif
        } else {
          for (uint32_t i = 0; i < src_dim; i++) {
            if (cur_xyz_data[k_dst][i * 3 + 2] < GDT_MAX_DIST &&
              cur_xyz_data[k_dst][i * 3 + 2] > 0) {
              // We have to pre-transform PC1:
  
              pc1_src.pushBack(cur_xyz_data[k_dst][i * 3]);
              pc1_src.pushBack(cur_xyz_data[k_dst][i * 3 + 1]);
              pc1_src.pushBack(cur_xyz_data[k_dst][i * 3 + 2]);
              npc1_src.pushBack(cur_norm_data[k_dst][i * 3]);
              npc1_src.pushBack(cur_norm_data[k_dst][i * 3 + 1]);
              npc1_src.pushBack(cur_norm_data[k_dst][i * 3 + 2]);
            }
            if (cur_xyz_data[k_src][i * 3 + 2] < GDT_MAX_DIST &&
              cur_xyz_data[k_src][i * 3 + 2] > 0) {
              pc2_src.pushBack(cur_xyz_data[k_src][i * 3]);
              pc2_src.pushBack(cur_xyz_data[k_src][i * 3 + 1]);
              pc2_src.pushBack(cur_xyz_data[k_src][i * 3 + 2]);
              npc2_src.pushBack(cur_norm_data[k_src][i * 3]);
              npc2_src.pushBack(cur_norm_data[k_src][i * 3 + 1]);
              npc2_src.pushBack(cur_norm_data[k_src][i * 3 + 2]);
            }
          }
        }

        // Now, since the PC2 point cloud gets transformed by it's matrix
        // in the ICP routine, we have to pre-transform PC1 points by it's
        // matrix incase the view matrix isn't identity
        Float3 pt, pt_transformed, norm, norm_transformed;
        Float4x4 normal_mat;
        Float4x4::inverse(normal_mat, camera_view[k_dst]);
        normal_mat.transpose();
        for (uint32_t i = 0; i < pc1_src.size(); i+= 3) {
          pt.set(&pc1_src[i]);
          Float3::affineTransformPos(pt_transformed, camera_view[k_dst], pt);
          pc1_src[i] = pt_transformed[0];
          pc1_src[i + 1] = pt_transformed[1];
          pc1_src[i + 2] = pt_transformed[2];
          norm.set(&npc1_src[i]);
          Float3::affineTransformVec(norm_transformed, normal_mat, norm);
          norm_transformed.normalize();
          npc1_src[i] = norm_transformed[0];
          npc1_src[i + 1] = norm_transformed[1];
          npc1_src[i + 2] = norm_transformed[2];
        }

        // Randomly permute the point clouds to find a subset
        Vector<float> pc1, npc1;
        MERSINE_TWISTER_ENG eng;
        jtil::data_str::Vector<int> indices;
        for (uint32_t i = 0; i < pc1_src.size() / 3; i++) {
          indices.pushBack(i);
        }
        int size = std::min<int>((int)(pc1_src.size()/3), MAX_ICP_PTS);
        for (int i = 0; i < size; i++) {
          UNIFORM_INT_DISTRIBUTION dist(i, indices.size()-1);
          int rand_index = dist(eng);
          int tmp = indices[i];
          indices[i] = indices[rand_index];
          indices[rand_index] = tmp;
        }
        indices.resize(size);
        for (int i = 0; i < (int)indices.size(); i++) {
          pc1.pushBack(pc1_src[indices[i] * 3]);
          pc1.pushBack(pc1_src[indices[i] * 3 + 1]);
          pc1.pushBack(pc1_src[indices[i] * 3 + 2]);
          npc1.pushBack(npc1_src[indices[i] * 3]);
          npc1.pushBack(npc1_src[indices[i] * 3 + 1]);
          npc1.pushBack(npc1_src[indices[i] * 3 + 2]);
        }

        Vector<float> pc2, npc2;
        indices.resize(0);
        for (uint32_t i = 0; i < pc2_src.size() / 3; i++) {
          indices.pushBack(i);
        }
        size = std::min<int>((int)(pc2_src.size()/3), MAX_ICP_PTS);
        for (int i = 0; i < size; i++) {
          UNIFORM_INT_DISTRIBUTION dist(i, indices.size()-1);
          int rand_index = dist(eng);
          int tmp = indices[i];
          indices[i] = indices[rand_index];
          indices[rand_index] = tmp;
        }
        indices.resize(size);
        for (int i = 0; i < (int)indices.size(); i++) {
          pc2.pushBack(pc2_src[indices[i] * 3]);
          pc2.pushBack(pc2_src[indices[i] * 3 + 1]);
          pc2.pushBack(pc2_src[indices[i] * 3 + 2]);
          npc2.pushBack(npc2_src[indices[i] * 3]);
          npc2.pushBack(npc2_src[indices[i] * 3 + 1]);
          npc2.pushBack(npc2_src[indices[i] * 3 + 2]);
        }

#ifdef CALIBRATION_RUN
        // Approximate the camera by using the fitted model coeffs
        ((CalibrateGeometry*)models[0])->calcCameraView(
          camera_view[k_dst], camera_view[k_src], k_dst, k_src, 
          (const float***)&coeffs[0], cur_image);
#endif

        // Now perform ICP for a tight fit
#ifdef PERFORM_ICP_FIT
        icp.num_iterations = ICP_NUM_ITERATIONS;
        icp.cos_normal_threshold = ICP_COS_NORM_THRESHOLD;
        icp.min_distance_sq = ICP_MIN_DISTANCE_SQ;
        icp.max_distance_sq = ICP_MAX_DISTANCE_SQ;
        icp.icp_method = ICP_METHOD;
        std::cout << "Performing ICP on " << (pc1.size()/3) << " and ";
        std::cout << (pc2.size()/3) << " pts" << std::endl;
  #ifdef USE_ICP_NORMALS
        // Use Normals
        icp.match(camera_view[k_src], &pc1[0], (pc1.size()/3), 
          &pc2[0], (pc2.size()/3), camera_view[k_src], &npc1[0],
          &npc2[0]);
  #else
        // Don't use normals
        icp.match(camera_view[k_src], &pc1[0], (pc1.size()/3), 
          &pc2[0], (pc2.size()/3), camera_view[k_src]);
  #endif

        // Create lines geometry from the last correspondance points:
        float* pc2_transformed = icp.getLastPC2Transformed();
        int* correspondances = icp.getLastCorrespondances();
        float* weights = icp.getLastWeights();
        float red[3] = {1, 0, 0};
        float blue[3] = {0, 0, 1};
        SAFE_DELETE(geometry_lines[k_src-1]);
        geometry_lines[k_src-1] = new GeometryColoredLines();
        for (uint32_t i = 0; i < (pc2.size()/3); i++) {
          if (weights[i] > EPSILON) {
            geometry_lines[k_src-1]->addLine(&pc2_transformed[i * 3],
              &pc1[correspondances[i] * 3], red, blue);
          }
        }
        geometry_lines[k_src-1]->syncVAO();
#endif

        // Now save the results to file
        ss.str("");
        ss << IM_DIR << "calibration_data" << k_src << ".bin";
        SaveArrayToFile<float>(camera_view[k_src].m, 16, ss.str());
        std::cout << "Calibration data saved to " << ss.str() << endl;
        last_icp_kinect = k_src;
        cur_icp_mat = icp.getTransforms().size() - 1;
      }
      break;
    case static_cast<int>('z'):
    case static_cast<int>('Z'): 
      if (action == RELEASED) {
        if (icp.getTransforms().size() > 0) {
          cur_icp_mat = (cur_icp_mat + 1) % icp.getTransforms().size();
        }
        std::cout << "cur_icp_mat = " << cur_icp_mat << std::endl;
      }
      break;
    case static_cast<int>('v'):
    case static_cast<int>('V'): 
      if (action == RELEASED) {
        render_correspondances = !render_correspondances;
        cout << "render_correspondances = " << render_correspondances << endl;
      }
      break;
    case static_cast<int>('x'):
    case static_cast<int>('X'): 
      if (action == RELEASED) {
#ifdef CALIBRATION_RUN
        render_all_views = !render_all_views;
        cout << "render_all_views = " << render_all_views << endl;
#endif
      }
      break;
    case static_cast<int>('h'):
    case static_cast<int>('H'):
      if (action == RELEASED) {
        saveCurrentCoeffs();
      }
      break;
    case static_cast<int>('i'):
    case static_cast<int>('I'):
      if (action == RELEASED) {
        if (!shift_down) {
          cur_kinect = (cur_kinect + 1) % MAX_KINECTS;
          cout << "cur_kinect = " << cur_kinect << endl;
        } else {
          cur_icp_dst_kinect = (cur_icp_dst_kinect + 1) % MAX_KINECTS;
          cout << "cur_icp_dst_kinect = " << cur_icp_dst_kinect << endl;
        }
      }
      break;
    case KEY_SPACE:
      if (action == RELEASED) {
#ifndef CALIBRATION_RUN
        if (fit_left && fit_right) {
          hand_to_modify = (hand_to_modify + 1) % 2;
        } else if (fit_left) {
          hand_to_modify = 0;
        } else {
          hand_to_modify = 1;
        }
        if (hand_to_modify == 0) {
          cout << "Adjusting LEFT HAND" << endl;
        } else {
          cout << "Adjusting RIGHT HAND" << endl;
        }
#endif
      }
      break;
    case static_cast<int>('Y'):
    case static_cast<int>('y'):
      if (action == RELEASED) {
        render_models = !render_models;
        for (uint32_t i = 0; i < num_models; i++) {
          models[i]->setRendererAttachement(render_models);
        }
      }
      break;
    case static_cast<int>('u'):
    case static_cast<int>('U'):
      if (action == RELEASED) {
        render_depth = !render_depth;
      }
      break;
    case static_cast<int>('j'):
    case static_cast<int>('J'):
      if (action == RELEASED) {
        fitFrame(false, true);
      }
      break;
#ifndef CALIBRATION_RUN
    case static_cast<int>('k'):
    case static_cast<int>('K'):
#if defined(WIN32) || defined(_WIN32)
      if (action == RELEASED) {
        if (shift_down) {
          repeat = 10;
        }
        {  // Closure
          for (int i = 0; i < repeat; i++) {
            // We only need to mark the first kinect's file as deleted...
            std::string full_im_filename = IM_DIR + 
              (im_files[0][cur_image].first);
            std::string new_full_im_filename = IM_DIR + string("deleted_") + 
              string(im_files[0][cur_image].first);
            r_coeff_file = IM_DIR + string("coeffr_") + im_files[0][cur_image].first;
            new_r_coeff_file = IM_DIR + string("deleted_") + string("coeffr_") + 
              im_files[0][cur_image].first;
            l_coeff_file = IM_DIR + string("coeffl_") + im_files[0][cur_image].first;
            new_l_coeff_file = IM_DIR + string("deleted_") + string("coeffl_") + 
              im_files[0][cur_image].first;

            bool move_OK = 
              MoveFile(r_coeff_file.c_str(), new_r_coeff_file.c_str()) &&
              MoveFile(l_coeff_file.c_str(), new_l_coeff_file.c_str()) &&
              MoveFile(full_im_filename.c_str(), new_full_im_filename.c_str());
            if (!move_OK) {
              cout << "Error moving files: " << endl;
              cout << "    - " << full_im_filename.c_str() << endl;
              cout << "    - " << r_coeff_file.c_str() << endl;
              cout << "    - " << l_coeff_file.c_str() << endl;
              cout << endl;
            } else {
              cout << "Files marked as deleted sucessfully: " << endl;
              cout << "    - " << full_im_filename.c_str() << endl;
              cout << "    - " << r_coeff_file.c_str() << endl;
              cout << "    - " << l_coeff_file.c_str() << endl;
              cout << endl;
              delete r_hand_coeffs[cur_image]; 
              delete l_hand_coeffs[cur_image];
              for (uint32_t i = cur_image; i < im_files[0].size() - 1; i++) {
                r_hand_coeffs[i] = r_hand_coeffs[i+1];
                l_hand_coeffs[i] = l_hand_coeffs[i+1];
              }
              SAFE_DELETE(im_files[0][cur_image].first); 
              im_files[0].deleteAtAndShift(cur_image);
            }
          }  // for (int i = 0; i < repeat; i++) {
          loadCurrentImage();
          InitXYZPointsForRendering();
        }  // Closure
      }
#else
      cout << "Move function not implemented for Mac OS X" << endl;
#endif
      break;
#endif
  }
}

using std::cout;
using std::endl;
jtil::math::Float4x4 mat_tmp;
WindowSettings settings;

void fitFrame(bool seed_with_last_frame, bool query_only) {
#ifdef CALIBRATION_RUN
  if (seed_with_last_frame && cur_image > 0) {
    for (uint32_t k = 0; k < MAX_KINECTS; k++) {
      memcpy(coeffs[k][cur_image], coeffs[k][cur_image-1], 
        sizeof(coeffs[k][cur_image][0]) * num_coeff);
    }
  }
  // Just fit each kinect independantly
  for (uint32_t k = 0; k < MAX_KINECTS; k++) {
    int16_t* depth = cur_depth_data[k];
    uint8_t* labels = cur_label_data[k];
    fit->fitModel(&depth, &labels, models, 
      &coeffs[k][cur_image], NULL, CalibrateGeometry::renormalizeCoeffs);
  }
#else
  if (seed_with_last_frame && cur_image > 0) {
    cout << "Using the previous frame as the optimization seed." << endl;
    if (fit_right) {
      r_hand_coeffs[cur_image]->copyCoeffFrom(r_hand_coeffs[cur_image - 1]);
    }
    if (fit_left) {
      l_hand_coeffs[cur_image]->copyCoeffFrom(l_hand_coeffs[cur_image - 1]);
    }
  }

  if (fit_left && !fit_right) {
    coeff[0] = l_hand_coeffs[cur_image]->coeff();
  } else if (!fit_left && fit_right) {
    coeff[0] = r_hand_coeffs[cur_image]->coeff();
  } else {
    coeff[0] = l_hand_coeffs[cur_image]->coeff();
    coeff[1] = r_hand_coeffs[cur_image]->coeff();
  }
  if (cur_image > 0) {
    if (fit_left && !fit_right) {
      prev_coeff[0] = l_hand_coeffs[cur_image - 1]->coeff();
    } else if (!fit_left && fit_right) {
      prev_coeff[0] = r_hand_coeffs[cur_image - 1]->coeff();
    } else {
      prev_coeff[0] = l_hand_coeffs[cur_image - 1]->coeff();
      prev_coeff[1] = r_hand_coeffs[cur_image - 1]->coeff();
    }
  }
  HandGeometryMesh::setCurrentStaticHandProperties(coeff[0]);
  if (query_only) {
    fit->queryObjFunc(cur_depth_data, cur_label_data, models, coeff);
  } else {
    if (cur_image > 0) {
      fit->fitModel(cur_depth_data, cur_label_data, models, coeff, 
        prev_coeff, HandModelCoeff::renormalizeCoeffs);
    } else {
      fit->fitModel(cur_depth_data, cur_label_data, models, coeff, 
        NULL, HandModelCoeff::renormalizeCoeffs);
    }
  }
#endif
}

void renderFrame(float dt) {
  if (rotate_light) {
    renderer::LightDir* light = render->light_dir();
    Float3* dir = light->direction_world();
    Float4x4::rotateMatYAxis(mat_tmp, dt);
    Float3 new_dir;
    Float3::affineTransformVec(new_dir, mat_tmp, *dir);
    dir->set(new_dir);
  }

  // Move the camera
  delta_pos.set(cur_dir);
  const Float3 zeros(0, 0, 0);
  if (!Float3::equal(delta_pos, zeros)) {
    delta_pos.normalize();
    Float3::scale(delta_pos, camera_speed * dt);
    if (running) {
      Float3::scale(delta_pos, camera_run_mulitiplier);
    }
    render->camera()->moveCamera(&delta_pos);
  }

#ifdef CALIBRATION_RUN
  if (!render_all_views) {
    models[0]->updateMatrices(coeffs[cur_kinect][cur_image]);
  } else {
    models[0]->updateMatrices(coeffs[0][cur_image]);
  }
#else
  float* cur_coeff;
  if (fit_right && fit_left) {
    cur_coeff = l_hand_coeffs[cur_image]->coeff();
    models[0]->updateMatrices(l_hand_coeffs[cur_image]->coeff());
    models[1]->updateMatrices(r_hand_coeffs[cur_image]->coeff());
  }
  if (fit_right) {
    cur_coeff = r_hand_coeffs[cur_image]->coeff();
    models[0]->updateMatrices(r_hand_coeffs[cur_image]->coeff());
  } else if (fit_left) {
    cur_coeff = l_hand_coeffs[cur_image]->coeff();
    models[0]->updateMatrices(l_hand_coeffs[cur_image]->coeff());
  }
  HandGeometryMesh::setCurrentStaticHandProperties(cur_coeff);
#endif

  // Now render the final frame
  Float4x4 identity;
  identity.identity();
  switch (render_output) {
  case 1:
    render->renderFrame(dt);
    if (render_depth) {
#ifdef CALIBRATION_RUN
      if (!render_all_views) {
        render->renderColoredPointCloud(geometry_points[cur_kinect], 
          &identity, 1.5f * static_cast<float>(settings.width) / 4.0f);
      } else {
        for (uint32_t k = 0; k < MAX_KINECTS; k++) {
          if (k == last_icp_kinect && icp.getTransforms().size() > 0) {
            render->renderColoredPointCloud(geometry_points[k], 
              &icp.getTransforms()[cur_icp_mat], 
              1.5f * static_cast<float>(settings.width) / 4.0f);
          } else {
            render->renderColoredPointCloud(geometry_points[k], 
              &camera_view[k], 1.5f * static_cast<float>(settings.width) / 4.0f);
          }
        }

      }
#else
      for (uint32_t k = 0; k < MAX_KINECTS; k++) {
        if (k == last_icp_kinect && icp.getTransforms().size() > 0) {
          render->renderColoredPointCloud(geometry_points[k], 
            &icp.getTransforms()[cur_icp_mat], 
            1.5f * static_cast<float>(settings.width) / 4.0f);
        } else {
          render->renderColoredPointCloud(geometry_points[k], 
            &camera_view[k], 1.5f * static_cast<float>(settings.width) / 4.0f);
        }
      }
#endif
      if (render_correspondances) {
        for (uint32_t k = 0; k < MAX_KINECTS - 1; k++) {
          if (geometry_lines[k] != NULL) {
            render->renderColoredLines(geometry_lines[k], &identity, 4.0f);
          }
        }
      }
    }
    break;
  case 2:
    float coeff[num_models * num_coeff_fit];
#ifdef CALIBRATION_RUN
    memcpy(coeff, coeffs[cur_kinect][cur_image], sizeof(coeff[0]) * num_coeff_fit);
#else
    if (fit_left && !fit_right) {
      memcpy(coeff, l_hand_coeffs[cur_image]->coeff(), sizeof(coeff[0])*num_coeff_fit);
    } else if (!fit_left && fit_right) {
      memcpy(coeff, r_hand_coeffs[cur_image]->coeff(), sizeof(coeff[0])*num_coeff_fit);
    } else {
      memcpy(coeff, l_hand_coeffs[cur_image]->coeff(), sizeof(coeff[0])*num_coeff_fit);
      memcpy(&coeff[num_coeff_fit], r_hand_coeffs[cur_image]->coeff(), 
        sizeof(coeff[0])*num_coeff_fit);
    }
#endif

#ifdef CALIBRATION_RUN
    fit->model_renderer()->drawDepthMap(coeff, num_coeff_fit, models,
      num_models, 0, false);
    fit->model_renderer()->visualizeDepthMap(wnd, 0);
#else
    fit->model_renderer()->drawDepthMap(coeff, num_coeff_fit, models,
      num_models, cur_kinect, false);
    fit->model_renderer()->visualizeDepthMap(wnd, cur_kinect);
#endif
    break;
  case 3:
    hand_detect->findHandLabels(cur_depth_data[cur_kinect], 
      cur_xyz_data[cur_kinect], HDLabelMethod::HDFloodfill, 
      cur_label_data[cur_kinect]);
    for (uint32_t v = 0; v < src_height; v++) {
      for (uint32_t u = 0; u < src_width; u++) {
        uint32_t i = v * src_width + u;
        uint32_t i_dst = (src_height - 1 - v) * src_width + u;
        int16_t depth = cur_depth_data[cur_kinect][i];
        uint8_t grey_val = (uint8_t)(depth >> 3);
        tex_data[3 * i_dst] = grey_val;
        tex_data[3 * i_dst + 1] = grey_val;
        tex_data[3 * i_dst + 2] = grey_val;
        if (cur_label_data[cur_kinect][i] != 0) {
          tex_data[3 * i_dst] = 255;
          tex_data[3 * i_dst + 1] = 0;
          tex_data[3 * i_dst + 2] = 0;
        }
      }
    }
    tex->reloadData((unsigned char*)tex_data);
    render->renderFullscreenQuad(tex);
    break;
  default:
    throw runtime_error("ERROR: render_output is an incorrect value");
  }
  wnd->swapBackBuffer();
}

int main(int argc, char *argv[]) {
  static_cast<void>(argc); static_cast<void>(argv);
#if defined(_DEBUG) && defined(_WIN32)
  _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
  // _CrtSetBreakAlloc(2997);
#endif

  cout << "Usage:" << endl;
  cout << "WSADQE - Move camera" << endl;
  cout << "shift - Sprint" << endl;
  cout << "mouse left click + drag - Rotate camera" << endl;
  cout << "space - Change hand to control" << endl;
  cout << "mouse right click + drag - Adjust coefficient" << endl;
  cout << "[] - Change adjustment coeff" << endl;
  cout << "r - rotate light" << endl;
  cout << "t - wireframe rendering" << endl;
  cout << "b - bounding sphere rendering" << endl;
  cout << "y - Render Hands ON/OFF" << endl;
  cout << "u - Render Point Cloud ON/OFF" << endl;
  cout << "12 - Render output type" << endl;
  cout << "+- - Change the current depth image" << endl;
  cout << "09 - Change the current depth image x 100" << endl;
  cout << "h - Store hand data to file" << endl;
  cout << "f - Fit model to current frame" << endl;
  cout << "g - Fit model to all remaining frames" << endl;
  cout << "p - Playback frames (@15fps)" << endl;
  cout << "o - Change playback frame skip" << endl;
  cout << "l - Go to start frame" << endl;
  cout << "c - Save calibration data (calibration mode only)" << endl;
  cout << "x - Render all views (calibration mode only)" << endl;
  cout << "z - Step through ICP frames (if ICP has been run) (calibration mode only)" << endl;
  cout << "v - Render Correspondances (if ICP has been run) (calibration mode only)" << endl;
  cout << "j - Query Objective Function Value" << endl;
  cout << "shift+12345 - Copy finger1234/thumb from last frame" << endl;
  cout << "k - (3 times) delete current file" << endl;
  cout << "i - Change the current kinect (for ICP to move onto dst)" << endl;
  cout << "I - Change the current dst kinect" << endl << endl;
  
  try {
    tp = new ThreadPool(NUM_WORKER_THREADS);

    clk = new jtil::clk::Clk();
    t1 = clk->getTime();
    
    // Initialize Windowing system
    Window::initWindowSystem();
    Texture::initTextureSystem();
    
    // Fill the settings structure
    settings.width = src_width*2;
    settings.height = src_height*2;
    //settings.width = 1280;
    //settings.height = 720;
    settings.fullscreen = false;
    settings.title = string("Hand Fit Project");
    settings.gl_major_version = 3;
    settings.gl_minor_version = 2;
    settings.num_depth_bits = 24;
    settings.num_stencil_bits = 0;
    settings.num_rgba_bits = 8;
    
    // Create the window
    wnd = new Window(settings);
    GLState::initGLState();    
    
    // Create an instance of the renderer
    FloatQuat eye_rot; eye_rot.identity();
    Float3 eye_pos(0, 0, 0);
    render = new Renderer();
    float fov_vert_deg = 360.0f * OpenNIFuncs::fVFOV_primesense_109 / 
      (2.0f * (float)M_PI);
    render->init(eye_rot, eye_pos, settings.width, settings.height,
      -HAND_MODEL_CAMERA_VIEW_PLANE_NEAR, -HAND_MODEL_CAMERA_VIEW_PLANE_FAR, 
      fov_vert_deg);

    tex = new Texture(GL_RGB8, src_width, src_height, GL_RGB, 
      GL_UNSIGNED_BYTE, (unsigned char*)tex_data, 
      TEXTURE_WRAP_MODE::TEXTURE_CLAMP, false,
      TEXTURE_FILTER_MODE::TEXTURE_NEAREST);

    // Initialize the XYZ points geometry
    for (uint32_t k = 0; k < MAX_KINECTS; k++) {
      geometry_points[k] = new GeometryColoredPoints;
      geometry_points[k]->vertices()->capacity(src_dim);
      geometry_points[k]->vertices()->resize(src_dim);
      geometry_points[k]->colors()->capacity(src_dim);
      geometry_points[k]->colors()->resize(src_dim);
    }
    for (uint32_t k = 0; k < MAX_KINECTS-1; k++) {
      geometry_lines[k] = NULL;
    }

    hand_detect = new HandDetector(tp);
    hand_detect->init(src_width, src_height, KINECT_HANDS_ROOT +
      FOREST_DATA_FILENAME);
 
    // Load the Kinect data for fitting from file and process it
    image_io = new DepthImagesIO();
    for (uint32_t k = 0; k < MAX_KINECTS; k++) {
#if defined(CALIBRATION_RUN)
      image_io->GetFilesInDirectory(im_files[k], IM_DIR, k, "calb");
      if (im_files[k].size() > CALIBRATION_MAX_FILES) {
        im_files[k].resize(CALIBRATION_MAX_FILES);
      }
#else
      image_io->GetFilesInDirectory(im_files[k], IM_DIR, k);
      if (im_files[k].size() == 0) {
        throw std::wruntime_error("ERROR: No frames exist for one of the "
          "sensors!");
      }
#endif
      std::stringstream ss;
      ss << IM_DIR << "calibration_data" << k << ".bin";
      if (!fileExists(ss.str())) {
        camera_view[k].identity();
      } else {
        LoadArrayFromFile<float>(camera_view[k].m, 16, ss.str());
      }
    }
    // Now align the frame times and frame zero (so that frame zero is T=0)
    for (uint32_t k = 0; k < MAX_KINECTS; k++) {
      int64_t t0 = im_files[k][0].second;
      for (uint32_t i = 0; i < im_files[k].size(); i++) {
        im_files[k][i].second -= t0;
      }
    }
    // Scale the frames so the time rate matches the global time rate (for drift)
    for (uint32_t k = 0; k < MAX_KINECTS; k++) {
      uint32_t len = im_files[k].size() - 1;
      double local_time = (double)(im_files[k][0].second - im_files[k][len].second);
      double global_time = (double)(im_files[k][0].third - im_files[k][len].third);
      double time_scale = global_time / local_time;
      for (uint32_t i = 0; i < im_files[k].size(); i++) {
        im_files[k][i].second = (uint64_t)((double)im_files[k][i].second * time_scale);
      }
    }

    cur_depth_data = new int16_t*[MAX_KINECTS];
    cur_label_data = new uint8_t*[MAX_KINECTS];
    for (uint32_t k = 0; k < MAX_KINECTS; k++) {
      cur_depth_data[k] = new int16_t[src_dim * 3];
      cur_label_data[k] = new uint8_t[src_dim];
    }

#if defined(CALIBRATION_RUN)
    // Load in all the images from file
    for (uint32_t k = 0; k < MAX_KINECTS; k++) {
      depth_database[k] = new int16_t*[im_files[k].size()];
      rgb_database[k] = new uint8_t*[im_files[k].size()];
      label_database[k] = new uint8_t*[im_files[k].size()];
      for (uint32_t f = 0; f < im_files[k].size(); f++) {
        depth_database[k][f] = new int16_t[src_dim];
        rgb_database[k][f] = new uint8_t[src_dim*3];
        label_database[k][f] = new uint8_t[src_dim];
        char* file = im_files[k][f].first;
        string full_filename = IM_DIR + string(file);
        image_io->LoadCompressedImage(full_filename, depth_database[k][f], 
          label_database[k][f], rgb_database[k][f]);
      }
    }

#endif
    loadCurrentImage();
  
    // Attach callback functions for event handling
    wnd->registerKeyboardCB(&KeyboardCB);
    wnd->registerMousePosCB(&MousePosCB);
    wnd->registerMouseButtonCB(&MouseButtonCB);
    wnd->registerMouseWheelCB(NULL);
    wnd->registerCharacterInputCB(NULL);

    // Create instances of the models to fit
    coeff = new float*[num_models];
    prev_coeff = new float*[num_models];
    models = new PoseModel*[num_models];
#ifdef CALIBRATION_RUN
    models[0] = new CalibrateGeometry(cal_type);
#else
    if (fit_left && fit_right) {
      models[0] = new HandGeometryMesh(LEFT);
      models[1] = new HandGeometryMesh(RIGHT);
    } else if (fit_left) {
      models[0] = new HandGeometryMesh(LEFT);
    } else if (fit_right) {
      models[0] = new HandGeometryMesh(RIGHT);
    }
#endif

    // Create the optimizer that will fit the models
    fit = new ModelFit(num_models, num_coeff_fit, num_model_fit_cameras);

#ifndef CALIBRATION_RUN
    Float4x4 old_view, cur_view, camera_view_inv;
    for (uint32_t k = 0; k < num_model_fit_cameras; k++) {
      fit->getCameraView(k, old_view);
      Float4x4::inverse(camera_view_inv, camera_view[k]);
      Float4x4::mult(cur_view, old_view, camera_view_inv);
      fit->setCameraView(k, cur_view);
    }
#endif

    // Load the coeffs from file
#ifdef CALIBRATION_RUN
    float calb_data[MAX_KINECTS * num_coeff];
    for (uint32_t i = 0; i < MAX_KINECTS; i++) {
      coeffs[i] = new float*[im_files[0].size()];
      for (uint32_t j = 0; j < im_files[0].size(); j++) {
        coeffs[i][j] = new float[num_coeff]; 
      }
    }

    for (uint32_t i = 0; i < im_files[0].size(); i++) {
      string filename = IM_DIR + string("coeff_") + im_files[0][i].first;
      if (fileExists(filename)) {
        LoadArrayFromFile<float>(calb_data, num_coeff * MAX_KINECTS, filename);
        for (uint32_t j = 0; j < MAX_KINECTS; j++) {
          memcpy(coeffs[j][i], &calb_data[j * num_coeff], sizeof(*calb_data) *
            num_coeff);
        }
      } else {
        for (uint32_t j = 0; j < MAX_KINECTS; j++) {
          for (uint32_t k = 0; k < num_coeff; k++) {
            coeffs[j][i][k] = 0.0f;
          }
          coeffs[j][i][CALIB_POS_Z] = 700.0f;  // Start 700 away 
        } 
      }
    }
#else
    r_hand_coeffs = new HandModelCoeff*[im_files[0].size()];
    l_hand_coeffs = new HandModelCoeff*[im_files[0].size()];
#ifdef LOAD_AND_SAVE_OLD_FORMAT_COEFFS
    for (uint32_t i = 0; i < im_files.size(); i++) {
      r_hand_coeffs[i] = new HandModelCoeff(kinect_interface::hand_net::HandType::RIGHT);
      r_hand_coeffs[i]->loadOldFormatFromFile(IM_DIR, string("coeffr_") + im_files[i].first);
      l_hand_coeffs[i] = new HandModelCoeff(kinect_interface::hand_net::HandType::LEFT);
      l_hand_coeffs[i]->loadOldFormatFromFile(IM_DIR, string("coeffl_") + im_files[i].first);
    }
    for (uint32_t i = 0; i < im_files.size(); i++) {
      string r_hand_file = string("coeffr_") + im_files[i].first;
      string l_hand_file = string("coeffl_") + im_files[i].first;
      if (fit_right) {
        r_hand_coeffs[i]->saveToFile(IM_DIR, r_hand_file);
      } else {
        r_hand_coeffs[i]->saveBlankFile(IM_DIR, r_hand_file);
      }
      if (fit_left) {
        l_hand_coeffs[i]->saveToFile(IM_DIR, l_hand_file);
      } else {
        l_hand_coeffs[i]->saveBlankFile(IM_DIR, l_hand_file);
      }
    }
#else
    for (uint32_t i = 0; i < im_files[0].size(); i++) {
      r_hand_coeffs[i] = new HandModelCoeff(kinect_interface::hand_net::HandType::RIGHT);
      r_hand_coeffs[i]->loadFromFile(IM_DIR, string("coeffr_") + im_files[0][i].first);
      l_hand_coeffs[i] = new HandModelCoeff(kinect_interface::hand_net::HandType::LEFT);
      l_hand_coeffs[i]->loadFromFile(IM_DIR, string("coeffl_") + im_files[0][i].first);
    }
#endif
#endif

    for (uint32_t i = 0; i < num_models; i++) {
      models[i]->setRendererAttachement(render_models);
    }

    // Finally, initialize the points for rendering
    InitXYZPointsForRendering();
    
    // Main render loop
    while (true) {
      t0 = t1;
      t1 = clk->getTime();
      float dt = static_cast<float>(t1-t0);

      if (continuous_fit) {
        if (cur_image < im_files[0].size() - 1) {
          cout << "fitting frame " << cur_image + 1 << " of ";
          cout << im_files[0].size() << endl;
          cur_image++;
          loadCurrentImage();
          fitFrame(true, false);
          saveCurrentCoeffs();
          InitXYZPointsForRendering();
        } else {
          std::cout << "Function Evals Per Second = ";
          std::cout << fit->func_eval_count() / (clk->getTime() - 
            continuous_fit_timer_start);
          continuous_fit = false;
        }
      }
      if (continuous_play) {
        if (cur_image < im_files[0].size() - playback_step) {
          continuous_play_timer_start += (t1 - t0);
          if (continuous_play_timer_start >= continuous_play_frame_time) {
            cur_image += playback_step;
            loadCurrentImage(false);
            continuous_play_timer_start = 0;
            InitXYZPointsForRendering();
          }
        } else {
          continuous_play = false;
          continuous_play_timer_start = 0;
        }
      }

      renderFrame(dt);
      
      // let someone else do some work
      std::this_thread::yield();
    }
  } catch (std::runtime_error e) {
    printf("%s\n", e.what());
#if defined(WIN32) || defined(_WIN32)
    system("PAUSE");
#endif
  }
  
  return 0;
}
