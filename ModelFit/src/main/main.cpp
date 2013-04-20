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
#include "renderer/gl_state.h"
#include "windowing/window.h"
#include "windowing/window_settings.h"
#include "model_fit/hand_geometry_mesh.h"
#include "model_fit/model_renderer.h"
#include "model_fit/model_fit.h"
#include "kinect_interface/hand_net/hand_model_coeff.h"
#include "kinect_interface/depth_images_io.h"
#include "kinect_interface/open_ni_funcs.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector.h"
#include "jtil/clk/clk.h"
#include "jtil/file_io/file_io.h"
#include "jtil/string_util/string_util.h"
#include "jtil/threading/thread_pool.h"

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif

// KINECT DATA
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_1/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_2_1/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_2_2/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_3/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_4/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_5/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_6/")  // Fit
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_7/")  // Fit -> Training Data

// PRIMESENSE DATA
#define IM_DIR_BASE string("data/hand_depth_data/")

//#define KINECT_DATA  // Otherwise Primesense 1.09 data

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
using namespace model_fit;
using namespace renderer;
using namespace windowing;
using namespace kinect_interface;
using namespace kinect_interface::hand_net;

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
bool running = false;

// Hand model
HandModelCoeff** l_hand_coeffs = NULL;  // Left Hand coefficients
HandModelCoeff** r_hand_coeffs = NULL;  // Right hand coeffs
PoseModel** models;
const uint32_t num_models = (fit_left ? 1 : 0) + (fit_right ? 1 : 0);
uint32_t cur_coeff = 0;
ModelFit* fit = NULL;
bool continuous_fit = false;  // fit frames continuously each frame
bool continuous_play = false;  // Play back recorded frames
int hand_to_modify = fit_left ? 0 : 1;
bool render_hand = true;
uint32_t coeff_src = 0;
float** coeff = NULL;  // Temp space only used when performing fit
float** prev_coeff = NULL; 

// Kinect Image data 
DepthImagesIO* image_io = NULL;
jtil::data_str::VectorManaged<char*> im_files;
float cur_xyz_data[src_dim*3];
float cur_uvd_data[src_dim*3];
int16_t cur_depth_data[src_dim*3];
uint8_t cur_label_data[src_dim];
uint8_t cur_image_rgb[src_dim*3];
uint32_t cur_image = 0;
GeometryColoredPoints* geometry_points= NULL;
float temp_xyz[3 * src_dim];
float temp_rgb[3 * src_dim];
bool render_depth = true;
int playback_step = 1;
OpenNIFuncs openni_funcs;

void quit() {
  for (uint32_t i = 0; i < num_models; i++) {
    models[i]->setRendererAttachement(true);  // Make sure Geometry manager deletes models
    delete models[i];
  }
  delete[] models;

  delete[] coeff;
  delete[] prev_coeff;
  delete image_io;
  delete clk; 
  if (l_hand_coeffs) {
    for (uint32_t i = 0; i < im_files.size(); i++) { 
      delete l_hand_coeffs[i];
    }
    delete[] l_hand_coeffs;
  }
  if (r_hand_coeffs) {
    for (uint32_t i = 0; i < im_files.size(); i++) {
      delete r_hand_coeffs[i];
    }
    delete[] r_hand_coeffs;
  }
  delete render;
  delete geometry_points;
  delete fit;
  Texture::shutdownTextureSystem();
  GLState::shutdownGLState();
  delete wnd;
  Window::killWindowSystem();
  exit(0);
}

void loadCurrentImage(bool print_to_screen = true) {
  char* file = im_files[cur_image];
  string full_filename = IM_DIR + string(file);
  if (print_to_screen) {
    std::cout << "loading image: " << full_filename << std::endl;
    std::cout << "cur_image = " << cur_image << " of ";
    std::cout << im_files.size() << std::endl;
  }

  image_io->LoadCompressedImageWithRedHands(full_filename, 
    cur_depth_data, cur_label_data, cur_image_rgb, NULL);

#ifdef KINECT_DATA
  DepthImagesIO::convertSingleImageToXYZ(cur_xyz_data, cur_depth_data);
#else
  openni_funcs.ConvertDepthImageToProjective((uint16_t*)cur_depth_data, 
    cur_uvd_data);
  openni_funcs.convertDepthToWorldCoordinates(cur_uvd_data, cur_xyz_data, 
    src_dim);
#endif
}

void InitXYZPointsForRendering() {
  if (geometry_points == NULL) {
    geometry_points = new GeometryColoredPoints;
  } else {
    geometry_points->unsyncVAO();
  }
  jtil::data_str::Vector<jtil::math::Float3>* vert = geometry_points->vertices();
  jtil::data_str::Vector<jtil::math::Float3>* cols = geometry_points->colors();

  if (vert->capacity() < src_dim) {
    vert->capacity(src_dim);
  }
  vert->resize(src_dim);
  if (cols->capacity() < src_dim) {
    cols->capacity(src_dim);
  }
  cols->resize(src_dim);

  float cur_col[3];
  for (uint32_t i = 0; i < src_dim; i++) {
    vert->at(i)->set(&cur_xyz_data[i*3]);
    if (cur_label_data[i] == 0) {
      cur_col[0] = 1.0f * static_cast<float>(cur_image_rgb[i*3]) / 255.0f;
      cur_col[1] = 1.0f * static_cast<float>(cur_image_rgb[i*3+1]) / 255.0f;
      cur_col[2] = 1.0f * static_cast<float>(cur_image_rgb[i*3+2]) / 255.0f;
    } else {
      cur_col[0] = std::max<float>(0.0f, (float)(cur_image_rgb[i*3]) / 255.0f - 0.1f);
      cur_col[1] = std::min<float>((float)(cur_image_rgb[i*3+1]) / 255.0f + 0.4f, 255.0f);
      cur_col[2] = std::max<float>(0.0f, (float)(cur_image_rgb[i*3+2]) / 255.0f - 0.1f);
    }
    cols->at(i)->set(cur_col);
  }
  geometry_points->syncVAO();
}


void saveCurrentHandCoeffs() {
  string r_hand_file = string("coeffr_") + im_files[cur_image];
  string l_hand_file = string("coeffl_") + im_files[cur_image];
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
}

void MouseButtonCB(int button, int action) {
  if (button == MOUSE_BUTTON_LEFT) {
    if (action == PRESSED) {
      camera_rotate = true;
    }
    if (action == RELEASED) {
      camera_rotate = false;
    }
  }
  else if (button == MOUSE_BUTTON_RIGHT) {
    if (action == PRESSED) {
      scale_coeff = true;
    }
    if (action == RELEASED) {
      scale_coeff = false;
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
  }
}

void renderFrame(float dt);
void fitFrame(bool seed_with_last_frame, bool query_only);

double continuous_fit_timer_start;
double continuous_play_timer_start;
const double continuous_play_frame_time = 1.0/15.0;
bool shift_down = false;
int delete_confirmed = 0;
string full_im_filename;
string r_coeff_file;
string l_coeff_file;
string new_full_im_filename;
string new_r_coeff_file;
string new_l_coeff_file;
void KeyboardCB(int key, int action) {
  int repeat = 1;
  if (key != 'k' && key != 'K') {
    delete_confirmed = 0;
  }

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
        saveCurrentHandCoeffs();
      }
      break;
    case KEY_KP_ADD:
      if (action == RELEASED) {
        cur_image = cur_image < static_cast<uint32_t>(im_files.size())-1 ? 
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
        if (cur_image >= im_files.size()) {
          cur_image = im_files.size()-1;
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
        cur_coeff = (cur_coeff + 1) % HandCoeff::NUM_PARAMETERS;
        cout << "cur_coeff = " << kinect_interface::hand_net::HandCoeffToString(cur_coeff); 
        if (hand_to_modify == 0) {
          cout << " = " << l_hand_coeffs[cur_image]->getCoeff(cur_coeff);
        } else {
          cout << " = " << r_hand_coeffs[cur_image]->getCoeff(cur_coeff);
        }
        cout << std::endl;
      }
      break;
    case static_cast<int>('{'):
    case static_cast<int>('['):
      if (action == RELEASED) {
        cur_coeff = cur_coeff != 0 ? (cur_coeff - 1) : HandCoeff::NUM_PARAMETERS - 1;
        cout << "cur_coeff = " << kinect_interface::hand_net::HandCoeffToString(cur_coeff); 
        if (hand_to_modify == 0) {
          cout << " = " << l_hand_coeffs[cur_image]->getCoeff(cur_coeff);
        } else {
          cout << " = " << r_hand_coeffs[cur_image]->getCoeff(cur_coeff);
        }
        cout << std::endl;
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
        std::cout << "playback_step = " << playback_step << std::endl;
      }
      break;
    case static_cast<int>('h'):
    case static_cast<int>('H'):
      if (action == RELEASED) {
        saveCurrentHandCoeffs();
      }
      break;
    case KEY_SPACE:
      if (action == RELEASED) {
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
      }
      break;
    case static_cast<int>('Y'):
    case static_cast<int>('y'):
      if (action == RELEASED) {
        render_hand = !render_hand;
        for (uint32_t i = 0; i < num_models; i++) {
          models[i]->setRendererAttachement(render_hand);
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
    case static_cast<int>('k'):
    case static_cast<int>('K'):
#if defined(WIN32) || defined(_WIN32)
      if (action == RELEASED) {
        if (shift_down) {
          repeat = 10;
        }

        for (int i = 0; i < repeat; i++) {
          full_im_filename = IM_DIR + string(im_files[cur_image]);
          new_full_im_filename = IM_DIR + string("deleted_") + 
            string(im_files[cur_image]);
          r_coeff_file = IM_DIR + string("coeffr_") + im_files[cur_image];
          new_r_coeff_file = IM_DIR + string("deleted_") + string("coeffr_") + 
            im_files[cur_image];
          l_coeff_file = IM_DIR + string("coeffl_") + im_files[cur_image];
          new_l_coeff_file = IM_DIR + string("deleted_") + string("coeffl_") + 
            im_files[cur_image];

          if (delete_confirmed == 1) {
            if(!MoveFile(full_im_filename.c_str(), new_full_im_filename.c_str()) 
              || !MoveFile(r_coeff_file.c_str(), new_r_coeff_file.c_str()) ||
              !MoveFile(l_coeff_file.c_str(), new_l_coeff_file.c_str())) {
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
              for (uint32_t i = cur_image; i < im_files.size() - 1; i++) {
                r_hand_coeffs[i] = r_hand_coeffs[i+1];
                l_hand_coeffs[i] = l_hand_coeffs[i+1];
              }
              im_files.deleteAtAndShift((uint32_t)cur_image);
              loadCurrentImage();
              InitXYZPointsForRendering();
            }
            if (i == (repeat - 1)) {
              delete_confirmed = 0;
            }
          } else {
            if (i == 0) {
              delete_confirmed++;
              cout << "About to mark files as deleted: " << endl;
              cout << "    - " << full_im_filename.c_str() << endl;
              cout << "    - " << r_coeff_file.c_str() << endl;
              cout << "    - " << l_coeff_file.c_str() << endl;
              cout << endl;
              cout << "Press 'd' again " << 2 - delete_confirmed;
              cout << " times to confirm" << endl;
            }
          }
        }
      }
#else
      cout << "Move function not implemented for Mac OS X" << endl;
#endif
      break;
  }
}

using std::cout;
using std::endl;
jtil::math::Float4x4 mat_tmp;
WindowSettings settings;

void fitFrame(bool seed_with_last_frame, bool query_only) {
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

  // Now render the final frame
  Float4x4 identity;
  identity.identity();
  switch (render_output) {
  case 1:
    render->renderFrame(dt);
    if (render_depth) {
      render->renderColoredPointCloud(geometry_points, &identity,
        4.0f * static_cast<float>(settings.width) / 4.0f);
        // 4.0f * static_cast<float>(settings.width) / 4.0f);
    }
    break;
  case 2:
    float coeff[num_models * HAND_NUM_COEFF];
    if (fit_left && !fit_right) {
      memcpy(coeff, l_hand_coeffs[cur_image]->coeff(), sizeof(coeff[0])*HAND_NUM_COEFF);
    } else if (!fit_left && fit_right) {
      memcpy(coeff, r_hand_coeffs[cur_image]->coeff(), sizeof(coeff[0])*HAND_NUM_COEFF);
    } else {
      memcpy(coeff, l_hand_coeffs[cur_image]->coeff(), sizeof(coeff[0])*HAND_NUM_COEFF);
      memcpy(&coeff[HAND_NUM_COEFF], r_hand_coeffs[cur_image]->coeff(), 
        sizeof(coeff[0])*HAND_NUM_COEFF);
    }

    fit->model_renderer()->drawDepthMap(coeff, HAND_NUM_COEFF, models,
      num_models, false);
    fit->model_renderer()->visualizeDepthMap(wnd);
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
  //_CrtSetBreakAlloc(4452);
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
  cout << "j - Query Objective Function Value" << endl;
  cout << "shift+12345 - Copy finger1234/thumb from last frame" << endl;
  cout << "k - (3 times) delete current file" << endl;
  
  try {
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

    // Load the Kinect data for fitting from file and process it
    image_io = new DepthImagesIO();
    image_io->GetFilesInDirectory(im_files, IM_DIR, false);
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
    if (fit_left && fit_right) {
      models[0] = new HandGeometryMesh(LEFT);
      models[1] = new HandGeometryMesh(RIGHT);
    } else if (fit_left) {
      models[0] = new HandGeometryMesh(LEFT);
    } else if (fit_right) {
      models[0] = new HandGeometryMesh(RIGHT);
    }

    // Create the optimizer that will fit the models
    fit = new ModelFit(num_models, HAND_NUM_COEFF);

    // Load the coeffs from file
    r_hand_coeffs = new HandModelCoeff*[im_files.size()];
    l_hand_coeffs = new HandModelCoeff*[im_files.size()];
#ifdef LOAD_AND_SAVE_OLD_FORMAT_COEFFS
    for (uint32_t i = 0; i < im_files.size(); i++) {
      r_hand_coeffs[i] = new HandModelCoeff(kinect_interface::hand_net::HandType::RIGHT);
      r_hand_coeffs[i]->loadOldFormatFromFile(IM_DIR, string("coeffr_") + im_files[i]);
      l_hand_coeffs[i] = new HandModelCoeff(kinect_interface::hand_net::HandType::LEFT);
      l_hand_coeffs[i]->loadOldFormatFromFile(IM_DIR, string("coeffl_") + im_files[i]);
    }
    for (uint32_t i = 0; i < im_files.size(); i++) {
      string r_hand_file = string("coeffr_") + im_files[i];
      string l_hand_file = string("coeffl_") + im_files[i];
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
    for (uint32_t i = 0; i < im_files.size(); i++) {
      r_hand_coeffs[i] = new HandModelCoeff(kinect_interface::hand_net::HandType::RIGHT);
      r_hand_coeffs[i]->loadFromFile(IM_DIR, string("coeffr_") + im_files[i]);
      l_hand_coeffs[i] = new HandModelCoeff(kinect_interface::hand_net::HandType::LEFT);
      l_hand_coeffs[i]->loadFromFile(IM_DIR, string("coeffl_") + im_files[i]);
    }
#endif

    for (uint32_t i = 0; i < num_models; i++) {
      models[i]->setRendererAttachement(render_hand);
    }

    // Finally, initialize the points for rendering
    InitXYZPointsForRendering();
    
    // Main render loop
    while (true) {
      t0 = t1;
      t1 = clk->getTime();
      float dt = static_cast<float>(t1-t0);

      if (continuous_fit) {
        if (cur_image < im_files.size() - 1) {
          cout << "fitting frame " << cur_image + 1 << " of ";
          cout << im_files.size() << endl;
          cur_image++;
          loadCurrentImage();
          fitFrame(true, false);
          saveCurrentHandCoeffs();
          InitXYZPointsForRendering();
        } else {
          std::cout << "Function Evals Per Second = ";
          std::cout << fit->func_eval_count() / (clk->getTime() - 
            continuous_fit_timer_start);
          continuous_fit = false;
        }
      }
      if (continuous_play) {
        if (cur_image < im_files.size() - playback_step) {
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
