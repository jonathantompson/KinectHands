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
#include "renderer/geometry/geometry_colored_mesh.h"
#include "renderer/geometry/geometry_manager.h"
#include "renderer/geometry/geometry_colored_points.h"
#include "windowing/window.h"
#include "windowing/window_settings.h"
#include "math/math_types.h"
#include "data_str/vector.h"
#include "clock/clock.h"
#include "string_util/string_util.h"
#include "hand_model/hand_model.h"
#include "hand_model/hand_model_geometry.h"
#include "hand_model/hand_model_renderer.h"
#include "hand_model/hand_model_fit.h"
#include "hand_net/hand_net.h"
#include "depth_images_io.h"
#include "open_ni_funcs.h"
#include "renderer/gl_state.h"

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif

//#define IM_DIR_BASE string("hand_data/left_only/set05/")
//#define IM_DIR_BASE string("hand_data/right_only/set05/")
// 3 -> Finished (4 partially finished)
//#define IM_DIR_BASE string("hand_data/both_hands/set03/") 

#define IM_DIR_BASE string("/data/hand_depth_data/") 

#if defined(__APPLE__)
  #define IM_DIR string("./../../../../../../../../../") + IM_DIR_BASE
  #define CONVNET_FILE string("./../../../../../../../../../data/handmodel_fullcoeffs_tanh_abs_mid_L2Pooling.net.convnet")
#else
  #define IM_DIR string("./../") + IM_DIR_BASE
  #define CONVNET_FILE string("./../data/handmodel_fullcoeffs_tanh_abs_mid_L2Pooling.net.convnet")
#endif
const bool fit_left = false;
const bool fit_right = true; 
const uint32_t num_hands = (fit_left ? 1 : 0) + (fit_right ? 1 : 0);

using namespace std;
using math::Float3;
using math::Int3;
using math::Float4;
using math::FloatQuat;
using math::Float4x4;
using data_str::Vector;
using hand_model::HandModel;
using hand_model::HandModelFit;
using hand_model::HandModelRenderer;
using hand_model::HandModelGeometry;
using hand_model::HandCoeff;
using hand_net::HandNet;
using renderer::Renderer;
using renderer::Geometry;
using renderer::GeometryManager;
using renderer::GeometryColoredMesh;
using renderer::GeometryColoredPoints;
using windowing::Window;
using windowing::WindowSettings;
using depth_images_io::DepthImagesIO;
using renderer::GLState;

Clock* clk = NULL;
double t1, t0;

// The main window and basic rendering system
Window* wnd = NULL;
Renderer* render = NULL;
bool rotate_light = false;
int render_output = 1;  // 0 - color, 1 - depth

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
HandModel** l_hands = NULL;  // Not using this yet
HandModel** r_hands = NULL;
uint32_t cur_coeff = 0;
HandModelRenderer* hand_renderer = NULL;
HandModelFit* hand_fit = NULL;
bool continuous_fit = false;  // fit frames continuously each frame
bool continuous_play = false;  // Play back recorded frames
int hand_to_modify = fit_left ? 0 : 1;
bool render_hand = true;
Eigen::MatrixXf coeffs;
uint32_t coeff_src = 0;

// Kinect Image data
DepthImagesIO* image_io = NULL;
data_str::VectorManaged<char*> im_files;
float image_xyz[src_dim*3];
int16_t cur_depth_data[src_dim*3];
uint8_t cur_label_data[src_dim];
uint8_t cur_image_rgb[src_dim*3];
uint32_t cur_image = 0;
GeometryColoredPoints* geometry_points= NULL;
float temp_xyz[3 * src_dim];
float temp_rgb[3 * src_dim];
bool render_depth = true;
int playback_step = 1;

// Convolutional Neural Network
HandNet* convnet = NULL;
float synthetic_depth[src_dim];
uint8_t synthetic_label[src_dim];

void quit() {
  delete image_io;
  delete clk;
  delete render;
  if (l_hands) {
    for (uint32_t i = 0; i < im_files.size(); i++) {
      delete l_hands[i];
    }
    delete[] l_hands;
  }
  if (r_hands) {
    for (uint32_t i = 0; i < im_files.size(); i++) {
      delete r_hands[i];
    }
    delete[] r_hands;
  }
  delete hand_renderer;
  delete hand_fit;
  GLState::shutdownGLState();
  delete wnd;
  delete geometry_points;
  delete convnet;
  Window::killWindowSystem();
  exit(0);
}

void loadCurrentImage() {
  char* file = im_files[cur_image];
  string full_filename = IM_DIR + string(file);
  std::cout << "loading image: " << full_filename << std::endl;
  
  image_io->LoadCompressedImageWithRedHands(full_filename, 
    cur_depth_data, cur_label_data, cur_image_rgb, NULL);
  DepthImagesIO::convertSingleImageToXYZ(image_xyz, cur_depth_data);
}

void InitXYZPointsForRendering() {
  if (geometry_points == NULL) {
    geometry_points = new GeometryColoredPoints;
  } else {
    geometry_points->unsyncVAO();
  }
  data_str::Vector<math::Float3>* vert = geometry_points->vertices();
  data_str::Vector<math::Float3>* cols = geometry_points->colors();

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
    vert->at(i)->set(&image_xyz[i*3]);
    if (cur_label_data[i] == 0) {
      cur_col[0] = 1.2f * static_cast<float>(cur_image_rgb[i*3]) / 255.0f;
      cur_col[1] = 1.2f * static_cast<float>(cur_image_rgb[i*3+1]) / 255.0f;
      cur_col[2] = 1.2f * static_cast<float>(cur_image_rgb[i*3+2]) / 255.0f;
    } else {
      cur_col[0] = 1.6f * static_cast<float>(cur_image_rgb[i*3]) / 255.0f;
      cur_col[1] = 1.6f * static_cast<float>(cur_image_rgb[i*3+1]) / 255.0f;
      cur_col[2] = 1.6f * static_cast<float>(cur_image_rgb[i*3+2]) / 255.0f;
    }
    cols->at(i)->set(cur_col);
  }
  geometry_points->syncVAO();
}


void saveCurrentHandCoeffs() {
  string r_hand_file = string("coeffr_") + im_files[cur_image];
  string l_hand_file = string("coeffl_") + im_files[cur_image];
  if (fit_right) {
    r_hands[cur_image]->saveToFile(IM_DIR, r_hand_file);
  } else {
    r_hands[cur_image]->saveBlankFile(IM_DIR, r_hand_file);
  }
  if (fit_left) {
    l_hands[cur_image]->saveToFile(IM_DIR, l_hand_file);
  } else {
    l_hands[cur_image]->saveBlankFile(IM_DIR, l_hand_file);
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
    if (hand_to_modify == 0) {
      coeff_val = l_hands[cur_image]->getCoeff(cur_coeff);
      l_hands[cur_image]->setCoeff(cur_coeff, coeff_val - theta_y);
    } else {
      coeff_val = r_hands[cur_image]->getCoeff(cur_coeff);
      r_hands[cur_image]->setCoeff(cur_coeff, coeff_val - theta_y);
    }
    cout << "cur_coeff " << hand_model::HandCoeffToString(cur_coeff);
    cout << " --> " << coeff_val - theta_y << endl;
  }
}

void renderFrame(float dt);
void fitFrame(bool seed_with_last_frame);

double continuous_fit_timer_start;
double continuous_play_timer_start;
const double continuous_play_frame_time = 1.0/15.0;
void KeyboardCB(int key, int action) {
  switch (key) {
    case KEY_LSHIFT:
      if (action == PRESSED) {
        running = true;
      } else {
        running = false;
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
    case static_cast<int>('1'):
      if (action == RELEASED) {
        render_output = 1;
      }
      break;
    case static_cast<int>('2'):
      if (action == RELEASED) {
        render_output = 2;
      }
      break;
    case KEY_KP_ADD:
      if (action == RELEASED) {
        cur_image = cur_image < static_cast<uint32_t>(im_files.size())-1 ? 
          cur_image+1 : cur_image;
        loadCurrentImage();
        InitXYZPointsForRendering();
        std::cout << "cur_image = " << cur_image << std::endl;
      }
      break;
    case KEY_KP_SUBTRACT:
      if (action == RELEASED) {
        cur_image = cur_image > 0 ? cur_image-1 : 0;
        loadCurrentImage();
        InitXYZPointsForRendering();
        std::cout << "cur_image = " << cur_image << std::endl;
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
        std::cout << "cur_image = " << cur_image << std::endl;
      }
      break;
    case static_cast<int>('9'):
      if (action == RELEASED) {
        cur_image = cur_image >= 100 ? cur_image-100 : 0;
        loadCurrentImage();
        InitXYZPointsForRendering();
        std::cout << "cur_image = " << cur_image << std::endl;
      }
      break;
    case static_cast<int>('}'):
    case static_cast<int>(']'):
      if (action == RELEASED) {
        cur_coeff = (cur_coeff + 1) % HandCoeff::NUM_PARAMETERS;
        cout << "cur_coeff = " << hand_model::HandCoeffToString(cur_coeff); 
        if (hand_to_modify == 0) {
          cout << " = " << l_hands[cur_image]->getCoeff(cur_coeff);
        } else {
          cout << " = " << r_hands[cur_image]->getCoeff(cur_coeff);
        }
        cout << std::endl;
      }
      break;
    case static_cast<int>('{'):
    case static_cast<int>('['):
      if (action == RELEASED) {
        cur_coeff = cur_coeff != 0 ? (cur_coeff - 1) : HandCoeff::NUM_PARAMETERS - 1;
        cout << "cur_coeff = " << hand_model::HandCoeffToString(cur_coeff); 
        if (hand_to_modify == 0) {
          cout << " = " << l_hands[cur_image]->getCoeff(cur_coeff);
        } else {
          cout << " = " << r_hands[cur_image]->getCoeff(cur_coeff);
        }
        cout << std::endl;
      }
      break;
    case static_cast<int>('f'):
    case static_cast<int>('F'):
      if (action == RELEASED) {
        cout << "Fitting model..." << endl;
        fitFrame(false);
      }
      break;
    case static_cast<int>('g'):
    case static_cast<int>('G'):
      if (action == RELEASED) {
        continuous_fit = !continuous_play && !continuous_fit;
        if (continuous_fit) {
          hand_fit->resetFuncEvalCount();
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
        std::cout << "cur_image = " << cur_image << std::endl;
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
        hand_renderer->setRendererAttachement(render_hand);
      }
      break;
    case static_cast<int>('u'):
    case static_cast<int>('U'):
      if (action == RELEASED) {
        render_depth = !render_depth;
      }
      break;
    case static_cast<int>('J'):
    case static_cast<int>('j'):
      if (action == RELEASED) {
        HandModel* hands[2];
        if (fit_left && !fit_right) {
          hands[0] = l_hands[cur_image];
          hands[1] = NULL;
        } else if (!fit_left && fit_right) {
          hands[0] = r_hands[cur_image];
          hands[1] = NULL;
        } else {
          hands[0] = l_hands[cur_image];
          hands[1] = r_hands[cur_image];
        }
        cout << "Objective Function: ";
        cout << hand_fit->queryObjectiveFunction(cur_depth_data, cur_label_data,
          hands) << endl;
      }
      break;
    case static_cast<int>('c'):
    case static_cast<int>('C'):
      if (action == RELEASED) {
        coeff_src = (coeff_src + 1) % 2;
        if (coeff_src == 0) {
          std::cout << "Coeff source: PSO" << std::endl;
        } else {
          std::cout << "Coeff source: Convnet" << std::endl;
        }
        break;
      }
  }
}

using std::cout;
using std::endl;
math::Float4x4 mat_tmp;
WindowSettings settings;

void fitFrame(bool seed_with_last_frame) {
  if (seed_with_last_frame && cur_image > 0) {
    cout << "Using the previous frame as the optimization seed." << endl;
    if (fit_right) {
      r_hands[cur_image]->coeff() = r_hands[cur_image - 1]->coeff();
    }
    if (fit_left) {
      l_hands[cur_image]->coeff() = l_hands[cur_image - 1]->coeff();
    }
  }
  HandModel* hands[2];
  if (fit_left && !fit_right) {
    hands[0] = l_hands[cur_image];
    hands[1] = NULL;
  } else if (!fit_left && fit_right) {
    hands[0] = r_hands[cur_image];
    hands[1] = NULL;
  } else {
    hands[0] = l_hands[cur_image];
    hands[1] = r_hands[cur_image];
  }
  hand_fit->fitModel(cur_depth_data, cur_label_data, hands);
}

void renderFrame(float dt) {
  if (rotate_light) {
    renderer::LightDir* light = render->light_dir();
    Float3* dir = light->direction_world();
    mat_tmp.rotateMatYAxis(dt);
    Float3 new_dir;
    Float3::affineTransformVec(&new_dir, &mat_tmp, dir);
    dir->set(&new_dir);
  }

  // Update the global HandScale with the current model version
  if (r_hands[cur_image]->local_scale() != l_hands[cur_image]->local_scale()) {
    throw std::wruntime_error("ERROR: HAND MODEL SCALES ARE DIFFERENT!");
  }
  HandModel::scale = r_hands[cur_image]->local_scale();

  // Move the camera
  delta_pos.set(&cur_dir);
  if (!delta_pos.equal(0,0,0)) {
    delta_pos.normalize();
    delta_pos.scale(camera_speed * dt);
    if (running) {
      delta_pos.scale(camera_run_mulitiplier);
    }
    render->camera()->moveCamera(&delta_pos);
  }

  // Extract the synthetic depth map (potentially required for convnet)
  HandModel* hands[2];
  if (coeff_src == 1) {
    if (fit_left && !fit_right) {
      coeffs = l_hands[cur_image]->coeff();
      hands[0] = l_hands[cur_image];
      hands[1] = NULL;
    } else if (!fit_left && fit_right) {
      coeffs = r_hands[cur_image]->coeff();
      hands[0] = r_hands[cur_image];
      hands[1] = NULL;
    } else {
      coeffs.block<1, HAND_NUM_COEFF>(0, 0) = l_hands[cur_image]->coeff();
      coeffs.block<1, HAND_NUM_COEFF>(0, HAND_NUM_COEFF) = r_hands[cur_image]->coeff();
      hands[0] = l_hands[cur_image];
      hands[1] = r_hands[cur_image];
    }

    hand_renderer->drawDepthMap(coeffs, hands, num_hands);
    hand_renderer->extractDepthMap(synthetic_depth);

    HandNet::createLabelFromSyntheticDepth(synthetic_depth, synthetic_label);
    convnet->calcHandCoeff(synthetic_depth, synthetic_label, coeffs);
  }

  if (fit_right) {
    hand_renderer->updateMatrices(
      (coeff_src == 0) ? r_hands[cur_image]->coeff() : coeffs,
      r_hands[cur_image]->hand_type());
  }
  if (fit_left) {
    hand_renderer->updateMatrices(
      (coeff_src == 0) ? l_hands[cur_image]->coeff() : coeffs,
      l_hands[cur_image]->hand_type());
  }

  // Now render the final frame
  switch (render_output) {
  case 1:
    render->renderFrame(dt);
    if (render_depth) {
      render->renderColoredPointCloud(geometry_points, &math::identity,
        3.0f * static_cast<float>(settings.width) / 4.0f);
        // 4.0f * static_cast<float>(settings.width) / 4.0f);
    }
    break;
  case 2:
    if (coeff_src != 1) {
      hand_renderer->drawDepthMap(coeffs, hands, num_hands);
    }
    hand_renderer->visualizeDepthMap(wnd);
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
  // _CrtSetBreakAlloc(6990);
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
  cout << "y - Render Hands ON/OFF" << endl;
  cout << "u - Render Point Cloud ON/OFF" << endl;
  cout << "12 - Render output type" << endl;
  cout << "+- - Change the current depth image" << endl;
  cout << "09 - Change the current depth image x 100" << endl;
  cout << "h - Store hand data to file" << endl;
  cout << "f - Fit model to current frame" << endl;
  cout << "g - Fit model to all remaining frames" << endl;
  cout << "j - Query the current residue" << endl;
  cout << "p - Playback frames (@15fps)" << endl;
  cout << "o - Change playback frame skip" << endl;
  cout << "l - Go to start frame" << endl;
  
  try {
    clk = new Clock();
    t1 = clk->getTime();
    
    // Initialize Windowing system
    Window::initWindowSystem();
    GLState::initGLState();
    
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

    // Load the convnet from file
    convnet = new HandNet(CONVNET_FILE);
    
    // Create an instance of the renderer
    math::FloatQuat eye_rot; eye_rot.identity();
    math::Float3 eye_pos(0, 0, 0);
    render = new Renderer();
    render->init(eye_rot, eye_pos, settings.width, settings.height,
                 -HAND_CAMERA_VIEW_PLANE_NEAR, -HAND_CAMERA_VIEW_PLANE_FAR, 
                 HAND_CAMERA_FOV);
    
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
    
    // Create the hand data and attach it to the renderer for lighting
    hand_renderer = new HandModelRenderer(render, fit_left, fit_right);
    coeffs.resize(1, HAND_NUM_COEFF * num_hands);
    hand_fit = new HandModelFit(hand_renderer, num_hands);
    r_hands = new HandModel*[im_files.size()];
    for (uint32_t i = 0; i < im_files.size(); i++) {
      r_hands[i] = new HandModel(hand_model::HandType::RIGHT);
      r_hands[i]->loadFromFile(IM_DIR, string("coeffr_") + im_files[i]);
    }
    l_hands = new HandModel*[im_files.size()];
    for (uint32_t i = 0; i < im_files.size(); i++) {
      l_hands[i] = new HandModel(hand_model::HandType::LEFT);
      l_hands[i]->loadFromFile(IM_DIR, string("coeffl_") + im_files[i]);
    }
    hand_renderer->setRendererAttachement(render_hand);

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
          fitFrame(true);
          saveCurrentHandCoeffs();
          InitXYZPointsForRendering();
        } else {
          std::cout << "Function Evals Per Second = ";
          std::cout << hand_fit->func_eval_count() / (clk->getTime() - 
            continuous_fit_timer_start);
          continuous_fit = false;
        }
      }
      if (continuous_play) {
        if (cur_image < im_files.size() - playback_step) {
          continuous_play_timer_start += (t1 - t0);
          if (continuous_play_timer_start >= continuous_play_frame_time) {
            cur_image += playback_step;
            loadCurrentImage();
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
