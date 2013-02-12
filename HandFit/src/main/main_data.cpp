//
//  main_data.cpp
//
//  Takes the coeffient values from HandFit and creates data for the convnet
//  classifier.
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
#include "depth_images_io.h"
#include "open_ni_funcs.h"
#include "renderer/gl_state.h"
#include "hand_net/hand_net.h"

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif

//#define IM_DIR_BASE string("hand_data/left_only/set05/")
//#define IM_DIR_BASE string("hand_data/right_only/set05/")
// 3 -> Finished (4 partially finished)
//#define IM_DIR_BASE string("hand_data/both_hands/set03/") 

#define IM_DIR_BASE string("/data/hand_depth_data/") 
#define DST_IM_DIR_BASE string("/data/hand_depth_data_processed/") 

#define FILE_SKIP 3
#define MAX_FILES 1

#if defined(__APPLE__)
  #define IM_DIR string("../../../../../../../../../") + IM_DIR_BASE
  #define DST_IM_DIR string("../../../../../../../../../") + DST_IM_DIR_BASE

#else
  #define IM_DIR string("./../") + IM_DIR_BASE
  #define DST_IM_DIR string("./../") + DST_IM_DIR_BASE
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
using renderer::Renderer;
using renderer::Geometry;
using renderer::GeometryManager;
using renderer::GeometryColoredMesh;
using renderer::GeometryColoredPoints;
using windowing::Window;
using windowing::WindowSettings;
using depth_images_io::DepthImagesIO;
using renderer::GLState;
using hand_net::HandNet;

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
HandModel* l_hand = NULL;  // Not using this yet
HandModel* r_hand = NULL;
HandModelRenderer* hand_renderer = NULL;
HandModelFit* hand_fit = NULL;
Eigen::MatrixXf coeffs;
float synthetic_depth[src_dim];
float cropped_downs_depth[HAND_NET_PIX * HAND_NET_PIX];
float cropped_downs_xyz[3 * HAND_NET_PIX * HAND_NET_PIX];

// Kinect Image data
DepthImagesIO* image_io = NULL;
data_str::VectorManaged<char*> im_files;
int16_t cur_depth_data[src_dim*3];
uint8_t cur_label_data[src_dim];
uint8_t cur_image_rgb[src_dim*3];
uint32_t cur_image = 0;
GeometryColoredPoints* geometry_points= NULL;
bool render_depth = true;
int playback_step = 1;
float r_modified_coeff[HAND_NUM_COEFF];
float l_modified_coeff[HAND_NUM_COEFF];

void quit() {
  delete image_io;
  delete clk;
  delete render;
  if (l_hand) {
    delete l_hand;
  }
  if (r_hand) {
    delete r_hand;
  }
  delete hand_renderer;
  delete hand_fit;
  GLState::shutdownGLState();
  delete wnd;
  delete geometry_points;
  Window::killWindowSystem();
  exit(0);
}

void loadCurrentImage() {
  char* file = im_files[cur_image];
  string full_filename = IM_DIR + string(file);
  std::cout << "loading image: " << full_filename << std::endl;

  image_io->LoadCompressedImage(full_filename, 
    cur_depth_data, cur_label_data, cur_image_rgb);
}

void saveModifiedHandCoeffs() {
  string r_hand_file = string("coeffr_") + im_files[cur_image];
  string l_hand_file = string("coeffl_") + im_files[cur_image];
  if (fit_right) {
    r_hand->saveToFile(DST_IM_DIR, r_hand_file);
  } else {
    r_hand->saveBlankFile(DST_IM_DIR, r_hand_file);
  }
  if (fit_left) {
    l_hand->saveToFile(DST_IM_DIR, l_hand_file);
  } else {
    l_hand->saveBlankFile(DST_IM_DIR, l_hand_file);
  }
  cout << "hand data saved to file" << endl;
}

using std::cout;
using std::endl;
math::Float4x4 mat_tmp;
WindowSettings settings;

int main(int argc, char *argv[]) {
  static_cast<void>(argc); static_cast<void>(argv);
#if defined(_DEBUG) && defined(_WIN32)
  _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
  // _CrtSetBreakAlloc(6990);
#endif

  cout << "Usage:" << endl;
  cout << "<none: everything is automated!" << endl;
  
  try {
    clk = new Clock();
    t1 = clk->getTime();
    
    // Initialize Windowing system
    Window::initWindowSystem();
    GLState::initGLState();
    
    // Fill the settings structure
    settings.width = src_width * 2;
    settings.height = src_height * 2;
    settings.fullscreen = false;
    settings.title = string("Hand Fit Project");
    settings.gl_major_version = 3;
    settings.gl_minor_version = 2;
    settings.num_depth_bits = 24;
    settings.num_stencil_bits = 0;
    settings.num_rgba_bits = 8;
    
    // Create the window so that we have a valid openGL context
    wnd = new Window(settings);
    
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
    wnd->registerKeyboardCB(NULL);
    wnd->registerMousePosCB(NULL);
    wnd->registerMouseButtonCB(NULL);
    wnd->registerMouseWheelCB(NULL);
    wnd->registerCharacterInputCB(NULL);
    
    // Create the hand data and attach it to the renderer for lighting
    if (num_hands != 1) {
      throw std::runtime_error("ERROR: This main routine only works with one "
        "hand (left or right)!");
    }
    hand_renderer = new HandModelRenderer(render, fit_left, fit_right);
    hand_fit = new HandModelFit(hand_renderer, num_hands);
    coeffs.resize(1, HAND_NUM_COEFF * num_hands);
    r_hand = new HandModel(hand_model::HandType::RIGHT);
    l_hand = new HandModel(hand_model::HandType::LEFT);

    if (HAND_NET_IM_SIZE * HAND_NET_DOWN_FACT != HAND_NET_PIX) {
      throw std::wruntime_error("ERROR: training image size is not an integer"
        " multiple of the total hand size.");
    } else {
      std::cout << "Using a cropped source image of " << HAND_NET_PIX;
      std::cout << std::endl << "Final image size after processing is ";
      std::cout << (HAND_NET_IM_SIZE) << std::endl;
    }

    // Iterate through, saving each of the data points
    HandModel* hands[2];
    for (uint32_t i = 0; i < std::min<uint32_t>(im_files.size(), MAX_FILES); 
      i += FILE_SKIP) {
      std::cout << "processing image " << (i/FILE_SKIP) << " of ";
      std::cout << (im_files.size()/FILE_SKIP) << std::endl;

      // Load in the image (rgb + depth) and load in the fitted coefficients
      cur_image = i;
      loadCurrentImage();

      r_hand->loadFromFile(IM_DIR, string("coeffr_") + im_files[cur_image]);
      l_hand->loadFromFile(IM_DIR, string("coeffl_") + im_files[cur_image]);
      hand_renderer->setHandRendering(true);

      // Render a synthetic depth image:
      if (fit_left && !fit_right) {
        coeffs = l_hand->coeff();
        hands[0] = l_hand;
        hands[1] = NULL;
      } else if (!fit_left && fit_right) {
        coeffs = r_hand->coeff();
        hands[0] = r_hand;
        hands[1] = NULL;
      } else {
        coeffs.block<1, HAND_NUM_COEFF>(0, 0) = l_hand->coeff();
        coeffs.block<1, HAND_NUM_COEFF>(0, HAND_NUM_COEFF) = r_hand->coeff();
        hands[0] = l_hand;
        hands[1] = r_hand;
      }

      // TEMP CODE:
      std::cout << std::setprecision(10);
      std::cout << std::fixed;
      std::cout << std::endl;
      for (uint32_t i = 0; i < coeffs.size(); i++) {
        std::cout << coeffs(i) << ", ";
      }
      std::cout << std::endl;
      // END TEMP CODE:

      hand_renderer->drawDepthMap(coeffs, hands, num_hands);
      // Rendering to screen will slow it all down.
      //hand_renderer->visualizeDepthMap(wnd);  
      //wnd->swapBackBuffer();

      hand_renderer->extractDepthMap(synthetic_depth);

      // TEMP CODE:
      hand_renderer->drawDepthMap(coeffs, hands, num_hands);
      hand_renderer->extractDepthMap(synthetic_depth);
      // END TEMP CODE:

      // TEMP CODE:
      image_io->saveUncompressedDepth<float>(string("./../data/") + 
        string("big") + im_files[cur_image] + string("1"), synthetic_depth, 
        src_width, src_height);
      // END TEMP CODE:

      float x_com;
      float y_com;
      float z_com;
      float std;
      HandNet::calcHandImageFromSytheticDepth(synthetic_depth, 
        cropped_downs_depth, cropped_downs_xyz, &x_com, &y_com, &z_com, &std);

      // Save the cropped image to file:
      image_io->saveUncompressedDepth<float>(DST_IM_DIR + im_files[cur_image], 
        cropped_downs_depth, HAND_NET_IM_SIZE, HAND_NET_IM_SIZE);

      // Play with the coeff values.
#if defined(DEBUG) || defined(_DEBUG)
      std::cout << "left before:" << std::endl;
      l_hand->printCoeff();
      std::cout << "right before:" << std::endl;
      r_hand->printCoeff();
#endif

      // 1. subtract off the xyz_com from the position
      r_hand->coeff()(HandCoeff::HAND_POS_X) -= x_com;
      r_hand->coeff()(HandCoeff::HAND_POS_X) /=std;
      r_hand->coeff()(HandCoeff::HAND_POS_Y) -= y_com;
      r_hand->coeff()(HandCoeff::HAND_POS_Y) /=std;
      r_hand->coeff()(HandCoeff::HAND_POS_Z) -= z_com;
      r_hand->coeff()(HandCoeff::HAND_POS_Z) /=std;
      l_hand->coeff()(HandCoeff::HAND_POS_X) -= x_com;
      l_hand->coeff()(HandCoeff::HAND_POS_X) /=std;
      l_hand->coeff()(HandCoeff::HAND_POS_Y) -= y_com;
      l_hand->coeff()(HandCoeff::HAND_POS_Y) /=std;
      l_hand->coeff()(HandCoeff::HAND_POS_Z) -= z_com;
      l_hand->coeff()(HandCoeff::HAND_POS_Z) /=std;

      // 2. convert quaternion to euler angles (might be easier to learn)
      FloatQuat lquat(l_hand->coeff()(HandCoeff::HAND_ORIENT_X), 
        l_hand->coeff()(HandCoeff::HAND_ORIENT_Y),
        l_hand->coeff()(HandCoeff::HAND_ORIENT_Z),
        l_hand->coeff()(HandCoeff::HAND_ORIENT_W));
      float leuler[3];
      lquat.quat2EulerAngles(leuler[0], leuler[1], leuler[2]);
      for (uint32_t i = 0; i < 3; i++) {
        l_hand->coeff()(HandCoeff::HAND_ORIENT_X + i) = leuler[i];
      }
      // Now shift all the other coeffs down
      for (uint32_t i = HandCoeff::HAND_ORIENT_W + 1; i < HAND_NUM_COEFF; i++) {
        l_hand->coeff()(i-1) = l_hand->coeff()(i);
      }
      l_hand->coeff()(HAND_NUM_COEFF - 1) = 0;
#if defined(DEBUG) || defined(_DEBUG)
      std::cout << "left after" << std::endl;
      l_hand->printCoeff();
#endif

      FloatQuat rquat(r_hand->coeff()(HandCoeff::HAND_ORIENT_X), 
        r_hand->coeff()(HandCoeff::HAND_ORIENT_Y),
        r_hand->coeff()(HandCoeff::HAND_ORIENT_Z),
        r_hand->coeff()(HandCoeff::HAND_ORIENT_W));
      float reuler[3];
      rquat.quat2EulerAngles(reuler[0], reuler[1], reuler[2]);
      for (uint32_t i = 0; i < 3; i++) {
        r_hand->coeff()(HandCoeff::HAND_ORIENT_X + i) = reuler[i];
      }
      // Now shift all the other coeffs down
      for (uint32_t i = HandCoeff::HAND_ORIENT_W + 1; i < HAND_NUM_COEFF; i++) {
        r_hand->coeff()(i-1) = r_hand->coeff()(i);
      }
      r_hand->coeff()(HAND_NUM_COEFF - 1) = 0;

#if defined(DEBUG) || defined(_DEBUG)
      std::cout << "right after" << std::endl;
      r_hand->printCoeff();
#endif

#if defined(DEBUG) || defined(_DEBUG)
      // At least make sure the inverse mapping and the conversion to matrix is
      // correct
      FloatQuat lquat_tmp;
      FloatQuat::eulerAngles2Quat(&lquat_tmp, leuler[0], leuler[1], leuler[2]);
      if (!lquat.approxEqual(&lquat_tmp)) {
        throw std::runtime_error("ERROR: Quat --> Euler is not correct!");
      }
      Float4x4 lmat;
      Float4x4 lmat2;
      lquat.quat2Mat4x4(&lmat);
      Float4x4::euler2RotMat(&lmat2, leuler[0], leuler[1], leuler[2]);
      if (!lmat.approxEqual(&lmat2)) {
        throw std::runtime_error("ERROR: Quat --> Euler is not correct!");
      }
      FloatQuat rquat_tmp;
      FloatQuat::eulerAngles2Quat(&rquat_tmp, reuler[0], reuler[1], reuler[2]);
      if (!rquat.approxEqual(&rquat_tmp)) {
        throw std::runtime_error("ERROR: Quat --> Euler is not correct!");
      }
      Float4x4 rmat;
      Float4x4 rmat2;
      rquat.quat2Mat4x4(&rmat);
      Float4x4::euler2RotMat(&rmat2, reuler[0], reuler[1], reuler[2]);
      if (!rmat.approxEqual(&rmat2)) {
        throw std::runtime_error("ERROR: Quat --> Euler is not correct!");
      }
#endif

      // Save the modified hand coefficients (to a new coeff file)
      saveModifiedHandCoeffs();
    }

    std::cout << "All done!" << std::endl;
#if defined(WIN32) || defined(_WIN32)
    system("PAUSE");
#endif
    quit();

  } catch (std::runtime_error e) {
    printf("%s\n", e.what());
#if defined(WIN32) || defined(_WIN32)
    system("PAUSE");
#endif
  }
  
  return 0;
}
