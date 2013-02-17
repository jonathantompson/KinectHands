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
#include "hand_detector.h"
#include "file_io/file_io.h"

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif

//#define IM_DIR_BASE string("hand_data/left_only/set05/")
//#define IM_DIR_BASE string("hand_data/right_only/set05/")
// 3 -> Finished (4 partially finished)
//#define IM_DIR_BASE string("hand_data/both_hands/set03/") 

#define IM_DIR_BASE string("/data/hand_depth_data/") 
#define DST_IM_DIR_BASE string("/data/hand_depth_data_processed/") 

#define FILE_STRIDE 1
#define MAX_FILES MAX_UINT32

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
using math::Float2;
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
using hand_net::HandCoeffConvnet;

Clock* clk = NULL;
double t1, t0;

// The main window and basic rendering system
Window* wnd = NULL;
Renderer* render = NULL;

// Hand model
HandModel* l_hand = NULL;  // Not using this yet
HandModel* r_hand = NULL;
HandModelRenderer* hand_renderer = NULL;
HandModelFit* hand_fit = NULL;
Eigen::MatrixXf coeffs;
float depth[src_dim];
uint8_t label[src_dim];

// Kinect Image data
DepthImagesIO* image_io = NULL;
data_str::VectorManaged<char*> im_files;
float cur_xyz_data[src_dim*3];
int16_t cur_depth_data[src_dim*3];
uint8_t cur_label_data[src_dim];
uint8_t cur_image_rgb[src_dim*3];
uint32_t cur_image = 0;
GeometryColoredPoints* geometry_points= NULL;
bool render_depth = true;
int playback_step = 1;
float r_modified_coeff[HAND_NUM_COEFF];
float l_modified_coeff[HAND_NUM_COEFF];

// Decision forests
HandDetector* hand_detector = NULL;

// Convnet --> We'll mostly just use it's utility functions
HandNet* convnet = NULL;
float blank_coeff[HandCoeffConvnet::HAND_NUM_COEFF_CONVNET];

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
  delete hand_detector;
  delete convnet;
  Window::killWindowSystem();
  exit(0);
}

void loadCurrentImage() {
  char* file = im_files[cur_image];
  string full_filename = IM_DIR + string(file);
  std::cout << "loading image: " << full_filename << std::endl;

  image_io->LoadCompressedImage(full_filename, 
    cur_depth_data, cur_label_data, cur_image_rgb);
  DepthImagesIO::convertSingleImageToXYZ(cur_xyz_data, cur_depth_data);
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

    hand_detector = new HandDetector(src_width, src_height, string("./../") +
      FOREST_DATA_FILENAME);
    convnet = new HandNet(CONVNET_FILE);
    for (uint32_t i = 0; i < HandCoeffConvnet::HAND_NUM_COEFF_CONVNET; i++) {
      blank_coeff[i] = 0.0f;
    }

    // Iterate through, saving each of the data points
    for (uint32_t i = 0; i < std::min<uint32_t>(im_files.size(), MAX_FILES); 
      i += FILE_STRIDE) {
      std::cout << "processing image " << (i/FILE_STRIDE) << " of ";
      std::cout << (im_files.size()/FILE_STRIDE) << std::endl;

      // Load in the image (rgb + depth) and load in the fitted coefficients
      cur_image = i;
      loadCurrentImage();

      r_hand->loadFromFile(IM_DIR, string("coeffr_") + im_files[cur_image]);
      l_hand->loadFromFile(IM_DIR, string("coeffl_") + im_files[cur_image]);
      HandModel::scale = r_hand->local_scale();
      hand_renderer->setRendererAttachement(true);

      for (uint32_t i = 0; i < src_dim; i++) {
        depth[i] = (float)cur_depth_data[i];
      }
      bool found_hand = hand_detector->findHandLabels(cur_depth_data, 
        cur_xyz_data, HDLabelMethod::HDFloodfill, label);

      if (!found_hand) {
        // skip this data point
        std::cout << "Warning couldn't find hand in " << im_files[cur_image];
        std::cout << std::endl;
        continue;
      }

      // Create the downsampled hand image, background is at 1 and hand is
      // from 0 --> 1.  Also calculates the convnet input.
      convnet->calcHandImage(depth, label);

      // Save the cropped image to file:
      file_io::SaveArrayToFile<float>(convnet->hpf_hand_image(),
        convnet->size_hpf_hand_image(), DST_IM_DIR + im_files[cur_image]);

      // Correctly modify the coeff values to those that are learnable by the
      // convnet (for instance angles are bad --> store cos(x), sin(x) instead)
      convnet->calcCoeffConvnet(r_hand->coeff());
      string r_hand_file = string("coeffr_") + im_files[cur_image];
      string l_hand_file = string("coeffl_") + im_files[cur_image];
      file_io::SaveArrayToFile<float>(convnet->coeff_convnet(),
        HandCoeffConvnet::HAND_NUM_COEFF_CONVNET, r_hand_file);
      file_io::SaveArrayToFile<float>(blank_coeff,
        HandCoeffConvnet::HAND_NUM_COEFF_CONVNET, r_hand_file);
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
