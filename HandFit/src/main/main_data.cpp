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
#include "renderer/texture/texture.h"
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

#define IM_DIR_BASE string("data/hand_depth_data_2_2/") 
#define DST_IM_DIR_BASE string("data/hand_depth_data_processed/") 

#define FILE_STRIDE 1
#define MAX_FILES MAX_UINT32
#define SAVE_FILES
// #define RENDER_FULL_DEPTH

#if defined(__APPLE__)
#define KINECT_HANDS_ROOT string("./../../../../../../../../../../")
#else
#define KINECT_HANDS_ROOT string("./../")
#endif

#define IM_DIR (KINECT_HANDS_ROOT + IM_DIR_BASE)
#define DST_IM_DIR (KINECT_HANDS_ROOT + DST_IM_DIR_BASE)

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
using renderer::Texture;
using renderer::TEXTURE_WRAP_MODE;
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
Texture* tex = NULL;
#ifdef RENDER_FULL_DEPTH
  #define TEX_W src_width
  #define TEX_H src_height
#else
  #define TEX_W HAND_NET_IM_SIZE
  #define TEX_H HAND_NET_IM_SIZE
#endif
uint8_t tex_data[TEX_W * TEX_H * 3];

// Decision forests
HandDetector* hand_detector = NULL;

// Convnet --> We'll mostly just use it's utility functions
HandNet* convnet = NULL;
float blank_coeff[HandCoeffConvnet::HAND_NUM_COEFF_CONVNET];
float coeff_convnet[HandCoeffConvnet::HAND_NUM_COEFF_CONVNET];

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
  delete tex;
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

// renderCrossToImageArr - UV is 0 to 1 in U and V
void renderCrossToImageArr(float* uv, uint8_t* im, int32_t w, int32_t h,
  int32_t rad, uint8_t r, uint8_t g, uint8_t b) {
#ifdef RENDER_FULL_DEPTH
  int32_t v = (int32_t)floor((uv[1] * HAND_NET_PIX) + (convnet->uvd_com()[1] - HAND_NET_PIX/2));
  int32_t u = (int32_t)floor((uv[0] * HAND_NET_PIX) + (convnet->uvd_com()[0] - HAND_NET_PIX/2));
#else
  int32_t v = (int32_t)floor(uv[1] * (float)h);
  int32_t u = (int32_t)floor(uv[0] * (float)w);
#endif
  v = h - v - 1;

  // Note: We need to render upside down
  // Render the horizontal cross
  int32_t vcross = v;
  for (int32_t ucross = u - rad; ucross <= u + rad; ucross++) {
    if (ucross >= 0 && ucross < w && vcross >= 0 && vcross < h) {
      int32_t dst_index = vcross * w + ucross;
      im[dst_index * 3] = r;
      im[dst_index * 3+1] = g;
      im[dst_index * 3+2] = b;
    }
  }
  // Render the vertical cross
  int32_t ucross = u;
  for (int32_t vcross = v - rad; vcross <= v + rad; vcross++) {
    if (ucross >= 0 && ucross < w && vcross >= 0 && vcross < h) {
      int32_t dst_index = vcross * w + ucross;
      im[dst_index * 3] = r;
      im[dst_index * 3+1] = g;
      im[dst_index * 3+2] = b;
    }
  }
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
    
    // Fill the settings structure
#ifdef RENDER_FULL_DEPTH    
    settings.width = src_width * 2;
    settings.height = src_height * 2;
#else
    settings.width = 1024;
    settings.height = 1024;
#endif
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
    tex = new Texture(GL_RGB8, TEX_W, TEX_H, GL_RGB, GL_UNSIGNED_BYTE, 
      (unsigned char*)tex_data, TEXTURE_WRAP_MODE::TEXTURE_CLAMP, false);
    
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

    hand_detector = new HandDetector(src_width, src_height, KINECT_HANDS_ROOT +
      FOREST_DATA_FILENAME);
    convnet = new HandNet();
    convnet->loadFromFile(CONVNET_FILE);
    for (uint32_t i = 0; i < HandCoeffConvnet::HAND_NUM_COEFF_CONVNET; i++) {
      blank_coeff[i] = 0.0f;
    }
    render->renderFrame(0);

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
      convnet->calcHandImage(cur_depth_data, label);

#ifdef SAVE_FILES
      // Save the cropped image to file:
      file_io::SaveArrayToFile<float>(convnet->hand_image(),
        convnet->size_images(), DST_IM_DIR + im_files[cur_image]);

      // Save the HPF images to file:
      file_io::SaveArrayToFile<float>(convnet->hpf_hand_images(),
        convnet->size_images(), DST_IM_DIR + std::string("hpf_") + 
        im_files[cur_image]);
#endif

      // Correctly modify the coeff values to those that are learnable by the
      // convnet (for instance angles are bad --> store cos(x), sin(x) instead)
      convnet->calcCoeffConvnet(r_hand, hand_renderer, coeff_convnet);
      string r_hand_file = DST_IM_DIR + string("coeffr_") + im_files[cur_image];
      string l_hand_file = DST_IM_DIR + string("coeffl_") + im_files[cur_image];
#ifdef SAVE_FILES
      file_io::SaveArrayToFile<float>(coeff_convnet,
        HandCoeffConvnet::HAND_NUM_COEFF_CONVNET, r_hand_file);
      file_io::SaveArrayToFile<float>(blank_coeff,
        HandCoeffConvnet::HAND_NUM_COEFF_CONVNET, l_hand_file);
#endif

      // Render the images for debugging
#ifdef RENDER_FULL_DEPTH
      int16_t* im = cur_depth_data;
      for (uint32_t v = 0; v < src_height; v++) {
        for (uint32_t u = 0; u < src_width; u++) {
          uint8_t val = (uint8_t)((im[v * src_width + u]) * 255.0f);
          uint32_t idst = (src_height-v-1) * src_width + u;
          // Texture needs to be flipped vertically and 0 --> 255
          tex_data[idst * 3] = val;
          tex_data[idst * 3 + 1] = val;
          tex_data[idst * 3 + 2] = val;
        }
      }
#else
      float* im = convnet->hpf_hand_images();
      for (uint32_t v = 0; v < HAND_NET_IM_SIZE; v++) {
        for (uint32_t u = 0; u < HAND_NET_IM_SIZE; u++) {
          uint32_t val = (uint32_t)((im[v * HAND_NET_IM_SIZE + u] + 0.5f) * 255.0f);
          // Clamp the value from 0 to 255 (otherwise we'll get wrap around)
          uint8_t val8 = (uint8_t)std::min<uint32_t>(std::max<uint32_t>(val,0),255);
          uint32_t idst = (HAND_NET_IM_SIZE-v-1) * HAND_NET_IM_SIZE + u;
          // Texture needs to be flipped vertically and 0 --> 255
          tex_data[idst * 3] = val8;
          tex_data[idst * 3 + 1] = val8;
          tex_data[idst * 3 + 2] = val8;
        }
      }
#endif

      renderCrossToImageArr(&coeff_convnet[HandCoeffConvnet::HAND_POS_U], 
        tex_data, TEX_W, TEX_H, 5, 255, 128, 255);
      for (uint32_t i = HandCoeffConvnet::THUMB_K1_U; 
        i <= HandCoeffConvnet::F3_TIP_U; i += 2) {
        const Float3* color = &renderer::colors[(i/2) % renderer::n_colors];
        renderCrossToImageArr(&coeff_convnet[i], tex_data, TEX_W, TEX_H, 2, 
          (uint8_t)(color->m[0] * 255.0f), (uint8_t)(color->m[1] * 255.0f), 
          (uint8_t)(color->m[2] * 255.0f));
      }

      tex->reloadData((unsigned char*)tex_data);
      render->renderFullscreenQuad(tex);
      wnd->swapBackBuffer();
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
