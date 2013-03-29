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
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector.h"
#include "jtil/clk/clk.h"
#include "jtil/string_util/string_util.h"
#include "kinect_interface/hand_net/hand_model.h"
#include "hand_fit/hand_geometry.h"
#include "hand_fit/hand_renderer.h"
#include "hand_fit/hand_fit.h"
#include "kinect_interface/depth_images_io.h"
#include "kinect_interface/open_ni_funcs.h"
#include "renderer/gl_state.h"
#include "kinect_interface/hand_net/hand_net.h"  // for HandCoeffConvnet
#include "kinect_interface/hand_net/hand_image_generator.h"
#include "kinect_interface/hand_detector/hand_detector.h"
#include "jtil/file_io/file_io.h"

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif

#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_1/")  // Added
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_2_1/")
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_2_2/")
//#define IM_DIR_BASE string("data/hand_depth_data_2013_01_11_3/")  // Added
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_4/")  // Added
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_5/")  // Added
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_6/")  // Added
//#define IM_DIR_BASE string("data/hand_depth_data_2013_03_04_7/")  // Added

#define DST_IM_DIR_BASE string("data/hand_depth_data_processed_for_CN/") 

#define SAVE_FILES  // Only enabled when we're not loading processed images
//#define LOAD_PROCESSED_IMAGES  // Load the images from the dst image directory
#define SAVE_HPF_IMAGES  // Save the hpf files
#define DESIRED_PLAYBACK_FPS 30.0f
#define FRAME_TIME (1.0f / DESIRED_PLAYBACK_FPS)

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
using namespace jtil::math;
using namespace jtil::data_str;
using namespace kinect_interface::hand_net;
using namespace kinect_interface::hand_detector;
using namespace kinect_interface;
using namespace hand_fit;
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

// Hand model
HandModel* l_hand = NULL;  // Not using this yet
HandModel* r_hand = NULL;
HandRenderer* hand_renderer = NULL;
HandFit* fit = NULL;
Eigen::MatrixXf coeffs;
uint8_t label[src_dim];

// Kinect Image data
DepthImagesIO* image_io = NULL;
VectorManaged<char*> im_files;
float cur_xyz_data[src_dim*3];
int16_t cur_depth_data[src_dim*3];
uint8_t cur_label_data[src_dim];
uint8_t cur_image_rgb[src_dim*3];
int32_t cur_image = 0;
GeometryColoredPoints* geometry_points= NULL;
bool render_depth = true;
int playback_step = 1;
float r_modified_coeff[HAND_NUM_COEFF];
float l_modified_coeff[HAND_NUM_COEFF];
Texture* tex = NULL;
uint8_t tex_data[HN_IM_SIZE * HN_IM_SIZE * 3];

// Decision forests
HandDetector* hand_detect = NULL;

// Hand Image Generator for the convnet
HandImageGenerator* hand_image_generator_ = NULL;
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
  delete fit;
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
  char* file = im_files[cur_image];
#ifdef LOAD_PROCESSED_IMAGES
  string DIR = DST_IM_DIR;
#else
  string DIR = IM_DIR;
#endif
  string full_filename = DIR + string(file);
  //std::cout << "loading image: " << full_filename << std::endl;

#ifdef LOAD_PROCESSED_IMAGES
  // Just load the processed image directly
  const float* im = hand_image_generator_->hand_image();
  jtil::file_io::LoadArrayFromFile<float>(const_cast<float*>(im),
    HN_IM_SIZE * HN_IM_SIZE, full_filename);
  memset(cur_depth_data, 0, src_dim * sizeof(cur_depth_data[0]));
  memset(cur_label_data, 0, src_dim * sizeof(cur_label_data[0]));
  memset(cur_image_rgb, 0, 3 * src_dim * sizeof(cur_image_rgb[0]));
  memset(cur_xyz_data, 0, 3 * src_dim * sizeof(cur_xyz_data[0]));

  string src_file = im_files[cur_image];
  src_file = src_file.substr(10, src_file.length());
  string r_coeff_file = DST_IM_DIR + string("coeffr_") + src_file;
  jtil::file_io::LoadArrayFromFile<float>(coeff_convnet,
    HandCoeffConvnet::HAND_NUM_COEFF_CONVNET, r_coeff_file);
#else
  // Load in the image
  image_io->LoadCompressedImage(full_filename, 
    cur_depth_data, cur_label_data, cur_image_rgb);
  DepthImagesIO::convertSingleImageToXYZ(cur_xyz_data, cur_depth_data);
  found_hand = hand_detect->findHandLabels(cur_depth_data, 
    cur_xyz_data, HDLabelMethod::HDFloodfill, label);

  // Load in the coeffs
  string src_file = im_files[cur_image];
#ifdef LOAD_PROCESSED_IMAGES
  src_file = src_file.substr(10, src_file.length());
#endif
  r_hand->loadFromFile(DIR, string("coeffr_") + src_file);
  // Don't bother loading it, it eats into our disk IO anyway.
  //l_hand->loadFromFile(DIR, string("coeffl_") + src_file);
  HandModel::scale = r_hand->local_scale();
  cout << "loaded image " << cur_image+1 << " of " << im_files.size() << endl;

  if (!found_hand) {
    // skip this data point
    std::cout << "Warning couldn't find hand in " << im_files[cur_image];
    std::cout << std::endl;
  }

  // Create the downsampled hand image, background is at 1 and hand is
  // from 0 --> 1.  Also calculates the convnet input.
#ifdef SAVE_HPF_IMAGES
  const bool create_hpf_images = true;
#else
  const bool create_hpf_images = false;
#endif
  hand_image_generator_->calcHandImage(cur_depth_data, label,
    create_hpf_images);

  // Correctly modify the coeff values to those that are learnable by the
  // convnet (for instance angles are bad --> store cos(x), sin(x) instead)
  hand_renderer->handCoeff2CoeffConvnet(r_hand, coeff_convnet,
    hand_image_generator_->hand_pos_wh(), hand_image_generator_->uvd_com());
#endif
}

void saveFrame() {
#if defined(SAVE_FILES) && !defined(LOAD_PROCESSED_IMAGES)
  if (found_hand) {
    // Save the cropped image to file:
    jtil::file_io::SaveArrayToFile<float>(hand_image_generator_->hand_image(),
      hand_image_generator_->size_images(), DST_IM_DIR + 
      string("processed_") + im_files[cur_image]);

    // Save the HPF images to file:
#if defined SAVE_HPF_IMAGES
    jtil::file_io::SaveArrayToFile<float>(hand_image_generator_->hpf_hand_images(),
      hand_image_generator_->size_images(), DST_IM_DIR + std::string("hpf_") + 
      im_files[cur_image]);
#endif

    string r_hand_file = DST_IM_DIR + string("coeffr_") + im_files[cur_image];
    string l_hand_file = DST_IM_DIR + string("coeffl_") + im_files[cur_image];
    jtil::file_io::SaveArrayToFile<float>(coeff_convnet,
      HandCoeffConvnet::HAND_NUM_COEFF_CONVNET, r_hand_file);
    // Don't save it...  It's just blank anyway and will eat into our disk IO
    //jtil::file_io::SaveArrayToFile<float>(blank_coeff,
    //  HandCoeffConvnet::HAND_NUM_COEFF_CONVNET, l_hand_file);
  }
#endif
}

int delete_confirmed = 0;
void keyboardCB(int key, int action) {
  string full_im_filename;
  string full_hpf_im_filename;
  string r_coeff_file;
  string l_coeff_file;
  string src_file;

  if (key != 'd' && key != 'D') {
    delete_confirmed = 0;
  }
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
    full_im_filename = DST_IM_DIR + string(im_files[cur_image]);
    full_hpf_im_filename = DST_IM_DIR + std::string("hpf_") + im_files[cur_image];
    src_file = im_files[cur_image];
    src_file = src_file.substr(10, src_file.length());
    r_coeff_file = DST_IM_DIR + string("coeffr_") + src_file;
    //l_coeff_file = DST_IM_DIR + string("coeffl_") + src_file;

    if (delete_confirmed == 1) {
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
      delete_confirmed = 0;
    } else {
      delete_confirmed++;
      cout << "About to delete files: " << endl;
      cout << "    - " << full_im_filename.c_str() << endl;
      cout << "    - " << r_coeff_file.c_str() << endl;
      //cout << "    - " << l_coeff_file.c_str() << endl;
      cout << "    - " << full_hpf_im_filename.c_str() << endl;
      cout << endl;
      cout << "Press 'd' again " << 2 - delete_confirmed;
      cout << " times to confirm" << endl;
    }
#else
    cout << "Delete function not implemented for Mac OS X" << endl;
#endif
    break;
#endif
    }
  }
  if (key == KEY_KP_ADD || key == KEY_KP_SUBTRACT || key == '0' || 
    key == '9') {
    loadCurrentImage();
  }
}

void renderFrame() {
  hand_renderer->setRendererAttachement(true);

  // Render the images for debugging
  const float* im = hand_image_generator_->hand_image();
  for (uint32_t v = 0; v < HN_IM_SIZE; v++) {
    for (uint32_t u = 0; u < HN_IM_SIZE; u++) {
      uint32_t val = (uint32_t)(im[v * HN_IM_SIZE + u] * 255.0f);
      // Clamp the value from 0 to 255 (otherwise we'll get wrap around)
      uint8_t val8 = (uint8_t)std::min<uint32_t>(std::max<uint32_t>(val,0),255);
      uint32_t idst = (HN_IM_SIZE-v-1) * HN_IM_SIZE + u;
      // Texture needs to be flipped vertically and 0 --> 255
      tex_data[idst * 3] = val8;
      tex_data[idst * 3 + 1] = val8;
      tex_data[idst * 3 + 2] = val8;
    }
  }

  hand_image_generator_->annotateFeatsToHandImage(tex_data, coeff_convnet);

  tex->reloadData((unsigned char*)tex_data);
  render->renderFullscreenQuad(tex);
  wnd->swapBackBuffer();
}

using std::cout;
using std::endl;
Float4x4 mat_tmp;
WindowSettings settings;

int main(int argc, char *argv[]) {
  static_cast<void>(argc); static_cast<void>(argv);
#if defined(_DEBUG) && defined(_WIN32)
  _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
  // _CrtSetBreakAlloc(7910);
#endif

  cout << "Usage:" << endl;
  cout << "p - Cycle through the frames continuously" << endl;
  cout << "+/- - Forward and back a frame" << endl;
  cout << "9/0 - Forward and back 100 frames" << endl;
  cout << "q/ESC - Quit" << endl;
  
  try {
    clk = new jtil::clk::Clk();
    t1 = clk->getTime();
    t0 = t1;
    
    // Initialize Windowing system
    Window::initWindowSystem();
    Texture::initTextureSystem();

    // Fill the settings structure
    settings.width = 1024;
    settings.height = 1024;
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
    render->init(eye_rot, eye_pos, settings.width, settings.height,
      -HAND_CAMERA_VIEW_PLANE_NEAR, -HAND_CAMERA_VIEW_PLANE_FAR, 
      HAND_CAMERA_FOV);
    
    // Load the Kinect data for fitting from file and process it
    image_io = new DepthImagesIO();
#ifdef LOAD_PROCESSED_IMAGES
    image_io->GetFilesInDirectory(im_files, DST_IM_DIR, true);
#else
    image_io->GetFilesInDirectory(im_files, IM_DIR, false);
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
    hand_renderer = new HandRenderer(render, fit_left, fit_right);
    fit = new HandFit(hand_renderer, num_hands);
    coeffs.resize(1, HAND_NUM_COEFF * num_hands);
    r_hand = new HandModel(HandType::RIGHT);
    l_hand = new HandModel(HandType::LEFT);

    std::cout << "Using a cropped source image of " << HN_SRC_IM_SIZE;
    std::cout << std::endl << "Final image size after processing is ";
    std::cout << (HN_IM_SIZE) << std::endl;

    hand_detect = new HandDetector();
    hand_detect->init(src_width, src_height, KINECT_HANDS_ROOT +
      FOREST_DATA_FILENAME);
    hand_image_generator_ = new HandImageGenerator(HN_DEFAULT_NUM_CONV_BANKS);
    for (uint32_t i = 0; i < HandCoeffConvnet::HAND_NUM_COEFF_CONVNET; i++) {
      blank_coeff[i] = 0.0f;
    }

    loadCurrentImage();
    tex = new Texture(GL_RGB8, HN_IM_SIZE, HN_IM_SIZE, GL_RGB, GL_UNSIGNED_BYTE, 
      (unsigned char*)tex_data, TEXTURE_WRAP_MODE::TEXTURE_CLAMP, false,
      TEXTURE_FILTER_MODE::TEXTURE_NEAREST);

    render->renderFrame(0);

    while (is_running) {
      clk = new jtil::clk::Clk();

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
