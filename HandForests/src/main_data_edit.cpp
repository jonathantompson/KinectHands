//
//  main.cpp
//
//  Created by Jonathan Tompson on 7/20/12.
// 

#if defined(WIN32) || defined(_WIN32)
  #include <Windows.h>
  #include <glut.h>
#elif defined(__APPLE__)
  #include <GLUT/GLUT.h>
#else
  #define FOREST_CREATION_ONLY
#endif
#include <signal.h>
#include <stdexcept>
#include <string>
#include <iostream>
#include <thread>
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector_managed.h"
#include "kinect_interface/depth_images_io.h"
#include "kinect_interface/hand_detector/forest_io.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"  // MAX_DIST
#include "jtil/string_util/string_util.h"
#include "jtil/image_util/image_util.h"
#include "jtil/clk/clk.h"
#include "jtil/threading/callback.h"
#include "jtil/threading/thread.h"
#include "jtil/debug_util/debug_util.h"  // Must come last in main.cpp

using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using namespace jtil;
using namespace jtil::image_util;
using namespace kinect_interface;
using namespace kinect_interface::hand_detector;

#define LOAD_PROCESSED_IMAGES
//#define SAFE_FLIPPED
#define DISABLE_ALL_SAVES

//#define IMAGE_DIRECTORY_BASE string("data/hand_depth_data_2012_07_27_and_08_03_DFProcessed/")
//#define IMAGE_DIRECTORY_BASE string("data/hand_depth_data_2013_01_11_1/")
//#define IMAGE_DIRECTORY_BASE string("data/hand_depth_data_2013_01_11_2_1/")
//#define IMAGE_DIRECTORY_BASE string("data/hand_depth_data_2013_01_11_2_2/")
//#define IMAGE_DIRECTORY_BASE string("data/hand_depth_data_2013_01_11_3/")
//#define IMAGE_DIRECTORY_BASE string("data/hand_depth_data_2013_03_04_4/")
//#define IMAGE_DIRECTORY_BASE string("data/hand_depth_data_2013_03_04_5/")
// NOTE: hand_depth_data_2013_03_04_6 and _7 need cleanUpRedPixelsUsingDepth Hack!
//#define IMAGE_DIRECTORY_BASE string("data/hand_depth_data_2013_03_04_6/")
//#define IMAGE_DIRECTORY_BASE string("data/hand_depth_data_2013_03_04_7/") 

#define IMAGE_DIRECTORY_BASE string("data/hand_depth_data_processed_for_DF/") 

#if defined(__APPLE__)
  #define IMAGE_DIRECTORY string("./../../../../../../") + IMAGE_DIRECTORY_BASE
#else
  #define IMAGE_DIRECTORY string("./../") + IMAGE_DIRECTORY_BASE
#endif

jtil::clk::Clk clk_;
double t1; double t0;
#define PLAY_SPEED (1.0/100000.0)  // Escentially as fast as possible

// DATA VARIABLES
data_str::VectorManaged<char*> im_files;
int32_t cur_image = 0;
DepthImagesIO* image_io = NULL;
#ifndef LOAD_PROCESSED_IMAGES
  const int32_t im_stride = 30;
#else
  const int32_t im_stride = 1;
#endif

// The current image being worked on
int16_t cur_depth_data[src_dim*3];
int16_t cur_depth_data_tmp[src_dim*3];
int16_t cur_depth_data_flipped[src_dim*3];
uint8_t cur_label_data[src_dim];
uint8_t cur_label_data_tmp[src_dim];
uint8_t cur_label_data_flipped[src_dim];
uint8_t cur_redlabel_data[src_dim];
uint8_t cur_image_rgb[src_dim*3];
uint8_t cur_image_hsv[src_dim*3];

// OPEN GL VARIABLES
int main_window;
uint8_t cur_display_rgb[src_dim*3];
uint32_t window_width = 3 * src_width;
uint32_t window_height = 3 * src_height;
const int num_textures = 2;
GLuint textures[num_textures];
unsigned char *texture_data;
IM_TYPE render_image_type = IM_TYPE::IM_DEPTH;  // Leave as depth
bool render_labels = true;
uint32_t label_type = 0;
bool continuously_play = false;

void shutdown();

// Glut registered callbacks and other functions
void idle();
void display();
void keyboard(unsigned char key, int x, int y);
void reshape(const int width, const int height);
void motion(const int x, const int y) { }
void mouse(int button, int state, int x, int y);
void mouseMotion(int x, int y) { }
void initGL();
void initTextures();

void idle() {
  glutSetWindow(main_window);
  glutPostRedisplay();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));  // let someone else do some work
}

void loadImageForRendering(bool force_reprocessing = false) {
  string full_filename = IMAGE_DIRECTORY + string(im_files[cur_image]);

#ifndef LOAD_PROCESSED_IMAGES
  image_io->LoadCompressedImageWithRedHands(full_filename, 
    cur_depth_data, cur_label_data, cur_image_rgb, cur_redlabel_data,
    cur_image_hsv);
#else
  image_io->loadProcessedDepthLabel(full_filename, cur_depth_data, 
    cur_label_data);
#endif

  // When rendering HSV, Hue, Sat or Val, copy over the current rgb array with
  // what we want to render to screen.
  switch (render_image_type) {
  case IM_TYPE::IM_RGB:
    memcpy(cur_display_rgb, cur_image_rgb, src_dim * sizeof(cur_display_rgb[0]) * 3);
    break;
  case IM_TYPE::IM_HSV:
    memcpy(cur_display_rgb, cur_image_hsv, src_dim * sizeof(cur_display_rgb[0]) * 3);
    break;
  case IM_TYPE::IM_HUE:
    convertHueToRGB<uint8_t>(cur_display_rgb, cur_image_hsv, src_width, src_height);
    break;
  case IM_TYPE::IM_SAT:
    convertSaturationToRGB<uint8_t>(cur_display_rgb, cur_image_hsv, src_width, src_height);
    break;
  case IM_TYPE::IM_VAL:
    convertValueToRGB<uint8_t>(cur_display_rgb, cur_image_hsv, src_width, src_height);
    break;
  case IM_TYPE::IM_DOWNSAMPLED_DEPTH:
    DownsampleImageWithoutNonZeroPixelsAndBackground<int16_t>(
      cur_depth_data_tmp, cur_depth_data, src_width, src_height, DT_DOWNSAMPLE,
      GDT_MAX_DIST);
    UpsampleNoFiltering<int16_t>(cur_depth_data, cur_depth_data_tmp,
      src_width / DT_DOWNSAMPLE, src_height / DT_DOWNSAMPLE, DT_DOWNSAMPLE);
    DownsampleBoolImageConservative<uint8_t>(
      cur_label_data_tmp, cur_label_data, src_width, src_height, DT_DOWNSAMPLE,
      0, 1);
    UpsampleNoFiltering<uint8_t>(cur_label_data, cur_label_data_tmp,
      src_width / DT_DOWNSAMPLE, src_height / DT_DOWNSAMPLE, DT_DOWNSAMPLE);
    break;
  }
}

void saveData() {
#ifndef DISABLE_ALL_SAVES
  string full_filename = IMAGE_DIRECTORY + string(im_files[cur_image]);
  image_io->saveProcessedDepthLabel(full_filename, cur_depth_data, 
    cur_label_data);
#ifdef SAFE_FLIPPED
  string name_dir, name_file;
  DepthImagesIO::extractDirFile(full_filename, name_dir, name_file);
  if (name_file.substr(0, 24) != string("processed_hands_flipped_")) {
    // a flipped version doesn't exist
    for (uint32_t v = 0; v < src_height; v++) {
      for (uint32_t u = 0; u < src_width; u++) {
        cur_depth_data_flipped[v * src_width + (src_width - u - 1)] = 
          cur_depth_data[v * src_width + u];
        cur_label_data_flipped[v * src_width + (src_width - u - 1)] = 
          cur_label_data[v * src_width + u];
      }
    }
    if (name_file.substr(0, 16) == string("processed_hands_")) {
      name_file = string("processed_hands_flipped_") + 
        name_file.substr(16, name_file.length());
    } else {
      name_file = string("processed_hands_flipped_") + name_file;
    }
    string flipped_file = name_dir + name_file;
    image_io->saveProcessedDepthLabel(flipped_file, cur_depth_data_flipped,
      cur_label_data_flipped);
  } else {
    std::cout << name_file << " is already a flipped file" << std::endl;
  }
#endif
#endif
}

const uint32_t num_colors = 5;
const float colors[num_colors][3] = {
  {1, 1, 1}, 
  {0, 1, 0}, 
  {1, 1, 0}, 
  {1, 0, 1}, 
  {0, 1, 1}};
const float min_color_scale = 50.0f;
const float max_color_scale = 255.0f;

template <class T>
void UpdateGreyscaleImageForRendering(T* src) {
  for (int32_t i = 0; i < src_dim; i++) {
    if (src[i] < GDT_MAX_DIST && src[i] != 0) {
      uint32_t cur_pixel = static_cast<uint32_t>(src[i]) << 2;
      float cur_color_scale = static_cast<float>(cur_pixel % 255) / 255; // 0 --> 1
      cur_color_scale = min_color_scale + (max_color_scale - min_color_scale) * cur_color_scale;
      uint32_t cur_color = (cur_pixel / 255) % num_colors;

      texture_data[4*i] = static_cast<uint8_t>(colors[cur_color][0] * cur_color_scale);
      texture_data[4*i+1] = static_cast<uint8_t>(colors[cur_color][1] * cur_color_scale);
      texture_data[4*i+2] = static_cast<uint8_t>(colors[cur_color][2] * cur_color_scale);
      texture_data[4*i+3] = 255;
    } else if (src[i] >= GDT_MAX_DIST) {
      texture_data[4*i] = 0;
      texture_data[4*i+1] = 0;
      texture_data[4*i+2] = 255;
      texture_data[4*i+3] = 255;
    } else if (src[i] == 0) {
      texture_data[4*i] = 0;
      texture_data[4*i+1] = 0;
      texture_data[4*i+2] = 0;
      texture_data[4*i+3] = 255;
    }
  }
}

template <class T>
void PaintLabelData(T* src) {
  for (int32_t i = 0; i < src_dim; i++) {
    switch(src[i]) {
    case 0:
      // texture_data[4*i+3] = 128;  // dim everything else slightly
      break;
    case 1:
      texture_data[4*i] = 255;
      texture_data[4*i+1] = 0;
      texture_data[4*i+2] = 0;
      texture_data[4*i+3] = 255;
      break;
    case 2:
      texture_data[4*i] = 0;
      texture_data[4*i+1] = 0;
      texture_data[4*i+2] = 255;
      texture_data[4*i+3] = 255;
      break;
    }
  }
}

uint64_t frame_counter = 0;
double time_accumulate = 0;
void display() {
  t0 = t1;
  t1 = clk_.getTime();

  if (continuously_play) {
    saveData();
    if (cur_image >= (int32_t)im_files.size() - im_stride) {
      continuously_play = false;
      time_accumulate = 0;
    } else {
      time_accumulate += t1 - t0;
      if (time_accumulate > PLAY_SPEED) {
        cur_image += im_stride;
        std::cout << "loading image " << cur_image << " of ";
        std::cout << im_files.size() << std::endl;
        loadImageForRendering();
        time_accumulate = 0;
      }
    }
  }

  memset(texture_data, 0, src_dim * sizeof(texture_data[0]) * 4);

  if (render_image_type == IM_TYPE::IM_DEPTH ||
    render_image_type == IM_TYPE::IM_DOWNSAMPLED_DEPTH) {
    // Render a texture here
    UpdateGreyscaleImageForRendering<int16_t>(cur_depth_data);
  }

  if (render_labels) {
    switch (label_type) {
    case 0:
      PaintLabelData<uint8_t>(cur_label_data);
      break;
    case 1:
      PaintLabelData<uint8_t>(cur_redlabel_data);
      break;
    }
  }
  
  // Copy texture data to OpenGL
  glBindTexture(GL_TEXTURE_2D, textures[0]);
  glTexImage2D(GL_TEXTURE_2D,  0, GL_RGBA, src_width, src_height, 0, GL_RGBA, 
    GL_UNSIGNED_BYTE, texture_data);
  if (render_image_type != IM_TYPE::IM_DEPTH) {
    glBindTexture(GL_TEXTURE_2D, textures[1]);
    glTexImage2D(GL_TEXTURE_2D,  0, GL_RGB, src_width, src_height, 0, GL_RGB, 
                 GL_UNSIGNED_BYTE, cur_display_rgb);
  }
  
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, window_width, window_height);

  // Draw a single texture
  glEnable(GL_TEXTURE_2D);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, 1, 0, 1);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  
  if (render_image_type != IM_TYPE::IM_DEPTH) {
    glBindTexture(GL_TEXTURE_2D, textures[1]);
    glColor3f(1,1,1);
    glBegin(GL_QUADS);
    glTexCoord2d(0.0,1.0); glVertex2d(0.0,0.0);
    glTexCoord2d(1.0,1.0); glVertex2d(1.0,0.0);
    glTexCoord2d(1.0,0.0); glVertex2d(1.0,1.0);
    glTexCoord2d(0.0,0.0); glVertex2d(0.0,1.0);
    glEnd();
  }

  // Select the texture based on the image mode we are currently on
  glBindTexture(GL_TEXTURE_2D, textures[0]);

  glColor3f(1,1,1);
  glBegin(GL_QUADS);
  glTexCoord2d(0.0,1.0); glVertex2d(0.0,0.0);
  glTexCoord2d(1.0,1.0); glVertex2d(1.0,0.0);
  glTexCoord2d(1.0,0.0); glVertex2d(1.0,1.0);
  glTexCoord2d(0.0,0.0); glVertex2d(0.0,1.0);
  glEnd();

  glutSwapBuffers();
}

void initGL() {
  glDisable(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST); 

  texture_data = new unsigned char[src_dim * 4];

  // Initialize textures
  glEnable(GL_TEXTURE_2D);
  glGenTextures(num_textures, textures);
  
  // Set the texture parameters
  for (int i = 0; i < num_textures; i++) {
    glBindTexture(GL_TEXTURE_2D, textures[i]); 
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  }
}

int delete_confirmed = 0;
std::string cur_filename;
void keyboard(unsigned char key, int x, int y) {
  string full_filename = IMAGE_DIRECTORY + string(im_files[cur_image]);
  if (key != 'd') {
    delete_confirmed = 0;
  }
  bool image_changed = false;
  switch (key) {
  case 27: // ESC
  case 'q':
  case 'Q':
    shutdown();
    break;
  case '-':
  case '_':
    if (cur_image > 0) {
      cur_image -= im_stride;
      loadImageForRendering();
    }
    cout << "depth image " << cur_image+1 << " of " << im_files.size() << endl;
    break;
  case '=':
  case '+':
    if (cur_image < static_cast<int32_t>(im_files.size())-im_stride) {
      saveData();
      cur_image += im_stride;
      loadImageForRendering();
    }
    cout << "depth image " << cur_image+1 << " of " << im_files.size() << endl;
    break;
  case 'D':
  case 'd':
#if defined(WIN32) || defined(_WIN32)
    cur_filename = IMAGE_DIRECTORY;
    cur_filename += string(im_files[cur_image]);
    if (delete_confirmed == 1) {
      if(!DeleteFile(cur_filename.c_str())) {
        cout << "Error deleting file: " << cur_filename.c_str() << endl;
      } else {
        cout << "File deleted sucessfully: " << cur_filename.c_str() << endl;
        im_files.deleteAtAndShift((uint32_t)cur_image);
        loadImageForRendering();
      }
      delete_confirmed = 0;
    } else {
      delete_confirmed++;
      cout << "About to delete file " << cur_filename.c_str() << "!" << endl;
      cout << "Press 'd' again " << 2 - delete_confirmed << " times to confirm" << endl;
    }
#else
    cout << "Delete function not implemented for Mac OS X" << endl;
#endif
    break;
  case 'R':
  case 'r':
    render_image_type = (IM_TYPE)((render_image_type + 1) % IM_TYPE::IM_NUM_TYPES);
    cout << "render_image_type = " << render_image_type << endl;
    loadImageForRendering();
    break;
  case 'L':
  case 'l':
    render_labels = !render_labels;
    if (render_labels) {
      cout << "render_labels = true" << endl;
    } else {
      cout << "render_labels = false" << endl;
    }
    break;
  case 'K':
  case 'k':
    label_type = (label_type + 1) % 2;
    cout << "label_type = " << label_type << endl;
    break;
  case 'S':
  case 's':
    saveData();
    break;
  case 'U':
  case 'u':
    image_changed = true;
    break;
  case 'P':
  case 'p':
    continuously_play = !continuously_play;
    break;
  case '[':
  case '{':
    cur_image = 0;
    loadImageForRendering();
    break;
  }      
  
  switch (key) {
    case '!':
      DepthImagesIO::red_hue_threshold = (DepthImagesIO::red_hue_threshold + 1) % 255;
      break;
    case '1':
      DepthImagesIO::red_hue_threshold = (DepthImagesIO::red_hue_threshold == 0) ? 255 : DepthImagesIO::red_hue_threshold - 1;
      break;      
    case '@':
      DepthImagesIO::red_sat_threshold = (DepthImagesIO::red_sat_threshold + 1) % 255;
      break;
    case '2':
      DepthImagesIO::red_sat_threshold = (DepthImagesIO::red_sat_threshold == 0) ? 255 : DepthImagesIO::red_sat_threshold - 1;
      break;         
    case '#':
      DepthImagesIO::red_val_threshold = (DepthImagesIO::red_val_threshold + 1) % 255;
      break;
    case '3':
      DepthImagesIO::red_val_threshold = (DepthImagesIO::red_val_threshold == 0) ? 255 : DepthImagesIO::red_val_threshold - 1;
      break;   
    case '$':
      DepthImagesIO::red_hue_target = (DepthImagesIO::red_hue_target + 1) % 255;
      break;
    case '4':
      DepthImagesIO::red_hue_target = (DepthImagesIO::red_hue_target == 0) ? 255 : DepthImagesIO::red_hue_target - 1;
      break;
    case '%':
      DepthImagesIO::red_sat_target = (DepthImagesIO::red_sat_target + 1) % 255;
      break;
    case '5':
      DepthImagesIO::red_sat_target = (DepthImagesIO::red_sat_target == 0) ? 255 : DepthImagesIO::red_sat_target - 1;
      break;
    case '^':
      DepthImagesIO::red_val_target = (DepthImagesIO::red_val_target + 1) % 255;
      break;
    case '6':
      DepthImagesIO::red_val_target = (DepthImagesIO::red_val_target == 0) ? 255 : DepthImagesIO::red_val_target - 1;
      break;
    case '&':
      DepthImagesIO::red_red_min = (DepthImagesIO::red_red_min + 1) % 255;
      break;
    case '7':
      DepthImagesIO::red_red_min = (DepthImagesIO::red_red_min == 0) ? 255 : DepthImagesIO::red_red_min - 1;
      break;
    case '*':
      DepthImagesIO::hsv_total_threshold = (DepthImagesIO::hsv_total_threshold + 1) % 765;
      break;
    case '8':
      DepthImagesIO::hsv_total_threshold = (DepthImagesIO::hsv_total_threshold == 0) ? 765 : DepthImagesIO::hsv_total_threshold - 1;
      break;
  }
  
  if (key == '!' || key == '1' || key == '@' || key == '2' ||
      key == '#' || key == '3' || key == '$' || key == '4' ||
      key == '%' || key == '5' || key == '^' || key == '6' ||
      key == '&' || key == '7' || key == '*' || key == '8') {
    image_changed = true;
    cout << "RED HSV Thesholds = <" << DepthImagesIO::red_hue_threshold << ", ";
    cout << DepthImagesIO::red_sat_threshold << ", " << DepthImagesIO::red_val_threshold << ">" << endl;
    cout << "RED HSV Targets = <" << DepthImagesIO::red_hue_target << ", ";
    cout << DepthImagesIO::red_sat_target << ", " << DepthImagesIO::red_val_target << ">" << endl;
    cout << "RED Minimum R-channel value = " << DepthImagesIO::red_red_min << endl;
    cout << "HSV Total threshold = " << DepthImagesIO::hsv_total_threshold << endl << endl;
  }   
  
  if (image_changed) {
    loadImageForRendering(true);
  }
}

void mouse(int button, int state, int x, int y) {
  float pos_window_x = static_cast<float>(x) / static_cast<float>(window_width);
  float pos_window_y = static_cast<float>(y) / static_cast<float>(window_height);
  int32_t u = static_cast<float>(floorf(pos_window_x * static_cast<float>(src_width)));
  int32_t v = static_cast<float>(floorf(pos_window_y * static_cast<float>(src_height))); 
  uint32_t index = v * (src_width) + u;

  if (render_image_type == IM_TYPE::IM_DEPTH && state == GLUT_DOWN) {
    if (u < src_width && v < src_height && u >= 0 && v >= 0) {
      if (button == GLUT_LEFT_BUTTON) {
        if (cur_depth_data[index] != 0 && cur_depth_data[index] < GDT_MAX_DIST) {
          cout << "flooding label pixel <" << u << ", ";
          cout << v << ">" << endl;
          image_io->floodPixel(cur_label_data, cur_depth_data, u, v, 200, 16);
        } else {
          cout << "Can't flood label pixel <" << u << ", ";
          cout << v << ">" << endl;
        }
      } else if (button == GLUT_RIGHT_BUTTON) {
        if (cur_depth_data[index] != 0 && cur_depth_data[index] < GDT_MAX_DIST) {
          cout << "flooding label pixel <" << u << ", ";
          cout << v << ">" << endl;
          image_io->floodPixel(cur_label_data, cur_depth_data, u, v, 100, 5);
        } else {
          cout << "Can't flood label pixel <" << u << ", ";
          cout << v << ">" << endl;
        }
      }
      cout << "Depth Val = " << static_cast<int>(cur_depth_data[index]) << endl;
    }
  } else if (state == GLUT_DOWN) {
    cout << "RGB Val = <" << static_cast<int>(cur_image_rgb[3*index]);
    cout << ", " << static_cast<int>(cur_image_rgb[3*index+1]);
    cout << ", " << static_cast<int>(cur_image_rgb[3*index+2]) << ">" << endl;
    cout << "HSV Val = <" << static_cast<int>(cur_image_hsv[3*index]);
    cout << ", " << static_cast<int>(cur_image_hsv[3*index+1]);
    cout << ", " << static_cast<int>(cur_image_hsv[3*index+2]) << ">" << endl;
    cout << "Depth Val = " << static_cast<int>(cur_depth_data[index]) << endl;
    image_io->testRedPixel(index, cur_image_hsv, cur_image_rgb);
    cout << endl;
  }
}

void reshape(const int width, const int height) {
  window_width = width;
  window_height = height;
  glutPostRedisplay();
}

void shutdown() {
  delete image_io;
  delete[] texture_data;
  exit(0);
}

int main(int argc, char *argv[]) { 
  static_cast<void>(argc);  // Get rid of unused variable warning
  static_cast<void>(argv);

  cout << "USAGE:" << endl;
  cout << "q - Quit" << endl;
  cout << "d - Delete current image" << endl;
  cout << "r - Change render output" << endl;
  cout << "l - Render labels on/off" << endl;
  cout << "k - Switch between label types (0 - output label, 1 - red pixels, 2 - graph cut pixels)" << endl;
  cout << "s - Save image to file" << endl;
  cout << "u - Undo (reload image)" << endl;
  cout << "+ - Go to the next image (and save labeling to file)" << endl;
  cout << "- - Go to the prev image" << endl;
  cout << "p - Play images and save labeling to file" << endl;
  cout << "[ - Go back to the start image" << endl;
  cout << "mouse-left - Flood fill inverse label large (when rendering depth)" << endl;
  cout << "mouse-right - Flood fill inverse label small (when rendering depth)" << endl << endl;
  cout << "1/! - Increase/Decrease Red Hue Threshold" << endl;
  cout << "2/@ - Increase/Decrease Red Saturation Threshold" << endl;
  cout << "3/# - Increase/Decrease Red Value Threshold" << endl;
  cout << "4/$ - Increase/Decrease Red Hue Target" << endl;
  cout << "5/% - Increase/Decrease Red Saturation Target" << endl;
  cout << "6/^ - Increase/Decrease Red Value Target" << endl;
  cout << "7/& - Increase/Decrease Red R-Channel Minimum" << endl << endl;

#ifdef _DEBUG
  debug::EnableMemoryLeakChecks();
  // debug::SetBreakPointOnAlocation(125714);
#endif
  try {
    if (DT_DOWNSAMPLE < 1) {
      cout << "ERROR: DT_DOWNSAMPLE < 1!" << endl;
      return -1;
    }

    image_io = new DepthImagesIO();
#ifdef LOAD_PROCESSED_IMAGES
    bool load_processed_images = true;
    render_image_type = IM_TYPE::IM_DEPTH;
#else
    bool load_processed_images = false;
#endif
    if (image_io->GetFilesInDirectory(im_files, IMAGE_DIRECTORY, 
      load_processed_images) == 0) {
      throw std::runtime_error("No image files in directory!");
    }
    loadImageForRendering();

    glutInit(&argc, argv);
    glutInitWindowSize(window_width, window_height);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    main_window = glutCreateWindow("Mesh Deformation Test");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(idle);
    glutMotionFunc(mouseMotion);

    initGL();

    t1 = clk_.getTime();
    glutMainLoop();

  } catch(runtime_error e) {
    printf("std::runtime_error caught!:\n");
    printf("  %s\n", e.what());
#if defined(WIN32) || defined(_WIN32)
    system("PAUSE");
#endif
  } catch(std::bad_alloc e) {
    printf("std::bad_alloc caught! --> Likely not enough memory!:\n");
#if defined(WIN32) || defined(_WIN32)
    system("PAUSE");
#endif
  }
}
