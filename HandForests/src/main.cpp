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
#include "math/math_types.h"
#include "depth_images_io.h"
#include "evaluate_decision_forest.h"
#include "generate_decision_tree.h"
#include "common_tree_funcs.h"
#include "forest_io.h"
#include "string_util/string_util.h"
#include "image_util.h"
#include "clock/clock.h"
#include "threading/callback.h"
#include "threading/thread.h"
#include "load_settings_from_file.h"
#include "debug_util/debug_util.h"  // Must come last in .cpp that includes main

using std::string;
using std::runtime_error;
using std::cout;
using std::endl;

// #define FOREST_CREATION_ONLY

typedef enum {
  CorrectLabeling = 0,
  ForrestLabeling = 1,
  NumResults = 2
} CurResult;

#if defined(__APPLE__)
  #define IMAGE_DIRECTORY string("../../../../../../hand_depth_data/")
  #define FOREST_DATA_FILENAME string("../../../../../../forest_data.bin")
  #define PROGRAM_SETTINGS_FILENAME string("../../../../../../program_settings.csv")
#else
  #define IMAGE_DIRECTORY string("./hand_depth_data/")
  #define FOREST_DATA_FILENAME string("./forest_data.bin")
  #define PROGRAM_SETTINGS_FILENAME string("./program_settings.csv")
#endif

// DATA VARIABLES
const bool load_processed_images = true;
const uint32_t file_skip = 4;  // 1 = every file, 4 = every 1 in 4 files 
                               // --> Typically, recoreded stream is skipped by 6, sometimes 4
const float frac_test_data = 0.05;  // 5% of data files will be test data, def > 0.05
ImageData* training_data;
ImageData* test_data;
uint8_t* label_data_evaluated = NULL;
uint8_t* label_data_filtered = NULL;
bool filter_results = false;
uint32_t filter_results_radius = 2;
int32_t total_num_images;

// DECISION TREE VARIABLES
ProgramSettings prog_settings;
DecisionTree* forest;
TrainingSettings* settings;
WLSet wl_set;
const uint32_t num_threshold_vals = 41;
int16_t threshold_vals[num_threshold_vals] = 
{ 0,
  1,  2,  3,  4,  5,  6,  8,  10,  12,  15,  25,  50,  75,  100,  250,  500, 
  750,  1000,  1250,  1500,
 -1, -2, -3, -4, -5, -6, -8, -10, -12, -15, -25, -50, -75, -100, -250, -500, 
  750, -1000, -1250, -1500 };
const uint32_t num_uv_offset_vals = 35;
// offset is divided by depth!, so 1000 is 1 pixel offset at 1m
int32_t uv_offset_vals[num_uv_offset_vals] = 
{ 0, 
  398,  631,  1000,  1585,  2512,  3981,  6310,  10000,  15850,  25120,  39810, 
  63100,  100000,  158500,  251200,  398100,  631000, 
 -398, -631, -1000, -1585, -2512, -3981, -6310, -10000, -15850, -25120, -39810,
 -63100, -100000, -158500, -251200, -398100, -631000};
uint32_t num_trees_to_evaluate;
uint32_t max_eval_height = 30;

// OPEN GL VARIABLES
#ifndef FOREST_CREATION_ONLY
int main_window;
uint32_t window_width = ((3 * src_width) / 2);
uint32_t window_height = ((3 * src_height) / 2);
CurResult cur_result = ForrestLabeling;
int32_t cur_image = 0;
const int num_textures = 2;
GLuint textures[num_textures];
unsigned char *texture_data;
Clock clk;
uint8_t* cur_image_rgb = NULL;
IM_TYPE render_image_type = IM_TYPE::IM_DEPTH;
bool render_labels = true;
#endif

void shutdown();

#ifndef FOREST_CREATION_ONLY
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
  for (int32_t i = 0; i < training_data->im_width*training_data->im_height; i++) {
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
void PaintLabelDataRed(T* src) {
  for (int32_t i = 0; i < training_data->im_width*training_data->im_height; i++) {
    if (src[i] != 0) {
      texture_data[4*i] = 255;
      texture_data[4*i+1] = 0;
      texture_data[4*i+2] = 0;
      texture_data[4*i+3] = 255;
    } else {
      // texture_data[4*i+3] = 128;  // dim everything else slightly
    }
  }
}

uint64_t frame_counter = 0;
double time_accumulate = 0;
void display() {
  int16_t* cur_image_data;
  uint8_t* cur_label_data;

  memset(texture_data, 0, training_data->im_width * training_data->im_height * 
         sizeof(texture_data[0]) * 4);
  
  if (cur_image < test_data->num_images) {
    cur_image_data = &test_data->image_data[cur_image * test_data->im_width * 
                                            test_data->im_height];
    cur_label_data = &test_data->label_data[cur_image * test_data->im_width * 
                                            test_data->im_height];
  } else {
    int32_t training_data_index = cur_image - test_data->num_images;
    cur_image_data = &training_data->image_data[training_data_index * 
      training_data->im_width * training_data->im_height];
    cur_label_data = &training_data->label_data[training_data_index * 
      training_data->im_width * training_data->im_height];
  }

  if (render_image_type == IM_DEPTH) {
    // Render a texture here
    UpdateGreyscaleImageForRendering<int16_t>(cur_image_data);
  }

  switch (cur_result) {
  case CorrectLabeling:
    if (render_labels) {
      PaintLabelDataRed<uint8_t>(cur_label_data);
    }
    break;
  case NumResults:
    break;    
  case ForrestLabeling:

    double t0 = clk.getTime();
    evaluateDecisionForest(label_data_evaluated, forest, max_eval_height,
        num_trees_to_evaluate, cur_image_data, training_data->im_width, 
        training_data->im_height);
    if (filter_results) {
      MedianLabelFilter<uint8_t, int16_t>(label_data_filtered, 
        label_data_evaluated, cur_image_data,
        training_data->im_width, training_data->im_height, filter_results_radius);
      // swap the buffers
      uint8_t* temp;
      temp = label_data_filtered;
      label_data_filtered = label_data_evaluated;
      label_data_evaluated = temp;
    }
    double t1 = clk.getTime();

    time_accumulate += t1 - t0;
    frame_counter++;
    if (time_accumulate > 10) {
      cout << "decision forest evaluation speed = ";
      cout << static_cast<double>(frame_counter) / time_accumulate << endl;
      time_accumulate = 0;
      frame_counter = 0;
    }

    if (render_labels) {
      PaintLabelDataRed<uint8_t>(label_data_evaluated);
    }
    break;

  }
  
  // Copy texture data to OpenGL
  glBindTexture(GL_TEXTURE_2D, textures[0]);
  glTexImage2D(GL_TEXTURE_2D,  0, GL_RGBA, training_data->im_width, 
               training_data->im_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture_data);
  if (render_image_type != IM_DEPTH) {
    glBindTexture(GL_TEXTURE_2D, textures[1]);
    glTexImage2D(GL_TEXTURE_2D,  0, GL_RGB, src_width, src_height, 0, GL_RGB, 
                 GL_UNSIGNED_BYTE, cur_image_rgb);
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
  
  if (render_image_type != IM_DEPTH) {
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

  texture_data = new unsigned char[test_data->im_width * test_data->im_height * 4];

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
  case '=':
    if (cur_image < (total_num_images - 1)) {
      cur_image++;
    }
    if (cur_image < test_data->num_images) {
      cout << "cur_image = " << cur_image << " --> Test data: " << test_data->filenames[cur_image] << endl;
    } else {
      cout << "cur_image = " << cur_image << " --> Training data: " << training_data->filenames[cur_image - test_data->num_images] << endl;
    }
    image_changed = true;
    break;
  case '-':
    if (cur_image > 0) {
      cur_image--;
    }
    if (cur_image < test_data->num_images) {
      cout << "cur_image = " << cur_image << " --> Test data: " << test_data->filenames[cur_image] << endl;
    } else {
      cout << "cur_image = " << cur_image << " --> Training data: " << training_data->filenames[cur_image - test_data->num_images] << endl;
    }
    image_changed = true;
    break;
  case '+':
    if (cur_image < (total_num_images - 100)) {
      cur_image+=100;
    }
    if (cur_image < test_data->num_images) {
      cout << "cur_image = " << cur_image << " --> Test data: " << test_data->filenames[cur_image] << endl;
    } else {
      cout << "cur_image = " << cur_image << " --> Training data: " << training_data->filenames[cur_image - test_data->num_images] << endl;
    }
    image_changed = true;
    break;
  case '_':
    if (cur_image > 99) {
      cur_image-=100;
    }
    if (cur_image < test_data->num_images) {
      cout << "cur_image = " << cur_image << " --> Test data: " << test_data->filenames[cur_image] << endl;
    } else {
      cout << "cur_image = " << cur_image << " --> Training data: " << training_data->filenames[cur_image - test_data->num_images] << endl;
    }
    image_changed = true;
    break;
  case ' ':
    cur_result = static_cast<CurResult>((cur_result + 1) % NumResults);
      if (cur_result == CurResult::CorrectLabeling) {
        cout << "cur_result = correct labeling" << endl;
      } else {
        cout << "cur_result = forest labeling" << endl;
      }
    break;
  case '}':
  case ']':
    if (num_trees_to_evaluate < prog_settings.num_trees) { num_trees_to_evaluate++; }
    cout << "num_trees_to_evaluate = " << num_trees_to_evaluate << endl;
    break;
  case '{':
  case '[':
    if (num_trees_to_evaluate > 1) { num_trees_to_evaluate--; }
    cout << "num_trees_to_evaluate = " << num_trees_to_evaluate << endl;
    break;
  case 'D':
  case 'd':
#if defined(WIN32) || defined(_WIN32)
    cur_filename = IMAGE_DIRECTORY;
    if (cur_image < test_data->num_images) {
      cur_filename += string(test_data->filenames[cur_image]);
    } else {
      cur_filename += string(training_data->filenames[cur_image - test_data->num_images]);
    }
    if (delete_confirmed == 1) {
      if(!DeleteFile(cur_filename.c_str())) {
        cout << "Error deleting file: " << cur_filename.c_str() << endl;
      } else {
        cout << "File deleted sucessfully: " << cur_filename.c_str() << endl;
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
  case 'F':
  case 'f':
    filter_results = !filter_results;
    cout << "filter_results = " << filter_results << endl;
    break;
  case '>':
  case '.':
    filter_results_radius++;
    cout << "filter_results_radius = " << filter_results_radius << endl;
    break;
  case '<':
  case ',':
    if (filter_results_radius > 0) { filter_results_radius--; }
    cout << "filter_results_radius = " << filter_results_radius << endl;
    break;
  case 'R':
  case 'r':
    render_image_type = (IM_TYPE)((render_image_type + 1) % IM_NUM_TYPES);
    cout << "render_image_type = " << render_image_type << endl;
    image_changed = true;
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
  case 'S':
  case 's':
    if (DT_DOWNSAMPLE != 1) {
      cout << "ERROR: can only save processed images when DT_DOWNSAMPLE = 1" << endl;
    } else {
      saveProcessedImages(IMAGE_DIRECTORY, test_data, training_data);
      cout << "Saved processed images to file!" << endl;
    }
    break;
  case 'U':
  case 'u':
    if (cur_image < test_data->num_images) {
      reloadDepthImage(IMAGE_DIRECTORY, &test_data, cur_image, load_processed_images);
    } else {
      reloadDepthImage(IMAGE_DIRECTORY, &training_data, cur_image - test_data->num_images, load_processed_images);
    }  
    cout << "Re-loaded image from file." << endl;
    image_changed = true;
    break;
  }      
  
  switch (key) {
    case '!':
      red_hue_threshold = (red_hue_threshold + 1) % 255;
      break;
    case '1':
      red_hue_threshold = (red_hue_threshold == 0) ? 255 : red_hue_threshold - 1;
      break;      
    case '@':
      red_sat_threshold = (red_sat_threshold + 1) % 255;
      break;
    case '2':
      red_sat_threshold = (red_sat_threshold == 0) ? 255 : red_sat_threshold - 1;
      break;         
    case '#':
      red_val_threshold = (red_val_threshold + 1) % 255;
      break;
    case '3':
      red_val_threshold = (red_val_threshold == 0) ? 255 : red_val_threshold - 1;
      break;   
    case '$':
      red_hue_target = (red_hue_target + 1) % 255;
      break;
    case '4':
      red_hue_target = (red_hue_target == 0) ? 255 : red_hue_target - 1;
      break;
    case '%':
      red_sat_target = (red_sat_target + 1) % 255;
      break;
    case '5':
      red_sat_target = (red_sat_target == 0) ? 255 : red_sat_target - 1;
      break;
    case '^':
      red_val_target = (red_val_target + 1) % 255;
      break;
    case '6':
      red_val_target = (red_val_target == 0) ? 255 : red_val_target - 1;
      break;
  }
  
  if (key == '!' || key == '1' || key == '@' || key == '2' ||
      key == '#' || key == '3' || key == '$' || key == '4' ||
      key == '%' || key == '5' || key == '^' || key == '6') {
    image_changed = true;
    cout << "RED HSV Thesholds = <" << red_hue_threshold << ", ";
    cout << red_sat_threshold << ", " << red_val_threshold << ">" << endl;
    cout << "RED HSV Targets = <" << red_hue_target << ", ";
    cout << red_sat_target << ", " << red_val_target << ">" << endl;
    if (cur_image < test_data->num_images) {
      reloadDepthImage(IMAGE_DIRECTORY, &test_data, cur_image, load_processed_images);
    } else {
      reloadDepthImage(IMAGE_DIRECTORY, &training_data, cur_image - test_data->num_images, load_processed_images);
    }     
  }  
  
  if (image_changed) {
    if (!load_processed_images) {
      if (cur_image < test_data->num_images) {
        LoadRGBImage(IMAGE_DIRECTORY + 
                     string(test_data->filenames[cur_image]), cur_image_rgb, 
                     render_image_type);
      } else {
        LoadRGBImage(IMAGE_DIRECTORY + 
                     string(training_data->filenames[cur_image - test_data->num_images]), 
                     cur_image_rgb, render_image_type);    
      }
    }
  }
}

void mouse(int button, int state, int x, int y) {
  if (render_image_type == IM_DEPTH && state == GLUT_DOWN) {
    float pos_window_x = static_cast<float>(x) / static_cast<float>(window_width);
    float pos_window_y = static_cast<float>(y) / static_cast<float>(window_height);
    int32_t u = static_cast<float>(floorf(pos_window_x * static_cast<float>(src_width / DT_DOWNSAMPLE)));
    int32_t v = static_cast<float>(floorf(pos_window_y * static_cast<float>(src_height / DT_DOWNSAMPLE))); 
    if (u < (src_width / DT_DOWNSAMPLE) && v < (src_height / DT_DOWNSAMPLE) &&
        u >= 0 && v >= 0) {
      uint32_t index = v * (src_width / DT_DOWNSAMPLE) + u;
      uint8_t* label_image;
      int16_t* depth_image;
      if (cur_image < test_data->num_images) {
        label_image = &test_data->label_data[(cur_image * test_data->im_width * 
                                              test_data->im_height)];
        depth_image = &test_data->image_data[(cur_image * test_data->im_width * 
                                              test_data->im_height)];
      } else {
        label_image = &training_data->label_data[((cur_image - test_data->num_images) * 
                                                  training_data->im_width * 
                                                  training_data->im_height)];   
        depth_image = &training_data->image_data[((cur_image - test_data->num_images) * 
                                                  training_data->im_width * 
                                                  training_data->im_height)];
      }
      
      if (button == GLUT_LEFT_BUTTON) {
        if (depth_image[index] != 0 && depth_image[index] < GDT_MAX_DIST) {
          cout << "flooding label pixel <" << u << ", ";
          cout << v << ">" << endl;
          floodPixel(label_image, depth_image, u, v, 200, 16);  // rad fill squared
        } else {
          cout << "Can't flood label pixel <" << u << ", ";
          cout << v << ">" << endl;
        }
      } else if (button == GLUT_RIGHT_BUTTON) {
        if (depth_image[index] != 0 && depth_image[index] < GDT_MAX_DIST) {
          cout << "flooding label pixel <" << u << ", ";
          cout << v << ">" << endl;
          floodPixel(label_image, depth_image, u, v, 100, 5);
        } else {
          cout << "Can't flood label pixel <" << u << ", ";
          cout << v << ">" << endl;
        }
      }
    }
  } else if (state == GLUT_DOWN) {
    float pos_window_x = static_cast<float>(x) / static_cast<float>(window_width);
    float pos_window_y = static_cast<float>(y) / static_cast<float>(window_height);
    int32_t u = static_cast<float>(floorf(pos_window_x * static_cast<float>(src_width)));
    int32_t v = static_cast<float>(floorf(pos_window_y * static_cast<float>(src_height))); 
    uint32_t index = v * src_width + u;
    cout << "Image Val = <" << cur_image_rgb[3*index] << ", ";
    cout << cur_image_rgb[3*index+1] << ", " << cur_image_rgb[3*index+2] << ">" << endl;
  }
}

void reshape(const int width, const int height) {
  window_width = width;
  window_height = height;
  glutPostRedisplay();
}
#endif

void shutdown() {
  release_image_io();
    delete[] label_data_evaluated;
    delete[] label_data_filtered;
#ifndef FOREST_CREATION_ONLY
    delete[] texture_data;
    delete[] cur_image_rgb;
#endif
    if (wl_set.wl_coeffs0 != NULL) {
      delete[] wl_set.wl_coeffs0;
      delete[] wl_set.wl_coeffs1;
      delete[] wl_set.wl_coeffs2;
      delete[] wl_set.wl_coeffs_sizes;
    }
    releaseImages(test_data);
    releaseImages(training_data);
    releaseForest(&forest, prog_settings.num_trees);
    exit(0);
}

int main(int argc, char *argv[]) { 
    static_cast<void>(argc);  // Get rid of unused variable warning
    static_cast<void>(argv);
#ifdef _DEBUG
  debug::EnableMemoryLeakChecks();
  // debug::SetBreakPointOnAlocation(20624);
#endif
  try {
    if (DT_DOWNSAMPLE < 1) {
      cout << "ERROR: DT_DOWNSAMPLE < 1!" << endl;
      return -1;
    }
    if (argc <= 1) {
      loadSettingsFromFile(&prog_settings, PROGRAM_SETTINGS_FILENAME);
    } else if (argc == 2) {
      loadSettingsFromFile(&prog_settings, string(argv[1]));
    } else {
      std::cout << "Usage: KinectHands <filename>    <-- filename is optional" << endl;
    }
    init_image_io();
    cout << "loading image data from file..." << endl;
    LoadDepthImagesFromDirectoryForDT(IMAGE_DIRECTORY, training_data, 
                                      test_data, frac_test_data, file_skip, 
                                      load_processed_images);
    total_num_images = test_data->num_images + training_data->num_images; 
    
    if (!prog_settings.load_forest_from_file) {
      srand(0);
      
      // Allocate space for our decision trees
      uint32_t total_num_trees = (prog_settings.num_bootstrap_passes + 1) * prog_settings.num_trees;
      forest = new DecisionTree[total_num_trees];
      
      // Allocate space for the WLInput data struct
      wl_set.wl_coeffs0 = new int32_t[num_uv_offset_vals];
      wl_set.wl_coeffs1 = new int32_t[num_uv_offset_vals];
      switch (prog_settings.wl_func_type) {
      case 0:
        wl_set.wl_funcs = new uint8_t[NUM_WL_FUNCS];
        break;
      case 1:
        wl_set.wl_funcs = new uint8_t[1];
        break;
      case 2:
        wl_set.wl_funcs = new uint8_t[2];
        break;
      }

      // Now fill the coefficient arrays
      cout << endl << "wl_coeffs0 and 1 coeffs = [";
      for (int32_t i = 0; i < num_uv_offset_vals; i++) {
        wl_set.wl_coeffs0[i] = uv_offset_vals[i];
        wl_set.wl_coeffs1[i] = uv_offset_vals[i];
        cout << wl_set.wl_coeffs1[i];
        if (i != num_uv_offset_vals - 1) {
          cout << ", ";
        }
      }
      cout << "]" << endl << endl;
      wl_set.wl_coeffs2 = new int16_t[num_threshold_vals];
      cout << endl << "wl_coeffs2 = [";
      for (uint32_t i = 0; i < num_threshold_vals; i++) {
        wl_set.wl_coeffs2[i] = threshold_vals[i];
        cout << wl_set.wl_coeffs2[i];
        if (i != num_threshold_vals - 1) {
          cout << ", ";
        }
      }
      cout << "]" << endl << endl << "wl_funcs = [";
      switch (prog_settings.wl_func_type) {
      case 0:
        for (uint8_t i = 0; i < NUM_WL_FUNCS; i++) {
          wl_set.wl_funcs[i] = i;
          cout << static_cast<int>(wl_set.wl_funcs[i]);
          if (i != NUM_WL_FUNCS - 1) {
            cout << ", ";
          }
        }
        break;
      case 1:
        wl_set.wl_funcs[0] = 0;
        cout << static_cast<int>(wl_set.wl_funcs[0]);
        break;
      case 2:
        wl_set.wl_funcs[0] = 1;
        cout << static_cast<int>(wl_set.wl_funcs[0]);
        break;
      }
      cout << "]" << endl << endl;
      
      wl_set.wl_coeffs_sizes = new uint32_t[4];
      wl_set.wl_coeffs_sizes[0] = num_uv_offset_vals;
      wl_set.wl_coeffs_sizes[1] = num_uv_offset_vals;
      wl_set.wl_coeffs_sizes[2] = num_threshold_vals;
      switch (prog_settings.wl_func_type) {
      case 0:
        wl_set.wl_coeffs_sizes[3] = NUM_WL_FUNCS;
        break;
      case 1:
      case 2:
        wl_set.wl_coeffs_sizes[3] = 1;
        break;
      }
      wl_set.num_samples_per_node = prog_settings.num_wl_samples_per_node;
      
      settings = new TrainingSettings[total_num_trees];
      for (uint32_t i = 0; i < total_num_trees; i++) {
        if (prog_settings.max_num_images > static_cast<uint32_t>(training_data->num_images)) {
          settings[i].num_images_to_consider = training_data->num_images;
        } else {
          settings[i].num_images_to_consider = prog_settings.max_num_images;
        }
        settings[i].min_info_gain = prog_settings.min_info_gain;
        settings[i].tree_height = prog_settings.tree_height;
        settings[i].max_pixels_per_image_per_label = prog_settings.max_pixels_per_image_per_label;
        settings[i].seed = static_cast<unsigned int>(rand());
        settings[i].dt_index = i;
      }
      for (uint32_t i = prog_settings.num_trees; i < total_num_trees; i++) {
        settings[i].tree_height = prog_settings.bootstrap_tree_height;
      }
     
      std::thread* threads = new std::thread[prog_settings.num_workers];
      
      GenerateDecisionTree genTree;
      for (uint32_t i = 0; i < prog_settings.num_trees; i += prog_settings.num_workers) {
        for (uint32_t j = 0; j < prog_settings.num_workers && (i + j) < prog_settings.num_trees; j++) {
          uint32_t cur_tree_index = i + j;
          // Hack to fix linux version when statically linking --> it throws operation not permitted
          threading::Callback<void>* threadBody = threading::MakeCallableOnce(
                &GenerateDecisionTree::generateDecisionTree, &genTree, &forest[cur_tree_index],
                training_data, &wl_set, &settings[cur_tree_index], static_cast<DecisionTree*>(NULL),
                static_cast<uint32_t>(0));
          threads[j] = threading::MakeThread(threadBody);
        }
        for (uint32_t j = 0; j < prog_settings.num_workers && (i + j) < prog_settings.num_trees; j++) {
          threads[j].join();
        }
      }

      for (uint32_t pass = 0; pass < prog_settings.num_bootstrap_passes; pass++) {
        cout << endl << "Performing bootstrap pass " << pass+1;
        cout << " of " << prog_settings.num_bootstrap_passes << endl;
        uint32_t num_trees_so_far = (pass + 1) * prog_settings.num_trees;
        for (uint32_t i = 0; i < prog_settings.num_trees; i += prog_settings.num_workers) {
          for (uint32_t j = 0; j < prog_settings.num_workers && (i + j) < prog_settings.num_trees; j++) {
            uint32_t cur_tree_index = ((pass + 1) * prog_settings.num_trees) + i + j;
            threading::Callback<void>* threadBody = threading::MakeCallableOnce(
              &GenerateDecisionTree::generateDecisionTree, &genTree, &forest[cur_tree_index], 
              training_data, &wl_set, &settings[cur_tree_index], forest, 
              static_cast<uint32_t>(num_trees_so_far));
            threads[j] = threading::MakeThread(threadBody);
          }
          for (uint32_t j = 0; j < prog_settings.num_workers && (i + j) < prog_settings.num_trees; j++) {
            threads[j].join();
          }
        }
      }
      
      cout << endl << "Saving Forest to file..." << endl << endl;
      prog_settings.num_trees = total_num_trees;  // Just pretend we have this many trees
      saveForest(forest, total_num_trees, FOREST_DATA_FILENAME);
    } else {
      wl_set.wl_coeffs0 = NULL;
      wl_set.wl_coeffs1 = NULL;
      wl_set.wl_coeffs2 = NULL;
      wl_set.wl_funcs = NULL;
      wl_set.wl_coeffs_sizes = NULL;
      cout << "Loading Forest from file..." << endl;
      loadForest(&forest, &prog_settings.num_trees, FOREST_DATA_FILENAME);
    }
    
    const int32_t evaluate_median_radius = 2;
    if (test_data->num_images > 0) {
      cout << "Evaluating results..." << endl;
      for (uint32_t i = 0; i < prog_settings.num_trees; i++) {
        float error = evaluateDecisionForestError(test_data, forest, i+1, false, 
          evaluate_median_radius);
        cout << i+1 << " trees --> error on test set = " << error*100 << endl;
        error = evaluateDecisionForestError(test_data, forest, i+1, true, 
          evaluate_median_radius);
        cout << i+1 << " trees --> error on test set using " << evaluate_median_radius;
        cout << " rad median filter = " << error*100 << endl;
      }
    }
    
#ifdef FOREST_CREATION_ONLY

#else
    label_data_evaluated = new uint8_t[training_data->im_width * training_data->im_height];
    label_data_filtered = new uint8_t[training_data->im_width * training_data->im_height];
    num_trees_to_evaluate = prog_settings.num_trees;
    
    // Allocate space for the RGB image
    uint8_t dummy8; static_cast<void>(dummy8);
    cur_image_rgb = (uint8_t*)malloc(src_width * src_height * sizeof(dummy8) * 3);
    memset(cur_image_rgb, 0, src_width * src_height * sizeof(dummy8) * 3);
    if (!load_processed_images) {
      if (cur_image < test_data->num_images) {
        LoadRGBImage(IMAGE_DIRECTORY + string(test_data->filenames[cur_image]), 
                     cur_image_rgb, render_image_type);
      } else {
        LoadRGBImage(IMAGE_DIRECTORY + string(training_data->filenames[cur_image]), 
                     cur_image_rgb, render_image_type);    
      }
    }

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

    glutMainLoop();
#endif
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
