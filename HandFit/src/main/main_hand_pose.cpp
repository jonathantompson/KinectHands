//
//  main.cpp
//
//  A quick test of some mesh triangulation and deformation techniques
//

#include <stdlib.h>
#include <cmath>
#include <thread>
#include <iostream>
#include <fstream>
#include <limits>

#include "renderer/open_gl_common.h"
#include "renderer/renderer.h"
#include "renderer/geometry/geometry_manager.h"
#include "renderer/geometry/geometry.h"
#include "renderer/geometry/geometry_colored_mesh.h"
#include "renderer/colors.h"
#include "renderer/camera/camera.h"
#include "renderer/lights/light_dir.h"
#include "renderer/texture/texture_renderable.h"
#include "windowing/window.h"
#include "windowing/window_settings.h"
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector.h"
#include "jtil/string_util/string_util.h"
#include "kinect_interface/hand_net/hand_model.h"
#include "hand_fit/hand_fit.h"
#include "hand_fit/hand_renderer.h"
#include "hand_fit/hand_geometry.h"
#include "hand_fit/bounding_sphere.h"
#include "hand_fit/hand_geometry_mesh.h"
#include "jtil/math/noise.h"
#include "jtil/clk/clk.h"
#include "renderer/gl_state.h"
#include "jtil/renderer/geometry/mesh_simplification/mesh_simplification.h"

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif

// Processed files are MUCH quicker to load (100s x faster), but require that
// you load the unprocessed files once and save them to jbin format.
#define LOAD_PROCESSED_FILES

#if defined(__APPLE__)
  #define MODELS_PATH string("./../../../../../../../../../models/")
#else
  #define MODELS_PATH string("./models/")
#endif

using namespace std;
using namespace jtil::math;
using namespace jtil::data_str;
using namespace hand_fit;
using namespace kinect_interface::hand_net;
using namespace kinect_interface::hand_detector;
using renderer::Renderer;
using renderer::GeometryColoredMesh;
using renderer::Geometry;
using renderer::GeometryManager;
using renderer::TextureRenderable;
using renderer::BoundingSphere;
using renderer::GLState;
using renderer::GeometryType;
using windowing::Window;
using windowing::WindowSettings;
using jtil::renderer::mesh_simplification::MeshSimplification;
using jtil::renderer::mesh_simplification::MeshSettings;

jtil::clk::Clk* clk = NULL;
double t1, t0;

// The main window and basic rendering system
Window* wnd = NULL;
WindowSettings* wnd_settings;
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
HandModel* rhand = NULL;
HandModel* lhand = NULL;
HandModel* rhand_rest_pose = NULL;
HandRenderer* hand_renderer = NULL;
uint32_t cur_coeff = 0;
bool animate_hand = false;
double t_animation = 0;
static const uint32_t num_keyframes = 100;
Noise<float>* hand_frames[HAND_NUM_COEFF];
bool save_frames_to_file = false;
float depth_tmp[src_dim * 4];
int16_t depth[src_dim];
uint8_t labels[src_dim];
Eigen::MatrixXf coeffs;

// Bunny model for decimation test
Geometry* bunny;
Geometry* bunny_copy;
int num_edges_to_remove = 0;
MeshSimplification* mesh_simplifier = NULL;
GeometryColoredMesh* sphere;

void saveCDepthDataToDisk(float* data, std::string filename) {
  // First collect all the depth values:
  for (uint32_t i = 0; i < src_dim; i++) {
    depth[i] = static_cast<int16_t>(data[4*i]);
  }
  // Now collect all the label values from RGB:
  for (uint32_t i = 0; i < src_dim; i++) {
    float r = data[4*i + 1];
    float g = data[4*i + 2];
    float b = data[4*i + 3];
    labels[i] = labelFromRGB(r, g, b);
  }
  std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error(string("error opening file:") + filename);
  }
  file.write((const char*)(depth), sizeof(depth[0]) * src_dim);
  file.flush();
  file.write((const char*)(labels), sizeof(labels[0]) * src_dim);
  file.flush();
  float cur_coeff;
  for (uint32_t i = 0; i < HAND_NUM_COEFF; i++) {
    cur_coeff = rhand->getCoeff(i);
    file.write((const char*)(&cur_coeff), sizeof(cur_coeff));
  }
  file.flush();
  file.close();
  cout << "saved frame: " << filename << endl;
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

Float4x4 trans_mat;
Float4x4 new_bone_mat;
float xoffset = 0;
float yoffset = 0;
float zoffset = 0;
uint64_t frame_count = 0;

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

    // CHANGE THE COEFFICIENT
    if (cur_coeff <= 2) {
      theta_y *= 50.0f;
    }
    if (cur_coeff == HandCoeff::WRIST_TWIST) {
      float scale = 1.0f - theta_y*0.1f;
      sphere->mat()->rightMultScale(scale, scale, scale);
    } else {
      float coeff_val;
      coeff_val = lhand->getCoeff(cur_coeff);
      lhand->setCoeff(cur_coeff, coeff_val - theta_y);
      cout << "cur_coeff " << HandCoeffToString(cur_coeff);
      cout << " --> " << coeff_val - theta_y << endl;
    }

    //// CHANGE THE POSITION OF THE BOUNDING SPHERE
    //HandModelGeometryMesh* geom = (HandModelGeometryMesh*)hand_renderer->l_hand_geom();
    //theta_y *= 0.1f;
    //uint32_t sphere_index = HandSphereIndices::F2_KNU3_B;
    //uint32_t coord_index = cur_coeff % 3;
    //switch (coord_index) {
    //case 0:
    //  xoffset += theta_y;
    //  break;
    //case 1:
    //  yoffset += theta_y;
    //  break;
    //case 2:
    //  zoffset += theta_y;
    //  break;
    //}
    //Float4x4* start_mat = geom->bsph(sphere_index)->starting_bone_mat();
    //Float4x4* mat = geom->bsph(sphere_index)->mat();
    //trans_mat.translationMat(xoffset, yoffset, zoffset);
    //math::Float4x4::mult(&new_bone_mat, start_mat, &trans_mat);
    //mat->set(&new_bone_mat);
    //cout << "offset = " << xoffset << ", " << yoffset << ", " << zoffset << endl << endl;
  }
}

void KeyboardCB(int key, int action) {
  bool decimate_mesh = false;
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
        for (uint32_t i = 0; i < HAND_NUM_COEFF; i++) {
          delete hand_frames[i]; hand_frames[i] = NULL;
        }
        delete clk;
        delete render;
        delete rhand;
        delete lhand;
        delete rhand_rest_pose;
        if (hand_renderer) {
          delete hand_renderer;
        }
        GLState::shutdownGLState();
        delete wnd_settings;
        delete wnd;
        delete bunny_copy;
        delete mesh_simplifier;
        Window::killWindowSystem();
        exit(0);
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
    case static_cast<int>('}'):
    case static_cast<int>(']'):
      if (action == RELEASED) {
        cur_coeff = (cur_coeff + 1) % HandCoeff::NUM_PARAMETERS;
        cout << "cur_coeff = " << HandCoeffToString(cur_coeff); 
        cout << " = " << rhand->getCoeff(cur_coeff);
        cout << std::endl;
      }
      break;
    case static_cast<int>('{'):
    case static_cast<int>('['):
      if (action == RELEASED) {
        cur_coeff = cur_coeff != 0 ? (cur_coeff - 1) : HandCoeff::NUM_PARAMETERS - 1;
        cout << "cur_coeff = " << HandCoeffToString(cur_coeff); 
        cout << " = " << rhand->getCoeff(cur_coeff);
        cout << std::endl;
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
        std::cout << "wireframe = " << (int)render->wireframe << std::endl;
      }
      break;
    case static_cast<int>('y'):
    case static_cast<int>('Y'):
      if (action == RELEASED) {
        render->render_bounding_spheres = !render->render_bounding_spheres;
        std::cout << "bounding spheres = ";
        std::cout << (int)render->render_bounding_spheres << std::endl;
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
    case static_cast<int>('3'):
      if (action == RELEASED) {
        render_output = 3;
      }
      break;
    case KEY_KP_ADD:
      if (action == RELEASED) {
        int nfaces = ((GeometryColoredMesh*)bunny)->indices()->size()/3;
        num_edges_to_remove += nfaces/4;
        decimate_mesh = true;
      }
      break;
    case KEY_KP_SUBTRACT:
      if (action == RELEASED) {
        int nfaces = ((GeometryColoredMesh*)bunny)->indices()->size()/3;
        num_edges_to_remove -= nfaces/2;
        if (num_edges_to_remove < 0) {
          num_edges_to_remove = 0;
        }
        decimate_mesh = true;
      }
      break;
    case static_cast<int>('K'):
    case static_cast<int>('k'):
      if (action == RELEASED) {
        animate_hand = !animate_hand;
        cout << "animate_hand = " << animate_hand << endl;
      }
      break;
    case static_cast<int>('L'):
    case static_cast<int>('l'):
      if (action == RELEASED) {
        save_frames_to_file = !save_frames_to_file;
        cout << "save_frames_to_file = " << save_frames_to_file << endl;
      }
      break;
  }

  if (decimate_mesh) {
    cout << "num_edges_to_remove = " << num_edges_to_remove << endl;
    Geometry* root = GeometryManager::g_geom_manager()->scene_graph_root();
    int bunny_index = -1;
    for (uint32_t i = 0 ; i < root->numChildren(); i++) {
      if (root->getChild(i) == bunny) {
        bunny_index = static_cast<int>(i);
      }
    }
    if (bunny_index == -1) {
      throw runtime_error("ERROR: Couldn't find the bunny in the scene's geom");
    }
    delete bunny;
    bunny = bunny_copy->copy();

    if (bunny->type() != GeometryType::GEOMETRY_COLORED_MESH) {
      throw runtime_error(string("Bunny is not a colored mesh!  Mesh ") + 
        string("decimation will fail!"));
    } else {
      GeometryColoredMesh* mesh = reinterpret_cast<GeometryColoredMesh*>(bunny);
      mesh->unsyncVAO();
      mesh_simplifier->simplifyMesh(*mesh->normals(), *mesh->indices(), 
        num_edges_to_remove, *mesh->vertices(), *mesh->colors());
      mesh->syncVAO();
      root->setChild(bunny, bunny_index);
    }
  }
}

using std::cout;
using std::endl;
Float4x4 mat_tmp;
Float4x4 mat_result;

int main(int argc, char *argv[]) {
#if defined(_DEBUG) && defined(_WIN32)
  _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
  // _CrtSetBreakAlloc(272);
#endif

  cout << "USAGE:" << endl;
  cout << "WSADQE - Move camera" << endl;
  cout << "shift - Sprint" << endl;
  cout << "mouse left click + drag - Rotate camera" << endl;
  cout << "mouse right click + drag - Adjust coefficient" << endl;
  cout << "[] - Change adjustment coeff" << endl;
  cout << "r - rotate light" << endl;
  cout << "t - wireframe rendering" << endl;
  cout << "y - render bounding spheres" << endl;
  cout << "123 - Render output type" << endl;
  cout << "+/- - Increase / Decrease bunny decimation" << endl;
  cout << "k - Animate Hand" << endl;
  cout << "l - Save frames to file" << endl << endl;
  
  try {
    clk = new jtil::clk::Clk();
    t1 = clk->getTime();

    for (uint32_t i = 0; i < HAND_NUM_COEFF; i++) {
      hand_frames[i] = new Noise<float>(num_keyframes);
    }
    
    // Initialize Windowing system
    Window::initWindowSystem();

    // Fill the settings structure
    wnd_settings = new WindowSettings();
    //wnd_settings->width = 1280;  // 780p
    //wnd_settings->height = 720;  // 780p
    wnd_settings->width = 1280;
    wnd_settings->height = 1024;
    wnd_settings->fullscreen = false;
    wnd_settings->title = "Hand Fit Project";
    wnd_settings->gl_major_version = 3;
    wnd_settings->gl_minor_version = 2;
    wnd_settings->num_depth_bits = 24;
    wnd_settings->num_stencil_bits = 0;
    wnd_settings->num_rgba_bits = 8;
    
    // Create the window
    wnd = new Window(*wnd_settings);

    GLState::initGLState();

    // Create a mesh simplifier with default settings
    mesh_simplifier = new MeshSimplification();
    
    // Create an instance of the renderer
    FloatQuat eye_rot; 
    eye_rot.identity();
    Float3 eye_pos(0, 0, 0);
    render = new Renderer();
    render->init(eye_rot, eye_pos, wnd_settings->width, wnd_settings->height,
      -1.0f, -2000.0f, HAND_CAMERA_FOV);
    
    // Spawn some pretty objects (to test the renderer)
    GeometryColoredMesh* tmp;
    tmp = GeometryColoredMesh::makeTorusKnot(renderer::red, 5, 64, 512);
    tmp->mat()->scaleMat(100, 100, 100);
    tmp->mat()->leftMultTranslation(0.0f, 100.0f, 1000.0f);
    GeometryManager::scene_graph_root()->addChild(tmp);
    tmp = GeometryColoredMesh::makeSphere(64, 64, 1.0f, renderer::blue);
    tmp->mat()->scaleMat(100, 100, 100);
    tmp->mat()->leftMultTranslation(0.0f, 100.0f, 1000.0f);
    GeometryManager::scene_graph_root()->addChild(tmp);

    // Try loading some meshes from file
    // Some nice ones here: http://graphics.cs.williams.edu/data/meshes.xml
    // and here: http://www.models-resource.com/
    // and definitely here: http://www.blendswap.com/blends/category/characters/
    bunny = GeometryManager::g_geom_manager()->loadFromFile(
      MODELS_PATH, "bunny.obj");
    bunny->mtrl()->specular_intensity = 1.0f;
    bunny->mtrl()->specular_power = 32.0f;
    bunny->mat()->scaleMat(1000, 1000, 1000);
    bunny->mat()->leftMultTranslation(-300.0f, 0, 750.0f);
    bunny_copy = bunny->copy();
    GeometryManager::scene_graph_root()->addChild(bunny);

    Geometry* dog = GeometryManager::g_geom_manager()->loadFromFile(
      MODELS_PATH + "SmallDog/", "small_dog.dae");
    dog->mat()->scaleMat(200, 200, 200);
    dog->mat()->leftMultTranslation(300.0f, 0, 750.0f);
    GeometryManager::scene_graph_root()->addChild(dog);

    
    sphere = GeometryColoredMesh::makeSphere(64, 64, 1.0f, renderer::white);
    sphere->mat()->scaleMat(50, 50, 50);
    sphere->mat()->leftMultTranslation(0.0f, 100.0f, 1000.0f);
    GeometryManager::scene_graph_root()->addChild(sphere);
    
    // Attach callback functions for event handling
    wnd->registerKeyboardCB(&KeyboardCB);
    wnd->registerMousePosCB(&MousePosCB);
    wnd->registerMouseButtonCB(&MouseButtonCB);
    wnd->registerMouseWheelCB(NULL);
    wnd->registerCharacterInputCB(NULL);

    hand_renderer = new HandRenderer(render, true, true);
    rhand = new HandModel(HandType::RIGHT);
    lhand = new HandModel(HandType::LEFT);
    lhand->setCoeff(HandCoeff::HAND_POS_X, -150);
    rhand_rest_pose = new HandModel(HandType::RIGHT);
    coeffs.resize(1, HAND_NUM_COEFF * 2);

    // Main render loop
    while (true) {
      t0 = t1;
      t1 = clk->getTime();
      float dt = static_cast<float>(t1-t0);

      if (rotate_light) {
        renderer::LightDir* light = render->light_dir();
        Float3* dir = light->direction_world();
        Float4x4::rotateMatYAxis(mat_tmp, dt);
        Float3 new_dir;
        Float3::affineTransformVec(new_dir, mat_tmp, *dir);
        dir->set(new_dir);
      }

      static const float pos_amp = 200;
      if (animate_hand) {
        t_animation += (t1 - t0);
        for (uint32_t i = HandCoeff::HAND_POS_X; i <= HandCoeff::HAND_POS_Z; i++) {
          float noise = hand_frames[i]->sample((float)t_animation);  // [-1, 1]
          lhand->setCoeff(i, rhand_rest_pose->getCoeff(i) + pos_amp * noise);
        }
        Float3 euler_ang;
        euler_ang[0] =  (float)(2.0 * M_PI) * 0.5f * 
          (1.0f + hand_frames[HandCoeff::HAND_ORIENT_X]->sample((float)t_animation*0.25f));  
        euler_ang[1] =  (float)(2.0 * M_PI) * 0.5f * 
          (1.0f + hand_frames[HandCoeff::HAND_ORIENT_Y]->sample((float)t_animation*0.25f)); 
        euler_ang[2] =  (float)(2.0 * M_PI) * 0.5f * 
          (1.0f + hand_frames[HandCoeff::HAND_ORIENT_Z]->sample((float)t_animation*0.25f)); 
        lhand->setRotation(euler_ang);
        for (uint32_t i = 0; i < 4; i++) {
          // Knuckle curl
          // noise = [0, 1]
          float noise = (0.5f*
            (hand_frames[HandCoeff::F0_KNUCKLE_CURL + i*3]->sample((float)t_animation) + 1.0f));
          float min = HandFit::coeff_min_limit[HandCoeff::F0_KNUCKLE_CURL + i*3];
          float max = HandFit::coeff_max_limit[HandCoeff::F0_KNUCKLE_CURL + i*3];
          lhand->setCoeff(HandCoeff::F0_KNUCKLE_CURL + i*3, min + noise * (max-min));
          // finger bend
          noise = (0.5f * 
            (hand_frames[HandCoeff::F0_PHI + i*3]->sample((float)t_animation) + 1.0f));
          min = HandFit::coeff_min_limit[HandCoeff::F0_PHI + i*3];
          max = HandFit::coeff_max_limit[HandCoeff::F0_PHI + i*3];
        }
      }

      // Move the camera
      Float3 zeros;
      zeros.zeros();
      delta_pos.set(cur_dir);
      if (!Float3::equal(zeros, delta_pos)) {
        delta_pos.normalize();
        Float3::scale(delta_pos, camera_speed * dt);
        if (running) {
          Float3::scale(delta_pos, camera_run_mulitiplier);
        }
        render->camera()->moveCamera(&delta_pos);
      }

      hand_renderer->updateMatrices(rhand->coeff(), rhand->hand_type());
      hand_renderer->updateMatrices(lhand->coeff(), lhand->hand_type());
      memcpy(coeffs.block<1, HAND_NUM_COEFF>(0, 0).data(), lhand->coeff(), 
        sizeof(lhand->coeff()[0]) * HAND_NUM_COEFF);
      memcpy(coeffs.block<1, HAND_NUM_COEFF>(0, HAND_NUM_COEFF).data(), 
        rhand->coeff(), sizeof(rhand->coeff()[0]) * HAND_NUM_COEFF);
      
      HandModel* hands[2];
      hands[0] = lhand;
      hands[1] = rhand;
      float interpenetration;
      switch (render_output) {
      case 1:
        render->renderFrame(dt);
        interpenetration = hand_renderer->calcInterpenetrationTerm();
        if (interpenetration > 1.0f + EPSILON) {
          cout << "interpenetration = " << interpenetration << endl;
        }
        break;
      case 2:
        hand_renderer->drawDepthMap(coeffs, hands, 2, false);
        hand_renderer->visualizeDepthMap(wnd, false);
        break;
      case 3:
        hand_renderer->drawDepthMap(coeffs, hands, 2, true);
        hand_renderer->visualizeDepthMap(wnd, true);
        break;
      default:
        throw runtime_error("ERROR: render_output is an incorrect value");
      }

      if (save_frames_to_file) {
        if (render_output != 3) {
          // If we haven't drawn the colored depth map, then draw it
          hand_renderer->drawDepthMap(coeffs, hands, 2, true);
        }
        TextureRenderable* tex = hand_renderer->cdepth_texture();
        tex->getTexture0Data<float>(depth_tmp);
        std::stringstream ss;
        ss << "data/hands_" << (uint64_t)(clk->getTime()*1e9) << ".bin";
        saveCDepthDataToDisk(depth_tmp, ss.str());
      }

      wnd->swapBackBuffer();

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
