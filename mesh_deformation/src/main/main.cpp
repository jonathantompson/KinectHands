//
//  main.cpp
//
//  A quick test of some mesh triangulation and deformation techniques
//

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#ifdef _WIN32
#include <windows.h>
#include <Commctrl.h>  // for InitCommonControls among others
#endif
#include <GLUT/glut.h>
#include <math.h>
#endif

#include <stdlib.h>
#include <cmath>
#include <thread>
#include <iostream>
#include <limits>

#include "math/math_types.h"
#include "file_io/file_io.h"
#include "camera/camera.h"
#include "data_str/vector.h"
#include "clock/clock.h"
#include "mesh_simplification/mesh_simplification.h"
#include "mesh_simplification/edge.h"
#include "mesh_simplification/test_plane.h"
#include "string_util/string_util.h"
#include "main/kinect_funcs.h"

using namespace std;
using renderer::Camera;
using math::Float3;
using math::Int3;
using math::Float4;
using math::FloatQuat;
using math::Float4x4;
using data_str::Vector;
using mesh_simplification::MeshSimplification;
using mesh_simplification::Edge;
using mesh_simplification::TestPlane;

Clock* clk = NULL;

// Glut window ids
int main_window;

// The current window size
int window_width = 1280, window_height = 1024;

// Camera class for handling view and proj matrices
Camera* camera;
const float camera_fov = 60.0f;
const float camera_znear = 1.0f;
const float camera_zfar = 1000.0f;
const float mouse_speed_rotation = 0.005f;
const float camera_speed = 30.0f;

// The texture for the hand
const int num_textures = 1;
GLuint textures[num_textures];

// Glut registered callbacks
void display();
void keyboard(unsigned char key, int x, int y);
void reshape(const int width, const int height);
void motion(const int x, const int y);
void mouse(int button, int state, int x, int y);
void mouseMotion(int x, int y);

typedef struct {
	float pos[4];
	float diffuse[4];
	float specular[4];
	float ambient[4];
} Light;

Light light = {
  {0,0,-50,1},	//position (w=0 --> directional light, w=1 --> point light)
  {0.8f,0.8f,0.8f,1},  //diffuse
  {0,0,0,1},  //specular
  {0.15f,0.15f,0.15f,1}  //ambient
};
bool rotate_light = false;

// The kinect data that we're going to load from file:
bool draw_point_cloud = false;
bool draw_wireframes = false;
bool draw_winged_edge = false;
bool draw_contour = true;
bool draw_hand = true;
MeshSimplification kinect_mesh_simplifier;

const uint32_t num_plane_subdivisions = 30;  // n*n per face (6 faces)
const float plane_fraction_holes = 0.05;
const bool simple_plane = true;  // if true, the plane is just a regular grid
TestPlane* plane;
mesh_simplification::MeshSettings mesh_settings;
MeshSimplification* plane_mesh_simplifier;
uint32_t edge_reduction_plane = 0;

Vector<std::string> usage_text;

void initGL(void) {
  glClearColor(0.0, 0.0, 0.0, 0.0);
  
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST); 
  glCullFace(GL_BACK);
  
  // Create just one light
  glShadeModel(GL_SMOOTH);
  glEnable(GL_NORMALIZE);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_COLOR_MATERIAL);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light.ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light.diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light.specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light.pos);
  
  camera->updateProjection(window_width, window_height, camera_fov, 
                           camera_znear, camera_zfar);  
}

void initTextures() { 
  glEnable(GL_TEXTURE_2D);
  glGenTextures(num_textures, textures);
  
  // Set the texture parameters
  for (int i = 0; i < num_textures; i++)
  {
    glBindTexture(GL_TEXTURE_2D, textures[i]); 
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  }
}

bool isnan(Float3& vec) {
#ifdef __APPLE__
  return isnan(vec[0]) || isnan(vec[1]) || isnan(vec[2]);
#else 
  return _isnan(vec[0]) || _isnan(vec[1]) || _isnan(vec[2]);
#endif
}

void updatePlaneData() {
  // vertices and rgb data will be modified, make a backup
  *plane->vertices_simplified() = *plane->vertices();
  *plane->indices_simplified() = *plane->indices();
  
  plane_mesh_simplifier->simplifyMesh(edge_reduction_plane, 
    plane->vertices_simplified(), plane->indices_simplified(), plane->normals(),
    NULL);

  if (plane->indices_simplified()->size() % 3 != 0) {
    printf("ERROR: sizeof(indices) must be a multiple of 3\n");
    exit(-1);
  }  
}

void RenderStrokeFontString(int x, int y, void *font,
                            const unsigned char *string, double scale) {
  //  double H = glutStrokeWidth(font, ' ');
  //  double W = glutStrokeLength(font, string);
  
  glPushMatrix();
  glTranslated(x, y, 0);
  glScaled(scale, scale, scale);
  for (const unsigned char *c = string; *c; c++)
    glutStrokeCharacter(font, *c);
  glPopMatrix();
}

void RenderStrokeFontString(Float3* xyz, void *font,
                            const unsigned char *string, double scale) {
  //  double H = glutStrokeWidth(font, ' ');
  //  double W = glutStrokeLength(font, string);
  
  glPushMatrix();
  glTranslatef(xyz->m[0], xyz->m[1], xyz->m[2]);
  glScaled(scale, scale, scale);
  for (const unsigned char *c = string; *c; c++)
    glutStrokeCharacter(font, *c);
  glPopMatrix();
}

char fps_string[256];
void drawHud() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluOrtho2D(0, 1, 0, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  glPushMatrix();
  glScalef(1.0f/window_width, 1.0f/window_height, 1.0f);
  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
  glLineWidth(2);
  glDisable(GL_LIGHTING);
  
  for (uint32_t i = 0; i < usage_text.size(); i++) {
    RenderStrokeFontString(5.0f, 20.0f*static_cast<float>(i+1), GLUT_STROKE_ROMAN, 
                           reinterpret_cast<const unsigned char *>(usage_text[usage_text.size()-1-i].c_str()), 
                           0.15f);
  }
  glLineWidth(1);
  glPopMatrix();
}

void drawStuff();

#define NUM_FRAMES_BETWEEN_STATS 60
double t0; double t1; double t2; double t3;
double contour_time = 0;
double mesh_time = 0;
Float3 cur_dir(0.0f, 0.0f, 0.0f);
Float3 delta_pos;
Float4x4 y_rotation;
uint32_t frame_number = 0;
void display() {
  // Not part of the update time since this is an operation that wouldn't
  // happen in real-world usage.  In general this kinect data is generated by
  // openNI
  hand_mesh_vertices = hand_mesh_vertices_raw;
  rgb = rgb_raw;
  
  /*
  memcpy(hand_mesh_vertices.at(0), hand_mesh_vertices_raw.at(0), 
    hand_mesh_vertices_raw.size() * sizeof(hand_mesh_vertices_raw[0]));
  hand_mesh_vertices.resize(hand_mesh_vertices_raw.size());
  memcpy(rgb.at(0), rgb_raw.at(0), rgb_raw.size() * sizeof(rgb_raw[0]));
  rgb.resize(rgb_raw.size());
  */
   
  frame_number++;
  t0 = t1;
  updatePlaneData();
  t1 = clk->getTime();
  
  hand_contour_simplifier->simplifyContour(target_contour_size, 
    &hand_mesh_vertices, &pts_mask, width, height);
  t2 = clk->getTime();
  contour_time += t2-t1;

  updateKinectData();
  t3 = clk->getTime();
  mesh_time += t3-t2;
   
  
  if (frame_number % NUM_FRAMES_BETWEEN_STATS == 0) {
    double mesh_calc_time = mesh_time / NUM_FRAMES_BETWEEN_STATS;
    mesh_time = 0;
    printf("Hand mesh calculation time = %f\n", mesh_calc_time);
    printf("   -> Hand mesh # faces = %d\n", hand_mesh_indices.size()/3);
    printf("   -> Plane mesh # faces = %d\n", plane->indices_simplified()->size()/3);
    double contour_calc_time = contour_time / NUM_FRAMES_BETWEEN_STATS;
    contour_time = 0;
    printf("Hand mesh contour calculation time = %f\n", contour_calc_time);

  }
  
  drawStuff();
  
  std::this_thread::yield();  // let someone else do some work
}

void drawStuff() {
  // Move the camera
  delta_pos.set(&cur_dir);
  if (!delta_pos.equal(0,0,0)) {
    delta_pos.normalize();
    delta_pos.scale(camera_speed * (t1-t0));
    camera->moveCamera(&delta_pos);
  }
  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  CHECK_OPEN_GL_ERROR;
  
  glViewport(0, 0, window_width, window_height);
  CHECK_OPEN_GL_ERROR;
  
  camera->updateProjection(window_width, window_height, camera_fov, 
                           camera_znear, camera_zfar);  
  CHECK_OPEN_GL_ERROR;
  camera->updateView(); 
  CHECK_OPEN_GL_ERROR;
  
  // Note: light position is multiplied by the current matrix stack
  if (rotate_light) {
    Float3 pos(light.pos);
    Float3 pos_rotated;
    y_rotation.rotateMatYAxis(t1-t0);
    Float3::affineTransformPos(&pos_rotated, &y_rotation, &pos);
    light.pos[0] = pos_rotated[0];
    light.pos[1] = pos_rotated[1];
    light.pos[2] = pos_rotated[2];
  }
  glLightfv(GL_LIGHT0, GL_POSITION, light.pos);
  CHECK_OPEN_GL_ERROR;
  
  // Render a point where the light is:
  glPointSize(10.0f);
  glDisable(GL_LIGHTING);
  glBegin(GL_POINTS);
  glColor3f(1.0f, 1.0f, 1.0f);
  glVertex3f(light.pos[0], light.pos[1], light.pos[2]);
  glEnd();
  glEnable(GL_LIGHTING);
  CHECK_OPEN_GL_ERROR;
  
  if (draw_wireframes || draw_winged_edge) {
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    glDisable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST); 
  } else {
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    glEnable(GL_LIGHTING);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST); 
  }
  CHECK_OPEN_GL_ERROR;
  
  drawKinectData();
  CHECK_OPEN_GL_ERROR;
  
  // Draw the test plane
  glPushMatrix();
  glTranslatef(-100.0f, 0, 0);
  glScalef(30.0f, 30.0f, 30.0f);
  glColor4f(1, 1, 1, 1);
  if (!draw_winged_edge) {
    drawVertexIndexNormalArr(plane->vertices_simplified(), 
                             plane->indices_simplified(), 
                             plane->normals(), NULL);
  } else {
    drawWingedEdgeStruct(plane_mesh_simplifier->getWEStructure(), 
                         plane->vertices_simplified());
  }
  glPopMatrix();
  CHECK_OPEN_GL_ERROR;
  
  drawHud();
  CHECK_OPEN_GL_ERROR;
  
  glutSwapBuffers();
  CHECK_OPEN_GL_ERROR;
}

void reshape(int width, int height) {
  window_width = width;
  window_height = height;

  camera->updateProjection(window_width, window_height, camera_fov, 
                           camera_znear, camera_zfar);  
  
  glutSetWindow(main_window);
  glutReshapeWindow(window_width, window_height);
  glutPostRedisplay();

}

int mouse_x, mouse_y, mouse_x_prev, mouse_y_prev;
bool camera_rotate = true;
void mouse(int button, int state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON) {
    if (state == GLUT_DOWN) {
      camera_rotate = true;
      mouse_x = x;
      mouse_y = y;
      mouse_x_prev = x;
      mouse_y_prev = y;
    }
    if (state == GLUT_UP) {
      camera_rotate = false;
    }
  }
  else if (button == GLUT_RIGHT_BUTTON) {
  }
}


void mouseMotion(int x, int y) {  // Only called when a button is down
  mouse_x_prev = mouse_x;
  mouse_y_prev = mouse_y;
  mouse_x = x;
  mouse_y = y;
  if (camera_rotate) {
    int dx = mouse_x - mouse_x_prev; 
    int dy = mouse_y - mouse_y_prev; 
    float theta_x = dx * mouse_speed_rotation;
    float theta_y = dy * mouse_speed_rotation;
    camera->rotateCamera(theta_x, theta_y);
  }
}

void keyboardUp(unsigned char key, int x, int y) {
  switch (key) {
    case 'w':
    case 'W':
      if (cur_dir[2] >= 0) {
        cur_dir[2] -= 1;
      }
      break;     
    case 's':
    case 'S':
      if (cur_dir[2] <= 0) {
        cur_dir[2] += 1;
      }
      break;            
    case 'a':
    case 'A':
      if (cur_dir[0] >= 0) {
        cur_dir[0] -= 1;
      }
      break;      
    case 'd':
    case 'D':
      if (cur_dir[0] <= 0) {
        cur_dir[0] += 1;
      }
      break;        
    case 'q':
    case 'Q':
      if (cur_dir[1] >= 0) {
        cur_dir[1] -= 1;
      }
      break;      
    case 'e':
    case 'E':
      if (cur_dir[1] <= 0) {
        cur_dir[1] += 1;
      }
      break; 
    case '=':
    case '+':
      edge_reduction+=500;
      printf("edge_reduction = %d\n", edge_reduction);
      break;
    case '_':
    case '-':
      edge_reduction = edge_reduction >= 500 ? edge_reduction - 500 : 0;
      printf("edge_reduction = %d\n", edge_reduction);
      break;     
    case '}':
    case ']':
      edge_reduction+=1;
      printf("edge_reduction = %d\n", edge_reduction);
      break;
    case '{':
    case '[':
      edge_reduction = edge_reduction >= 1 ? edge_reduction - 1 : 0;
      printf("edge_reduction = %d\n", edge_reduction);
      break;  
    case '"':
    case '\'':
      edge_reduction_plane+=25;
      printf("edge_reduction_plane = %d\n", edge_reduction_plane);
      break;
    case ':':
    case ';':
      edge_reduction_plane = edge_reduction_plane >= 25 ? edge_reduction_plane - 25 : 0;
      printf("edge_reduction_plane = %d\n", edge_reduction_plane);
      break;
    case '>':
    case '.':
      target_contour_size += 25;
      printf("target_contour_size = %d\n", target_contour_size);
      break;
    case '<':
    case ',':
      target_contour_size = target_contour_size >= 25 ? target_contour_size - 25 : 0;
      printf("target_contour_size = %d\n", target_contour_size);
      break;
    case 'm':
    case 'M':
      target_contour_size += 1;
      printf("target_contour_size = %d\n", target_contour_size);
      break;
    case 'n':
    case 'N':
      target_contour_size = target_contour_size >= 1 ? target_contour_size - 1 : 0;
      printf("target_contour_size = %d\n", target_contour_size);
      break;      
  }
}

void keyboard(unsigned char key, int x, int y) {
  switch (key) {
    case 27: // ESC
      exit(0);
    case 'w':
    case 'W':
      if (cur_dir[2] <= 0) {
        cur_dir[2] += 1;
      }
      break;     
    case 's':
    case 'S':
      if (cur_dir[2] >= 0) {
        cur_dir[2] -= 1;
      }
      break;            
    case 'a':
    case 'A':
      if (cur_dir[0] <= 0) {
        cur_dir[0] += 1;
      }
      break;      
    case 'd':
    case 'D':
      if (cur_dir[0] >= 0) {
        cur_dir[0] -= 1;
      }
      break;        
    case 'q':
    case 'Q':
      if (cur_dir[1] <= 0) {
        cur_dir[1] += 1;
      }
      break;      
    case 'e':
    case 'E':
      if (cur_dir[1] >= 0) {
        cur_dir[1] -= 1;
      }
      break;     
    case 'p':
    case 'P':
      draw_point_cloud = !draw_point_cloud;
      break;
    case 'r':
    case 'R':
      rotate_light = !rotate_light;
      break;
    case 't':
    case 'T':
      draw_wireframes = !draw_wireframes;
      if (draw_wireframes && draw_winged_edge) {
        draw_winged_edge = false;
      }
      break;  
    case 'c':
    case 'C':
      draw_contour = !draw_contour;
      break;
    case 'y':
    case 'Y':
      draw_winged_edge = !draw_winged_edge;
      if (draw_wireframes && draw_winged_edge) {
        draw_wireframes = false;
      }
      break;
    case 'h':
    case 'H':
      draw_hand = !draw_hand;
      break;      
    case 'V':
    case 'v':
      processHandData();
      break;
  }
}

void idle() {
  glutSetWindow(main_window);
  glutPostRedisplay();
   std::this_thread::yield();  // let someone else do some work
}

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
  usage_text.pushBack("USAGE:");
  usage_text.pushBack("mouse + left click - Rotate camera");
  usage_text.pushBack("wasdqe - Move camera");
  usage_text.pushBack("p - draw point cloud");
  usage_text.pushBack("r - rotate light");
  usage_text.pushBack("t - draw wireframe mesh");
  usage_text.pushBack("n - draw contour");
  usage_text.pushBack("h - draw hand mesh");
  usage_text.pushBack("v - process hand data (in ~/Desktop/hand_mesh/*)");
  usage_text.pushBack("-+ - adjust number of hand edges (coarse)");
  usage_text.pushBack("[] - adjust number of hand edges (fine)");
  usage_text.pushBack(";' - adjust number of plane edges (coarse)");
  usage_text.pushBack("<> - adjust number of hand contour edges (coarse)");
  usage_text.pushBack("nm - adjust number of hand contour edges (fine)");  
  
  try {
    clk = new Clock();
    t1 = clk->getTime();
#ifdef _WIN32
    std::string full_filename = std::string("./") + lHand_filename;
#endif
#ifdef __APPLE__
    // std::string full_filename = file_io::GetHomePath() + std::string("Desktop/") + 
    std::string full_filename = std::string("../../../../../../") +  
    lHand_filename;
#endif        
    initKinectData(full_filename);
    camera = new Camera();
    plane = new TestPlane(num_plane_subdivisions, simple_plane, 
                          plane_fraction_holes);

    // Default settings --> SLOW, BUT ROBUST
    mesh_settings.vertex_merge_method = mesh_simplification::VertexMergeOperation::MidpointVertexMerge;
    mesh_settings.edge_constraint = mesh_simplification::EdgeRemovalConstraint::EdgeRemovalConstrainNonManifold;
    mesh_settings.edge_cost_func = mesh_simplification::EdgeCostFunction::EdgeLengthWithCurvature;
    mesh_settings.normal_method = mesh_simplification::NormalApproximationMethod::RobustNormalApproximation;

    // Non-default settings --> FAST, BUT POOR QUALITY
    // mesh_settings.vertex_merge_method = mesh_simplification::VertexMergeOperation::StaticVertexMerge;
    // mesh_settings.edge_constraint = mesh_simplification::EdgeRemovalConstraint::NoEdgeRemovalConstraints;
    // mesh_settings.edge_cost_func = mesh_simplification::EdgeCostFunction::EdgeLengthEdgeCost;
    // mesh_settings.normal_method = mesh_simplification::NormalApproximationMethod::SimpleNormalApproximation;

    plane_mesh_simplifier = new MeshSimplification(mesh_settings);
    hand_mesh_simplifier = new MeshSimplification(mesh_settings);
    hand_contour_simplifier = new ContourSimplification();
    
    glutInit(&argc, argv);
    glutInitWindowSize(window_width, window_height);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    main_window = glutCreateWindow("Mesh Deformation Test");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutKeyboardFunc(keyboard);
    glutKeyboardUpFunc(keyboardUp);
    glutIdleFunc(idle);
    glutMotionFunc(mouseMotion);
    
    initGL();
    CHECK_OPEN_GL_ERROR;
    initTextures();
    
    //  glutCreateMenu(selectDisplay);
    //  glutAddMenuEntry("JJJ", MENU_ID_JJJ);
    //  glutAttachMenu(GLUT_RIGHT_BUTTON);
    
    initGL();
    
    glutMainLoop();
  } catch (std::runtime_error e) {
    printf("%s\n", e.what());
#ifdef _WIN32
    InitCommonControls();
    MessageBox(NULL, (LPCTSTR)string_util::ToWideString(e.what()).c_str(), TEXT("Error"), MB_OK | MB_ICONERROR);
#endif
  }
  
  return 0;
}
