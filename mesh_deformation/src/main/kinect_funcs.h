//
//  kinect_funcs.h
//
//  Not strictly good C++ coding style, but I just want to segment these here
//  so that main.cpp isn't too crowded.
//

#ifndef KINECT_FUNCS
#define KINECT_FUNCS

#include "data_str/vector.h"
#include "math/math_types.h"
#include "mesh_simplification/mesh_simplification.h"
#include "contour_simplification/contour_simplification.h"

#if defined(DEBUG) || defined(_DEBUG)
  #define CHECK_OPEN_GL_ERROR checkOpenGLError()
#else
  #define CHECK_OPEN_GL_ERROR
#endif

using data_str::Vector;
using math::Float3;
using math::Float4;
using mesh_simplification::Edge;
using mesh_simplification::MeshSimplification;
using contour_simplification::Contour;
using contour_simplification::ContourSimplification;

void initKinectData(std::string filename);

void drawVertexIndexNormalArr(Vector<Float3>* vertices, 
                              Vector<uint32_t>* indices,
                              Vector<Float3>* normals,
                              Vector<Float3>* rgb);

void drawWingedEdgeStruct(Vector<Edge>* we, Vector<Float3>* vertices);

void drawKinectData();

void calculateNormal(Float3* normal, Float3* pt0, Float3* pt1, Float3* pt2);

void updateKinectData();

void processHandData();
void checkOpenGLError();

const std::string lHand_filename = "Hand.bin";
const uint32_t width = 640;
const uint32_t height = 480;
const uint32_t kinect_data_raw_size = (width*height*3*sizeof(float) + 
                                       width*height*4*sizeof(char));
extern const Float4 edge_cols[8];
extern uint32_t size_kinect_data;
extern float* kinect_float_data_raw;
extern unsigned char* kinect_char_data_raw;
extern Vector<Float3> rgb_raw;  // This is the original kinect data
extern Vector<Float3> rgb;  // data after simplification
extern Vector<unsigned char> pts_mask;
extern Vector<Float3> hand_mesh_vertices_raw;  // This is the original kinect data
extern Vector<Float3> hand_mesh_vertices;
extern Vector<uint32_t> hand_mesh_indices;
extern Vector<Float3> hand_mesh_normals;
extern uint32_t edge_reduction;
extern uint32_t target_contour_size;
extern bool draw_point_cloud;
extern bool draw_wireframes;
extern bool draw_winged_edge;
extern bool draw_hand;
extern MeshSimplification* hand_mesh_simplifier;
extern ContourSimplification* hand_contour_simplifier;
extern bool draw_contour;
void RenderStrokeFontString(int x, int y, void *font,
                            const unsigned char *string, double scale);
void RenderStrokeFontString(Float3* xyz, void *font,
                            const unsigned char *string, double scale);
void drawStuff();

#endif