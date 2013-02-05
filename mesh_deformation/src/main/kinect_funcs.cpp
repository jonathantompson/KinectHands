//
//  kinect_funcs.cpp
//
//  Not strictly good C++ coding style, but I just want to segment these here
//  so that main.cpp isn't too crowded.
//

#include <sstream>
#ifdef __APPLE__
#include <dirent.h> // for opendir(), readdir(), and closedir()
#endif
#if defined(WIN32) || defined(_WIN32)
#include "dirent_win.h" // for opendir(), readdir(), and closedir()
#define snprintf _snprintf
#endif
#include <GLUT/glut.h>
#include <math.h>

#include <iostream>
#include "main/kinect_funcs.h"
#include "file_io/file_io.h"
#include "mesh_simplification/mesh_simplification.h"

using math::Float4x4;
using mesh_simplification::MeshSimplification;
using std::stringstream;
using std::string;
using std::endl;

uint32_t size_kinect_data;
float* kinect_float_data_raw = NULL;
unsigned char* kinect_char_data_raw = NULL;
Vector<Float3> rgb_raw;  // This is the original kinect data
Vector<Float3> rgb;  // data after simplification
Vector<unsigned char> pts_mask;
Vector<Float3> hand_mesh_vertices_raw;  // This is the original kinect data
Vector<Float3> hand_mesh_vertices;
Vector<uint32_t> hand_mesh_indices;
Vector<Float3> hand_mesh_normals;
uint32_t edge_reduction = 0;
MeshSimplification* hand_mesh_simplifier;
ContourSimplification* hand_contour_simplifier;
uint32_t target_contour_size = 600;
uint32_t target_contour_size_to_file = 200;

void processHandData() {
#ifdef __APPLE__
  // open the desktop directory
  string desktop_path = file_io::GetHomePath() + string("Desktop/");  
  DIR *dir_desktop = opendir(desktop_path.c_str()); 
  if (!dir_desktop) {
    std::cerr << "Cannot open ~/Desktop/ directory!" << std::endl;
    return;
  }
  closedir(dir_desktop);
  
  // Open the data directory
  string data_path = desktop_path + string("hand_data/");
  DIR *dir_data = opendir(data_path.c_str()); 
  if (!dir_data) {
    std::cerr << "Cannot open " << data_path << " directory!" << std::endl;
    return;
  }
#endif
#if defined(WIN32) || defined(_WIN32)
  // Open the data directory
  string data_path = string("../../matlab/hand_data/");
  DIR *dir_data = opendir(data_path.c_str()); 
  if (!dir_data) {
    std::cerr << "Cannot open " << data_path << " directory!" << std::endl;
    return;
  }
#endif
  
  uint32_t edge_reduction_old = edge_reduction;
  edge_reduction = 0;
  struct dirent *entry;
  while ((entry = readdir(dir_data))) {
    string cur_file = string(entry->d_name);
    if (cur_file.substr(0,6) == string("hands_") &&
        cur_file.substr(cur_file.size() - 4, 4) == string(".bin")) {
      std::cout << "processing " << cur_file << endl;
      initKinectData(data_path + cur_file);
      hand_contour_simplifier->simplifyContour(target_contour_size_to_file, 
        &hand_mesh_vertices, &pts_mask, width, height);
      hand_contour_simplifier->saveContourToFile(data_path + 
                                                 string("contour_") +
                                                 cur_file);
                                                 
      updateKinectData();
      drawStuff();
    }
  }
  edge_reduction = edge_reduction_old;
  
  closedir(dir_data);

}

void initKinectData(std::string filename) {

  // ios::ate -> Postion read pointer at end of file (so we can read the size)
  std::ifstream file(filename.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    throw std::runtime_error(std::string("error opening file:") + 
                             filename);
  }
  
  size_kinect_data = width * height;
  
  if (kinect_float_data_raw == NULL) {
    kinect_float_data_raw = new float[width*height*3];
  }
  if (kinect_char_data_raw == NULL) {
    kinect_char_data_raw = new unsigned char[width*height*4]; // rgb and mask
  }
  
  if (hand_mesh_vertices_raw.capacity() < size_kinect_data) {
    hand_mesh_vertices_raw.capacity(size_kinect_data);
  }
  hand_mesh_vertices_raw.resize(size_kinect_data);
  if (rgb_raw.capacity() < size_kinect_data) {
    rgb_raw.capacity(size_kinect_data);
  }
  rgb_raw.resize(size_kinect_data);
  if (pts_mask.capacity() < size_kinect_data) {
    pts_mask.capacity(size_kinect_data);
  }
  pts_mask.resize(size_kinect_data);  
  
  uint32_t size_bytes = static_cast<uint32_t>(file.tellg());
  if (size_bytes < kinect_data_raw_size) {
    throw std::runtime_error("error, file size is too small");
  }
  file.seekg (0, std::ios::beg);  // Go to the beginning of the file
  
  file.read (reinterpret_cast<char*>(kinect_float_data_raw), 
             width*height*3*sizeof(kinect_float_data_raw[0]));
  file.read (reinterpret_cast<char*>(kinect_char_data_raw), 
             width*height*4*sizeof(kinect_char_data_raw[0]));  
  file.close();
  
  memset(pts_mask.at(0), 0, sizeof(unsigned char)*size_kinect_data);
  Float3 com;  
  uint32_t com_pts = 0;
  com.zeros();
  for (uint32_t i = 0; i < size_kinect_data; i ++) {
    hand_mesh_vertices_raw[i][0] = kinect_float_data_raw[i*3];
    hand_mesh_vertices_raw[i][1] = kinect_float_data_raw[i*3+1];
    hand_mesh_vertices_raw[i][2] = kinect_float_data_raw[i*3+2];
    rgb_raw[i][0] = static_cast<float>(kinect_char_data_raw[i*3])/255.0f;
    rgb_raw[i][1] = static_cast<float>(kinect_char_data_raw[i*3+1])/255.0f;
    rgb_raw[i][2] = static_cast<float>(kinect_char_data_raw[i*3+2])/255.0f;
    pts_mask[i] = kinect_char_data_raw[width*height*3+i];
    
    if (pts_mask[i]) {
      Float3::add(&com, &com, &hand_mesh_vertices_raw[i]);
      com_pts++;
    }    
  }
  Float3::scale(&com, 1.0f / static_cast<float>(com_pts));
  
  // Now, just for debug purposes, we will subtract off the COM
  for (uint32_t i = 0; i < size_kinect_data; i ++) {
    if (pts_mask[i]) {
      Float3::sub(&hand_mesh_vertices_raw[i], &hand_mesh_vertices_raw[i], &com);
    }
  }

  hand_mesh_vertices = hand_mesh_vertices_raw;
  rgb = rgb_raw;
}

void drawVertexIndexNormalArr(Vector<Float3>* vertices, 
                              Vector<uint32_t>* indices,
                              Vector<Float3>* normals,
                              Vector<Float3>* rgb) {
  Float3 cur_normal;  // only used if we're not calculating vertex normals
  Float3 vec1;
  Float3 vec2;

  if (normals == NULL) {
    glPushAttrib(GL_LIGHTING);
    glDisable(GL_LIGHTING);
  }
  
  glBegin(GL_TRIANGLES);
  for (uint32_t i = 0; i < indices->size() / 3; i++) {
    uint32_t v1 = (*indices)[i*3];
    uint32_t v2 = (*indices)[i*3+1];
    uint32_t v3 = (*indices)[i*3+2];
    
    // Vertex 1
    if (normals) {
      glNormal3fv(normals->at(v1)->m);
    }
    if (rgb != NULL) {
      glColor4fv((*rgb)[v1].m); 
    }
    glVertex3f((*vertices)[v1].m[0],
               (*vertices)[v1].m[1],
               (*vertices)[v1].m[2]);
    
    // Vertex 2
    if (normals) {
      glNormal3fv(normals->at(v2)->m);
    }
    if (rgb != NULL) {
      glColor4fv((*rgb)[v2].m); 
    }
    glVertex3f((*vertices)[v2].m[0],
               (*vertices)[v2].m[1],
               (*vertices)[v2].m[2]);  
    
    // Vertex 3
    if (normals) {
      glNormal3fv(normals->at(v3)->m);
    }
    if (rgb != NULL) {
      glColor4fv((*rgb)[v3].m); 
    }
    glVertex3f((*vertices)[v3].m[0],
               (*vertices)[v3].m[1],
               (*vertices)[v3].m[2]);         
  }
  glEnd();  

  if (normals == NULL) {
    glPopAttrib();
  }
}

const Float4 edge_cols[8] = {
  Float4(1, 0, 0, 1),
  Float4(0, 1, 0, 1),
  Float4(0, 0, 1, 1),
  Float4(1, 1, 1, 1),
  Float4(1, 1, 0, 1),
  Float4(1, 0, 1, 1),
  Float4(0, 1, 1, 1),
  Float4(0.5, 0.5, 0.5, 1),
};

Float3 pyramid_vertices[] = {
  Float3( 0,  0,  0),  // Top
  Float3(-1, -1, -5),  // bottom back left
  Float3( 1, -1, -5),  // bottom back right
  Float3( 1,  1, -5),  // bottom front right
  Float3(-1,  1, -5),  // bottom front left
};

const uint32_t pyramid_indicies[] = {
  1, 3, 4,
  1, 2, 3,
  0, 4, 3,
  0, 3, 2,
  0, 2, 1,
  0, 1, 4,
};

Float3 pyramid_forward = Float3(0, 0, 1);  // must be unit length

const uint32_t num_pyramid_indices = 3 * 6;

// #define DRAW_ARROWS

void drawWingedEdgeStruct(Vector<Edge>* we, Vector<Float3>* vertices) {
  Float3 vec;
  Float3 p1;
  Float3 vec_neighbor;
  Float3 p1_neighbor;
  glBegin(GL_LINES);
  for (uint32_t i = 0; i < we->size(); i++) {
    Edge* cur_edge = we->at(i);
    
    int col = 0;
    col |= cur_edge->face_a ? 1 : 0;
    col |= cur_edge->face_b ? 2 : 0;
    if (cur_edge->face_a && cur_edge->face_b && cur_edge->cost == std::numeric_limits<float>::infinity()) {
      col = 0;
    }
  
    uint32_t v1 = cur_edge->v1;
    uint32_t v2 = cur_edge->v2;
    
    if (vertices->at(v1)->m[2] == REMOVED_VERTEX_POSITION_Z ||
        vertices->at(v2)->m[2] == REMOVED_VERTEX_POSITION_Z) {
      continue;  // Don't draw removed vertices
    }
    
    glColor4fv(edge_cols[col].m);
    glVertex3f(vertices->at(v1)->m[0], vertices->at(v1)->m[1], vertices->at(v1)->m[2]);
#ifndef DRAW_ARROWS
    glColor4f(0, 0, 0, 1.0f);
#endif
    glVertex3f(vertices->at(v2)->m[0], vertices->at(v2)->m[1], vertices->at(v2)->m[2]);
  
    // We also want to draw lines showing connectivity information
    vec.sub(vertices->at(v2), vertices->at(v1));
    
    // Draw e1a --> neightbour
    p1[0] = vertices->at(v1)->m[0] + 0.15f * vec[0];
    p1[1] = vertices->at(v1)->m[1] + 0.15f * vec[1];
    p1[2] = vertices->at(v1)->m[2] + 0.15f * vec[2];
    
    uint32_t v1_neighbour = cur_edge->e1a->v1;
    uint32_t v2_neighbour = cur_edge->e1a->v2;
    
    if (v1_neighbour == v1) {
      vec_neighbor.sub(vertices->at(v2_neighbour), vertices->at(v1_neighbour));
      vec_neighbor.scale(0.225f);
      p1_neighbor.add(vertices->at(v1_neighbour), &vec_neighbor);
    } else {
      vec_neighbor.sub(vertices->at(v1_neighbour), vertices->at(v2_neighbour));
      vec_neighbor.scale(0.225f);
      p1_neighbor.add(vertices->at(v2_neighbour), &vec_neighbor);
    }

    glColor4f(0, 1, 1, 1);
    glVertex3f(p1[0], p1[1], p1[2]);
    glVertex3f(p1_neighbor[0], p1_neighbor[1], p1_neighbor[2]);
    
    // Draw e1b --> neightbour 
    v1_neighbour = cur_edge->e1b->v1;
    v2_neighbour = cur_edge->e1b->v2;
    
    if (v1_neighbour == v1) {
      vec_neighbor.sub(vertices->at(v2_neighbour), vertices->at(v1_neighbour));
      vec_neighbor.scale(0.275f);
      p1_neighbor.add(vertices->at(v1_neighbour), &vec_neighbor);
    } else {
      vec_neighbor.sub(vertices->at(v1_neighbour), vertices->at(v2_neighbour));
      vec_neighbor.scale(0.275f);
      p1_neighbor.add(vertices->at(v2_neighbour), &vec_neighbor);
    }
    
    glColor4f(1, 1, 0, 1);
    glVertex3f(p1[0], p1[1], p1[2]);
    glVertex3f(p1_neighbor[0], p1_neighbor[1], p1_neighbor[2]);
    
    // Draw e2b --> neightbour
    p1[0] = vertices->at(v1)->m[0] + 0.85f * vec[0];
    p1[1] = vertices->at(v1)->m[1] + 0.85f * vec[1];
    p1[2] = vertices->at(v1)->m[2] + 0.85f * vec[2];
    
    v1_neighbour = cur_edge->e2b->v1;
    v2_neighbour = cur_edge->e2b->v2;
    
    if (v1_neighbour != v2) {
      vec_neighbor.sub(vertices->at(v2_neighbour), vertices->at(v1_neighbour));
      vec_neighbor.scale(0.725f);
      p1_neighbor.add(vertices->at(v1_neighbour), &vec_neighbor);
    } else {
      vec_neighbor.sub(vertices->at(v1_neighbour), vertices->at(v2_neighbour));
      vec_neighbor.scale(0.725f);
      p1_neighbor.add(vertices->at(v2_neighbour), &vec_neighbor);
    }
    
    glColor4f(0, 1, 1, 1);
    glVertex3f(p1[0], p1[1], p1[2]);
    glVertex3f(p1_neighbor[0], p1_neighbor[1], p1_neighbor[2]);
    
    // Draw e2a --> neightbour 
    v1_neighbour = cur_edge->e2a->v1;
    v2_neighbour = cur_edge->e2a->v2;
    
    if (v1_neighbour != v2) {
      vec_neighbor.sub(vertices->at(v2_neighbour), vertices->at(v1_neighbour));
      vec_neighbor.scale(0.775f);
      p1_neighbor.add(vertices->at(v1_neighbour), &vec_neighbor);
    } else {
      vec_neighbor.sub(vertices->at(v1_neighbour), vertices->at(v2_neighbour));
      vec_neighbor.scale(0.775f);
      p1_neighbor.add(vertices->at(v2_neighbour), &vec_neighbor);
    }
    
    glColor4f(1, 1, 0, 1);
    glVertex3f(p1[0], p1[1], p1[2]);
    glVertex3f(p1_neighbor[0], p1_neighbor[1], p1_neighbor[2]);
  }
  glEnd(); 
  
#ifdef DRAW_ARROWS
  // Draw a pyramid at one end of each line so that we can see which way it is 
  // oriented
  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
  Float3 axis;
  float angle;
  for (uint32_t i = 0; i < we->size(); i++) {
    Edge* cur_edge = we->at(i);
    
    int col = 0;
    col |= cur_edge->face_a ? 1 : 0;
    col |= cur_edge->face_b ? 2 : 0;
    
    uint32_t v1 = cur_edge->v1;
    uint32_t v2 = cur_edge->v2;
    
    if (vertices->at(v1)->equal(REMOVED_VERTEX_POSITION_X, REMOVED_VERTEX_POSITION_Y, REMOVED_VERTEX_POSITION_Z) ||
        vertices->at(v2)->equal(REMOVED_VERTEX_POSITION_X, REMOVED_VERTEX_POSITION_Y, REMOVED_VERTEX_POSITION_Z)) {
      continue;  // Don't draw removed vertices
    }
    
    vec.sub(vertices->at(v2), vertices->at(v1));
    vec.normalize();
    axis.cross(&pyramid_forward, &vec);
    axis.normalize();
    angle = acosf(Float3::dot(&vec, &pyramid_forward));
    
    glColor4fv(edge_cols[col].m);
    glPushMatrix();
    glTranslatef(vertices->at(v2)->m[0],
                 vertices->at(v2)->m[1],
                 vertices->at(v2)->m[2]);
    glScalef(0.01f, 0.01f, 0.01f);  // make them small 
    glRotatef((angle / static_cast<float>(2*M_PI))*360.0f, 
              axis[0], axis[1], axis[2]);
    
    glBegin(GL_TRIANGLES);
    for (uint32_t i = 0; i < num_pyramid_indices; i++) {
      glVertex3fv(pyramid_vertices[pyramid_indicies[i]].m);
    }
    glEnd();
    glPopMatrix();
    
  }
  glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
#endif
}

char angle_str[16];
void drawKinectData() {
  glPushMatrix();
  glScalef(0.4f, 0.4f, 0.4f);
  
  if (draw_hand) {
    if (draw_point_cloud) {
      
      // Draw the point cloud data
      glDisable(GL_LIGHTING);
      glPointSize(3.0f);
      glBegin(GL_POINTS);
      for (uint32_t i = 0; i < size_kinect_data; i++) {
        if (pts_mask[i]) {
          glColor3f(rgb_raw[i][0], rgb_raw[i][1], rgb_raw[i][2]);
          glVertex3f(hand_mesh_vertices_raw[i][0], 
                     hand_mesh_vertices_raw[i][1], 
                     hand_mesh_vertices_raw[i][2]);
        }
      }
      glEnd();
      glEnable(GL_LIGHTING);
    } else {
      if (!draw_winged_edge) {
        drawVertexIndexNormalArr(&hand_mesh_vertices, 
                                 &hand_mesh_indices, 
                                 &hand_mesh_normals, 
                                 &rgb); 
      } else {
        drawWingedEdgeStruct(hand_mesh_simplifier->getWEStructure(), 
                             &hand_mesh_vertices);
      }
    }
  }

  // Draw the contours
  if (draw_contour) {
    glDisable(GL_LIGHTING);
    glLineWidth(1.5);
    Vector<Contour>* contours = hand_contour_simplifier->getContourStructure();
    Vector<uint32_t>* starts = hand_contour_simplifier->getContourStarts();
    glBegin(GL_LINES);
    for (uint32_t i = 0; i < starts->size(); i++) {
      uint32_t cur_contour = *starts->at(i);
      do {
        glColor4fv(edge_cols[(i+1) % 7].m);
        Contour* cont = contours->at(cur_contour);
        Float3* v1 = &cont->v1;
        Float3* v2 = &contours->at(cont->next)->v1;
        glVertex3fv(v1->m);
        glVertex3fv(v2->m);
        cur_contour = cont->next;
      } while (cur_contour != *starts->at(i));
    }   
    glEnd();
    // TEMP CODE: Annotate the cost at the vertex
    glLineWidth(2);
    Float3 midpoint;
    for (uint32_t i = 0; i < starts->size(); i++) {
      uint32_t cur_contour = *starts->at(i);
      do {
        glColor4fv(edge_cols[(i+1) % 7].m);
        Contour* cont = contours->at(cur_contour);
        Float3* v1 = &cont->v1;
        Float3* v2 = &contours->at(cont->next)->v1;
        midpoint.add(v1, v2);
        midpoint.scale(0.5f);
        snprintf(angle_str, sizeof(angle_str), "%.2f", cont->cost);
        RenderStrokeFontString(&midpoint, GLUT_STROKE_ROMAN, 
                               reinterpret_cast<const unsigned char *>(angle_str), 
                               0.01f);
        cur_contour = cont->next;
      } while (cur_contour != *starts->at(i));
    }
    // END TEMP CODE
    
    if (!draw_wireframes && !draw_winged_edge) {
      glEnable(GL_LIGHTING);
    }
  }
  
  glPopMatrix();
}

// pt1, pt2 and pt3 must be counter clockwise
Float3 tmp1;
Float3 tmp2;
void calculateNormal(Float3* normal, Float3* pt0, Float3* pt1, Float3* pt2) {
  tmp1.sub(pt0, pt1);
  tmp2.sub(pt2, pt1);
  normal->cross(&tmp1, &tmp2);
  normal->normalize();
  if (normal->m[2] > 0) {
    normal->scale(-1);
  }
}

void updateKinectData() {
  hand_mesh_indices.resize(0);  // Set the size to zero without deallocation
  uint32_t verts_case;
  uint32_t p0, p1, p2, p3;
  
  // Populate the mesh data with faces
  // Step through the UV map examining each quad of 4 neighbouring vertices.
  // If 3 or more of those vertices are part of the hand, then add the
  // corresponding face.
  for (uint32_t v = 0; v < (height-1); v++) {
    for (uint32_t u = 0; u < (width-1); u++) {
      p0 = v*width + u;          // top left
      p1 = v*width + (u+1);      // top right
      p2 = (v+1)*width + u;      // bottom left
      p3 = (v+1)*width + (u+1);  // bottom right
      
      // Early out for a large number of quads
      if (!pts_mask[p0] && !pts_mask[p1]) {
        continue;
      }
      
      // For each box of 4 vertices, if 3 of them have valid points, add
      // a triangle, if 4 of them have vertices add 4 triangles
      verts_case = 0;
      if (pts_mask[p0]) {
        verts_case = verts_case | 1;
      }
      if (pts_mask[p1]) {
        verts_case = verts_case | 2;
      }
      if (pts_mask[p2]) {
        verts_case = verts_case | 4;
      }
      if (pts_mask[p3]) {
        verts_case = verts_case | 8;
      } 
      
      // Recall: Front face is counter-clockwise
      switch(verts_case) { 
        case 7:  // 0111 = p2 & p1 & p0
          hand_mesh_indices.pushBack(p0);
          hand_mesh_indices.pushBack(p2);
          hand_mesh_indices.pushBack(p1);
          break;
        case 11:  // 1011 = p3 & p1 & p0
          hand_mesh_indices.pushBack(p0);
          hand_mesh_indices.pushBack(p3);
          hand_mesh_indices.pushBack(p1);              
          break;
        case 13:  // 1101 = p3 & p2 & p0
          hand_mesh_indices.pushBack(p0);
          hand_mesh_indices.pushBack(p2);
          hand_mesh_indices.pushBack(p3);               
          break;      
        case 14:  // 1110 = p3 & p2 & p1
          hand_mesh_indices.pushBack(p1);
          hand_mesh_indices.pushBack(p2);
          hand_mesh_indices.pushBack(p3);                
          break;   
        case 15:  // 1111 = p3 & p2 & p1 & p0
          // THIS METHOD JUST SPLITS EVERY QUAD DOWN THE SAME DIAGONAL
          hand_mesh_indices.pushBack(p0);
          hand_mesh_indices.pushBack(p2);
          hand_mesh_indices.pushBack(p1);
          hand_mesh_indices.pushBack(p2);
          hand_mesh_indices.pushBack(p3);
          hand_mesh_indices.pushBack(p1);         
          break;
          
      }
    }
  }
  if (hand_mesh_indices.size() % 3 != 0) {
    printf("ERROR: sizeof(indices) must be a multiple of 3\n");
    exit(-1);
  }
  
  if (edge_reduction != 0) {
    hand_mesh_simplifier->simplifyMesh(edge_reduction, &hand_mesh_vertices,
      &hand_mesh_indices, &hand_mesh_normals, &rgb);
  } else {
    hand_mesh_simplifier->calcNormalsPerVertex(&hand_mesh_normals, 
      &hand_mesh_vertices, &hand_mesh_indices);
  }
  
  if (hand_mesh_indices.size() % 3 != 0) {
    printf("ERROR: sizeof(indices) must be a multiple of 3\n");
    exit(-1);
  }  
}

void checkOpenGLError() {
  GLenum err = glGetError();
  if (err != GL_NO_ERROR) {
    std::stringstream ss;
    ss << "Renderer::checkOpenGLError() - ERROR: '";
    ss << reinterpret_cast<const char *>(gluErrorString(err));
    ss << "'";
    throw std::wruntime_error(ss.str());
  }
}
