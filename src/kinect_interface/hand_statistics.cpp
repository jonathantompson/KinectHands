//
//  hand_statistics.cpp
//  openNiSample007
//
//  Created by Jonathan Tompson on 3/19/12.
//  Copyright (c) 2012 NYU. All rights reserved.
//

#include <sys/types.h>
#include <sys/stat.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <limits>
#include "XnCppWrapper.h"
#include "kinect_interface/hand_statistics.h"
#include "rendering/opengl_include.h"
#include "math/math_types.h"
#include "math/math_base.h"
#include "rendering/effects.h"
#include "app/app.h"
#include "kinect_interface/kinect_interface.h"
#include "file_io/file_io.h"
#include "fastlz/fastlz.h"

#ifdef _WIN32
  #ifndef snprintf
    #define snprintf _snprintf_s
  #endif
#endif

#ifdef __APPLE__
#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a > b) ? a : b)
#endif

namespace kinect {
  
  float HandStatistics::hand_search_radius_;
  float HandStatistics::hand_search_radius_sq_;
  bool HandStatistics::static_data_initialized_ = false;
  float HandStatistics::u_quantization_;
  float HandStatistics::rescale_factor_;
  float HandStatistics::v_quantization_;
  const float c_r = FILL_COARSE_RADIUS;
  const float f_r = FILL_FINE_RADIUS;
  const float HandStatistics::floodFillKernel_[N_PTS_FILL][2] = 
  {{-f_r, -f_r}, {-f_r, 0}, {-f_r, +f_r}, {0, +f_r}, {+f_r, +f_r}, {+f_r, 0}, 
   {+f_r, -f_r}, {0, -f_r}, {-c_r, -c_r}, {-c_r, 0}, {-c_r, +c_r}, {0, +c_r},
   {+c_r, +c_r}, {+c_r, 0}, {+c_r, -c_r}, {0, -c_r}};
  float HandStatistics::finger_inner_rad_ = 60.0f;
  float HandStatistics::finger_outer_rad_ = 100.0f;
  uint16_t HandStatistics::finger_threshold_ = 1920;
  int HandStatistics::finger_blur_rad_ = 2;
  float* HandStatistics::pc_image = NULL;
  unsigned char* HandStatistics::rgb_image = NULL;
  unsigned char* HandStatistics::mask_image = NULL;
  unsigned short* HandStatistics::depth_image = NULL;
  uint16_t* HandStatistics::image_for_dt_ = NULL;
  uint16_t* HandStatistics::image_for_dt_compressed_ = NULL;

  using math::Float2;
  using math::Float3;
  using math::Float3x3;
  using app::App;

  float round(float r) {
    return (r > 0.0f) ? floor(r + 0.5f) : ceil(r - 0.5f);
  }
  
  HandStatistics::HandStatistics(KinectInterface* kinect, HandType hand) {
    hand_ = hand;
    kinect_ = kinect;
    int npts_max = KinectInterface::width_ * KinectInterface::height_;
    int size_of_pixel_queue = npts_max > (dim_hand_pixels*dim_hand_pixels) ? 
      npts_max : (dim_hand_pixels*dim_hand_pixels);
    pixel_on_queue_ = new uint8_t[size_of_pixel_queue];
    pixel_queue_ = new int[size_of_pixel_queue];
    axes[0].set(1.0f, 0.0f, 0.0f);
    axes[1].set(0.0f, 1.0f, 0.0f);
    axes[2].set(0.0f, 0.0f, 1.0f);
    eigVals.set(1.0f, 1.0f, 1.0f);
    blur_effect_ = new Effects<uint16_t>(dim_hand_pixels, dim_hand_pixels);
    snprintf(num_fingers_str, sizeof(num_fingers_str), "0");
    num_fingers = 0;
    dirty_uvd_data_ = true;
  }
  
  // NOTE: THIS DESTRUCTOR IS NOT THREAD SAFE!
  HandStatistics::~HandStatistics() {
    delete[] pixel_on_queue_;
    delete[] pixel_queue_;
    delete blur_effect_;
    if (pc_image != NULL) {
      delete[] pc_image;
      pc_image = NULL;
    }
    if (rgb_image != NULL) {
      delete[] rgb_image;
      rgb_image = NULL;
    }
    if (mask_image != NULL) {
      delete[] mask_image;
      mask_image = NULL;
    }    
    if (depth_image != NULL) {
      delete[] depth_image;
      depth_image = NULL;
    }
    if (image_for_dt_compressed_ != NULL) {
      free(image_for_dt_compressed_);
      image_for_dt_compressed_ = NULL;
    }
    if (image_for_dt_ != NULL) {
      free(image_for_dt_);
      image_for_dt_ = NULL;
    }    
  }

  void stereographicUV2XYZ(Float3* XYZ, Float2* UV) {
    float u_sq = UV->m[0]*UV->m[0];
    float v_sq = UV->m[1]*UV->m[1];
    XYZ->m[0] = (2.0f * UV->m[0]) / (1.0f + u_sq + v_sq);
    XYZ->m[1] = (2.0f * UV->m[1]) / (1.0f + u_sq + v_sq);
    XYZ->m[2] = (-1.0f + u_sq + v_sq) / (1.0f + u_sq + v_sq);
  }
  void stereographicXYZ2UV(Float2* UV, Float3* XYZ) {
    UV->m[0] = XYZ->m[0] / (1.0f - XYZ->m[2]);
    UV->m[1] = XYZ->m[1] / (1.0f - XYZ->m[2]);
  }

  void HandStatistics::initHandStatistics() {
    // Maximum extent of the stereographic projection.
    float phi_min = (M_PI) - acosf(OPEN_FINGER_DOT_PROD_CUTOFF);

    // http://en.wikipedia.org/wiki/Stereographic_projection --> Calculate
    // maximum uv coord extents UV coords
    float max_rad_in_UV_ = 1.001f * sinf(phi_min) / (1 - cosf(phi_min));  // 1.001 = a little bit extra to cover overflow
    rescale_factor_ = 2.0f * atanf(max_rad_in_UV_);  // After linearization
    
    u_quantization_ = static_cast<float>(dim_hand_pixels) / 2.0f;
    v_quantization_ = u_quantization_;
     
    static_data_initialized_ = true;
  }

  // Special dot product outside of math class for simplicity
  float Float3Dot(float* a, float b0, float b1, float b2) {
    return a[0]*b0 + a[1]*b1 + a[2]*b2;
  }
  float Float3Dot(float* a, Float3* b) {
    return a[0]*b->m[0] + a[1]*b->m[1] + a[2]*b->m[2];
  }
  
  void HandStatistics::calcBoundingBox() {
    // Get pointers to the internal data in the pts and ptsOBB vectors to avoid
    // function call overhead
    if(this->pts.capacity() > this->ptsOBB.capacity()) {
      this->ptsOBB.capacity(this->pts.capacity());
    }
    this->ptsOBB.resize(this->pts.size());
    
    Float3 pt_proj;
    
    // Find extremal vertecies along each axis (of the eigen basis) to find box
    // dimensions
    pt_proj.zeros();
    
    // Note: Projection of a 3D point onto a line is proj = A.B / ||B||
    // www.euclideanspace.com/maths/geometry/elements/line/projections/index.htm
    // Initialize min and max with first vertex
    float min[3];
    float max[3];
    min[0] = std::numeric_limits<float>::infinity();
    max[0] = -std::numeric_limits<float>::infinity();
    min[1] = std::numeric_limits<float>::infinity();
    max[1] = -std::numeric_limits<float>::infinity();
    min[2] = std::numeric_limits<float>::infinity();
    max[2] = -std::numeric_limits<float>::infinity();
    
    Float3 cur_point;
    
    // Go through each vertex and find min and max O(n)
    for (int i = 0; i < npts; i ++) {
      cur_point.set(pts.at(i*3));
      // We have box center and vertex in object coords 
      // --> take subtraction to find vertex from box center.
      for (int j = 0; j < 3; j ++) {
        float proj = Float3::dot(&cur_point, &axes[j]);
        if (min[j] > proj) 
          min[j] = proj;
        if (max[j] < proj) 
          max[j] = proj;
        ptsOBB.set(i * 3 + j, proj);
      }
    }
    
    boxDimension[0] = ((max[0] - min[0])*0.5f);  // Half the length of a box
    boxDimension[1] = ((max[1] - min[1])*0.5f);  
    boxDimension[2] = ((max[2] - min[2])*0.5f);
    boxCenterOBB[0] = ((max[0] + min[0])*0.5f);  // Half the length of a box
    boxCenterOBB[1] = ((max[1] + min[1])*0.5f);  
    boxCenterOBB[2] = ((max[2] + min[2])*0.5f);
    // This is the box Center in the OBB coordinate system... Transform to world
    // space by the inverse rotation (or just A^T * vec)
    boxCenter[0] = Float3Dot(boxCenterOBB, axes[0][0], axes[1][0], axes[2][0]);
    boxCenter[1] = Float3Dot(boxCenterOBB, axes[0][1], axes[1][1], axes[2][1]);
    boxCenter[2] = Float3Dot(boxCenterOBB, axes[0][2], axes[1][2], axes[2][2]);
    
    // Lots of the later algorithms (*hacks*) need the axes ordered, do it once
    float max_axis = boxDimension[0];
    float second_axis = -std::numeric_limits<float>::infinity();
    axes_order[0] = 0;
    axes_order[1] = -1;
    axes_order[2] = -1;
    for (int i = 1; i < 3; i ++) {
      if (boxDimension[i] > max_axis) {  // axis[i] is the biggest
        axes_order[2] = axes_order[1];
        axes_order[1] = axes_order[0];
        second_axis = max_axis;
        max_axis = boxDimension[i];
        axes_order[0] = i;
      } else if (axes_order[1] == -1 || 
                 boxDimension[i] > second_axis) {  // axis[i] is the 2nd biggest
        axes_order[2] = axes_order[1];
        axes_order[1] = i;
        second_axis = boxDimension[i];
        
      } else if (axes_order[2] == -1) {
        axes_order[2] = i;
      }
    }
    
#ifdef FIND_FINGER_LONGEST_AXIS
    // Check if longest axis is up or down (we need inverted axis sometimes)
    // Here's the trick.  Find the extremal points along the longest axis, then 
    // do a UV search with very small radius it see if there is a surrounding
    // point that will grow the bounding box.  Only one end of the box will grow
    // in this manner.
    Float3 longestAxis(axes[axes_order[0]]);
    float maxFinger = -std::numeric_limits<float>::infinity();
    float minFinger = std::numeric_limits<float>::infinity();
    int indMinFingerPoint = -1;
    int indMaxFingerPoint = -1;
    for (int i = 0; i < npts; i ++) {
      float proj = Float3Dot(pts.at(i*3), &longestAxis);
      if (maxFinger < proj) {
        maxFinger = proj;
        indMaxFingerPoint = i;
      }
      if (minFinger > proj) {
        minFinger = proj;
        indMinFingerPoint = i;
      }
    }
    // Now do a UV search and see how many neighbours pass the flood fill test,
    // the wrist end should have more neighbours that do.
    Float3 vec;
    int numMinEndPoints = 0;
    int numMaxEndPoints = 0;
    kinect_->depth_generator_.ConvertRealWorldToProjective(1, 
                        reinterpret_cast<XnPoint3D*>(pts.at(indMinFingerPoint*3)),
                        reinterpret_cast<XnPoint3D*>(vec.m));
    searchUVForClosePoints(vec.m, pts.at(indMinFingerPoint*3), 
                           FINGER_SEARCH_RADIUS, &numMinEndPoints);
    kinect_->depth_generator_.ConvertRealWorldToProjective(1, 
                        reinterpret_cast<XnPoint3D*>(pts.at(indMaxFingerPoint*3)),
                        reinterpret_cast<XnPoint3D*>(vec.m));
    searchUVForClosePoints(vec.m, pts.at(indMaxFingerPoint*3), 
                           FINGER_SEARCH_RADIUS, &numMaxEndPoints);
    int indFingerPoint = -1;
    if (numMaxEndPoints < numMinEndPoints) {
      indFingerPoint = indMaxFingerPoint;
    } else {
      indFingerPoint = indMinFingerPoint;
    }
#else
    int indFingerPoint = 0;
    int highesty = pts[1];
    
    for (int i = 0; i < npts; i++) {
      if (pts[i*3+1] > highesty) {
        highesty = pts[i*3+1];  // world
        indFingerPoint = i;
      }
    }
#endif
    highpt[0] = *pts.at(3*indFingerPoint);
    highpt[1] = *pts.at(3*indFingerPoint+1);
    highpt[2] = *pts.at(3*indFingerPoint+2);
  
  }
  
  void HandStatistics::chooseHandColor(const Float3* cur_color) {
/*    
    float f1 = fabsf(approx_thumb_angle) / THUMB_ANGLE_THRESH_RAD;
    if (f1 > 1.0f) {
      f1 = 1.0f;
    }
    glColor3f(cur_color->m[0], f1, cur_color->m[2]);
*/
    glColor3f(cur_color->m[0], 0.0f, cur_color->m[2]);
  }
  
  void HandStatistics::drawPoints(const Float3* cur_color) {
    chooseHandColor(cur_color);
    bool depthTestOn = glIsEnabled(GL_DEPTH_TEST) != 0;
    if (depthTestOn)
      glDisable(GL_DEPTH_TEST);
    glPointSize(1.0f);
    glBegin(GL_POINTS);
    
    for (int i = 0; i < npts; i++) {
      int curInd = i*3;
      glVertex3f(*pts.at(curInd), *pts.at(curInd+1), *pts.at(curInd+2)-0.0001f);
    }
    
    glEnd();
    if (depthTestOn)
      glEnable(GL_DEPTH_TEST);
  }
  
  void HandStatistics::drawPointsUVD(const Float3* cur_color) {
    chooseHandColor(cur_color);
    bool depthTestOn = glIsEnabled(GL_DEPTH_TEST) != 0;
    if (depthTestOn)
      glDisable(GL_DEPTH_TEST);
    glBegin(GL_POINTS);
    
    for (int i = 0; i < npts; i++) {
      int curInd = i*3;
      glVertex3f(round(*pts_UVD.at(curInd)), round(*pts_UVD.at(curInd+1)), 0.0f);
    }
    
    glEnd();
    if (depthTestOn)
      glEnable(GL_DEPTH_TEST);
  }
 
  
  void HandStatistics::calcBoxCorner(float* boxCorner, float s0, float s1, 
                                     float s2) {
    tmp_pt_world_[0] = boxCenter[0] + 
    axes[0][0]*s0*boxDimension[0] + 
    axes[1][0]*s1*boxDimension[1] + 
    axes[2][0]*s2*boxDimension[2];
    tmp_pt_world_[1] = boxCenter[1] + 
    axes[0][1]*s0*boxDimension[0] + 
    axes[1][1]*s1*boxDimension[1] + 
    axes[2][1]*s2*boxDimension[2];
    tmp_pt_world_[2] = boxCenter[2] + 
    axes[0][2]*s0*boxDimension[0] + 
    axes[1][2]*s1*boxDimension[1] + 
    axes[2][2]*s2*boxDimension[2];
    kinect_->depth_generator_.ConvertRealWorldToProjective(1, 
                                     reinterpret_cast<XnPoint3D*>(tmp_pt_world_),
                                      reinterpret_cast<XnPoint3D*>(tmp_pt_uvd_));
    boxCorner[0] = tmp_pt_uvd_[0];
    boxCorner[1] = tmp_pt_uvd_[1];
  }
  
  // Only worth doing if you're actually rending all the points in UVD space.
  void HandStatistics::convertPointsToUVD() {
    if (dirty_uvd_data_) {
      if (pts.capacity() > pts_UVD.capacity()) {
        pts_UVD.capacity(pts.capacity());
      }
      pts_UVD.resize(pts.size());
      kinect_->depth_generator_.ConvertRealWorldToProjective(npts, 
                                        reinterpret_cast<XnPoint3D*>(pts.at(0)), 
                                        reinterpret_cast<XnPoint3D*>(pts_UVD.at(0)));
      kinect_->depth_generator_.ConvertRealWorldToProjective(1, 
                                        reinterpret_cast<XnPoint3D*>(boxCenter), 
                                        reinterpret_cast<XnPoint3D*>(boxCenterUVD));
      // Don't draw a solid box in UVD... it's too hard ;-) ) instead just draw
      // the 8 points of the box corners.  So convert these to UVD space.
      calcBoxCorner(&boxCornerUV[0],  1.0f,  1.0f,  1.0f);
      calcBoxCorner(&boxCornerUV[1*2],  1.0f,  1.0f, -1.0f);
      calcBoxCorner(&boxCornerUV[2*2],  1.0f, -1.0f,  1.0f);
      calcBoxCorner(&boxCornerUV[3*2],  1.0f, -1.0f, -1.0f);
      calcBoxCorner(&boxCornerUV[4*2], -1.0f,  1.0f,  1.0f);
      calcBoxCorner(&boxCornerUV[5*2], -1.0f,  1.0f, -1.0f);
      calcBoxCorner(&boxCornerUV[6*2], -1.0f, -1.0f,  1.0f);
      calcBoxCorner(&boxCornerUV[7*2], -1.0f, -1.0f, -1.0f);
      kinect_->depth_generator_.ConvertRealWorldToProjective(1, 
                                        reinterpret_cast<XnPoint3D*>(highpt), 
                                        reinterpret_cast<XnPoint3D*>(highpt_UV));
      if (num_fingers != 0) {
        if (fingers.capacity() > fingers_uvd.capacity()) {
          fingers_uvd.capacity(fingers.capacity());
        }
        fingers_uvd.resize(fingers.size());
        kinect_->depth_generator_.ConvertRealWorldToProjective(num_fingers, 
                                          reinterpret_cast<XnPoint3D*>(fingers.at(0)), 
                                          reinterpret_cast<XnPoint3D*>(fingers_uvd.at(0)));
      }
      dirty_uvd_data_ = false;
    }
  }
  
  float floorf_sym(float value) {
    if (value < 0.0f) {
      return ceilf(value);
    } else {
      return floorf(value);
    }
  }
  
  float ceilf_sym(float value) {
    if (value < 0.0f) {
      return floorf(value);
    } else {
      return ceilf(value);
    }
  }
  
  void HandStatistics::findHandPoints() {
    hand_stats_lock_.lock();
    // Zero the number of points
    npts = 0;
    dirty_uvd_data_ = true;
    // Store pointers to the kinect data for later
    pts_all_world = kinect_->cloud_pts_world_;
    pts_all_uvd = kinect_->cloud_pts_uvd_;
    pts.resize(0);
    switch (hand_) {
      case HandLHand:
        findHandPointsFloodFill(&kinect_->joints_[XN_SKEL_LEFT_HAND], 
          &kinect_->joints_projected_[XN_SKEL_LEFT_HAND]);
        break;
      case HandRHand:
        findHandPointsFloodFill(&kinect_->joints_[XN_SKEL_RIGHT_HAND], 
          &kinect_->joints_projected_[XN_SKEL_RIGHT_HAND]);        
        break;
    }
    hand_stats_lock_.unlock();
  }
  
  // Find the hand points from the kenect point cloud + OpenNI hand point
  // NEW VERSION --> O(m), where m is the number of points in the set
  void HandStatistics::findHandPointsFloodFill(XnPoint3D* ptHand_world, 
                                               XnPoint3D* ptHand_UVD) {
    Float3 vec;
    // Search in a small UV window for the minimum depth value around
    // the hand point.  This makes sure we can seed our connected components 
    // with a real hand point.
    
    // We need the radius to be depth dependant, so transform a point with X 
    // offset back into UV space (using openNI) to calculate radius in UV space
    Float3 UVvec;
    XnPoint3D ptOffset_world;
    XnPoint3D ptOffset_UVD;
    ptOffset_world.X = (*ptHand_world).X - SMALL_HAND_RADIUS;
    ptOffset_world.Y = (*ptHand_world).Y;
    ptOffset_world.Z = (*ptHand_world).Z;
    kinect_->depth_generator_.ConvertRealWorldToProjective(1, &ptOffset_world, 
      &ptOffset_UVD);
    UVvec.set(ptOffset_UVD.X - ptHand_UVD->X, 
              ptOffset_UVD.Y - ptHand_UVD->Y, 
              0.0f); 
    int radius = static_cast<int>(UVvec.length());
    if (radius < SMALL_HAND_RADIUS_MIN_UV)
      radius = SMALL_HAND_RADIUS_MIN_UV;
    
    float minPt[3];
    uint32_t num_iterations = 0;
    while (!searchUVForMinHandPoint(ptHand_UVD, minPt, radius)) {
      radius *= 2;  // If we're unsuccessful then repeat with double the radius
      num_iterations++;
      if (num_iterations > 10) {
        return;  // Give up
      }
    }
    
    // Add the one point that we know about and start a walk checking each 
    // adjacent pixel and if it is within some small theshold then add it to the
    // queue of pixels
    npts = 0;
    int minPtIndex = static_cast<int>(round(minPt[1])) * KinectInterface::width_ + 
                     static_cast<int>(round(minPt[0]));
    memset(pixel_on_queue_, 0, KinectInterface::width_ * 
           KinectInterface::height_ * sizeof(pixel_on_queue_[0]));
    queue_head_ = 0;
    queue_tail_ = 1;  // When queue_head_ == queue_tail_ the queue is empty
    pixel_queue_[queue_head_] = minPtIndex;
    pixel_on_queue_[minPtIndex] = true;
    int neighbourPtIndexUV[2];
    while (queue_head_ != queue_tail_) {
      // Take the current pixel off the queue
      int curPtIndex = pixel_queue_[queue_head_];
      int curPtU = curPtIndex % KinectInterface::width_;
      int curPtV = curPtIndex / KinectInterface::width_;         
      queue_head_++;
      // Add the point to the set
      pts.pushBack(pts_all_world[curPtIndex].X);
      pts.pushBack(pts_all_world[curPtIndex].Y);
      pts.pushBack(pts_all_world[curPtIndex].Z);
      npts++;
      
      for (int i = 0; i < N_PTS_FILL; i ++) {
        int cur_rad_u = static_cast<int>(ceilf_sym(floodFillKernel_[i][0] /  
                                                   static_cast<float>(pts_all_uvd[curPtIndex].Z)));
        int cur_rad_v = static_cast<int>(ceilf_sym(floodFillKernel_[i][1] /  
                                                   static_cast<float>(pts_all_uvd[curPtIndex].Z)));
        neighbourPtIndexUV[0] = curPtU + cur_rad_u;
        neighbourPtIndexUV[1] = curPtV + cur_rad_v;
        processNeighbour(neighbourPtIndexUV, curPtIndex, &vec, ptHand_world);
      }
    }
  }
  
  
  void clipOnScreen(int* uMin, int* uMax, int* vMin, int* vMax, float* 
                    ptHand_UVD, int radius, int width, int height) {
    // Now clamp the UV min and max coordinates so we don't go off screen
    *uMin = static_cast<int>(round(ptHand_UVD[0])) - radius;
    *uMin = (*uMin >= 0) ? *uMin : 0;
    *uMax = static_cast<int>(round(ptHand_UVD[0])) + radius;
    *uMax = (*uMax < width) ? *uMax : (width-1);
    
    *vMin = static_cast<int>(round(ptHand_UVD[1])) - radius;
    *vMin = (*vMin >= 0) ? *vMin : 0;
    *vMax = static_cast<int>(round(ptHand_UVD[1])) + radius;
    *vMax = (*vMax < height) ? *vMax : (height-1);
  }
  void clipOnScreen(int* uMin, int* uMax, int* vMin, int* vMax, 
                    XnPoint3D* ptHand_UVD, int radius, int width, int height) {
    clipOnScreen(uMin, uMax, vMin, vMax, &ptHand_UVD->X, radius, width, height);
  }
  
  // Special sub outside math library (don't want to polute math library)
  void Float3Sub(Float3* ret, float* a, Float3* b) {
    ret->m[0] = a[0] - b->m[0];
    ret->m[1] = a[1] - b->m[1];
    ret->m[2] = a[2] - b->m[2];
  }
  
  bool HandStatistics::searchUVForClosePoints(float* pt_UVD,
                                              float* pt_world, 
                                              int radius,
                                              int* numPoints) {
    int uMin, uMax, vMin, vMax;
    clipOnScreen(&uMin, &uMax, &vMin, &vMax, pt_UVD, radius, 
                 KinectInterface::width_, KinectInterface::height_);
    
    Float3 vec;
    for (int u = uMin; u <= uMax;  u ++) {
      for (int v = vMin; v <= vMax;  v ++) {
        int nIndex = v * KinectInterface::width_ + u;
        if (pts_all_uvd[nIndex].Z > 1) {
          vec[0] = pt_world[0] - pts_all_world[nIndex].X;
          vec[1] = pt_world[1] - pts_all_world[nIndex].Y;
          vec[2] = pt_world[2] - pts_all_world[nIndex].Z;
          float length_sq = vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2];
          if (length_sq < BACKGROUND_THRESH_SQ) {
            (*numPoints)++;
          }
        }
      }
    }
    return true;
  }
  
  bool HandStatistics::searchUVForMinHandPoint(XnPoint3D* ptHand_UVD,
                                               float* minPt, 
                                               int radius) {
    int uMin, uMax, vMin, vMax;
    clipOnScreen(&uMin, &uMax, &vMin, &vMax, ptHand_UVD, radius, 
                 KinectInterface::width_, KinectInterface::height_);
    
    Float3 vec;
    minPt[2] = std::numeric_limits<float>::infinity();
    for (int u = uMin; u <= uMax;  u ++) {
      for (int v = vMin; v <= vMax;  v ++) {
        int nIndex = v * KinectInterface::width_ + u;
        if (pts_all_uvd[nIndex].Z > 1) {
          if (pts_all_uvd[nIndex].Z < minPt[2]) {
            minPt[0] = pts_all_uvd[nIndex].X;
            minPt[1] = pts_all_uvd[nIndex].Y;
            minPt[2] = pts_all_uvd[nIndex].Z;
          }
        }
      }
    }
    // Check if we were sucessful --> this happens when the joint position lags
    // behind the actual depth image...
    if (minPt[2] < std::numeric_limits<float>::infinity()) {
      return true;
    } else {
      return false;
    }
  }
  
  void HandStatistics::processNeighbour(int* nieghbourPtUV, int curPtIndex,
                                        Float3* tmp, XnPoint3D* ptHand) {
    if (nieghbourPtUV[0] >= 0 && nieghbourPtUV[0] < KinectInterface::width_ && 
        nieghbourPtUV[1] >= 0 && nieghbourPtUV[1] < KinectInterface::height_) {
      int nieghbourPtIndex = nieghbourPtUV[1]*KinectInterface::width_ + nieghbourPtUV[0];
      if (!pixel_on_queue_[nieghbourPtIndex]) {  // don't add it if we have
        // See if the pixel is a user pixel and that it is outside our big rad
        tmp->m[0] = pts_all_world[nieghbourPtIndex].X - ptHand->X;
        tmp->m[1] = pts_all_world[nieghbourPtIndex].Y - ptHand->Y;
        tmp->m[2] = pts_all_world[nieghbourPtIndex].Z - ptHand->Z;
        float len_sq = (tmp->m[0] * tmp->m[0] + tmp->m[1] * tmp->m[1] + 
                        tmp->m[2] * tmp->m[2]); 
        if (pts_all_uvd[nieghbourPtIndex].Z > 0 && 
            len_sq < hand_search_radius_sq_) {
          // See if the pixel is close to the current point (otherwise it's the 
          // background)
          tmp->m[0] = pts_all_world[nieghbourPtIndex].X - 
                      pts_all_world[curPtIndex].X;
          tmp->m[1] = pts_all_world[nieghbourPtIndex].Y - 
                      pts_all_world[curPtIndex].Y;
          tmp->m[2] = pts_all_world[nieghbourPtIndex].Z - 
                      pts_all_world[curPtIndex].Z;     
          len_sq = (tmp->m[0] * tmp->m[0] + tmp->m[1] * tmp->m[1] + 
                    tmp->m[2] * tmp->m[2]);          
          if (len_sq < BACKGROUND_THRESH_SQ) {
            pixel_queue_[queue_tail_] = nieghbourPtIndex;
            pixel_on_queue_[nieghbourPtIndex] = true;
            queue_tail_++;
          }
        }
      }
    }
  }
  
  void HandStatistics::calculateStatistics() {
    hand_stats_lock_.lock();
    
    if (npts < min_npts) {
      hand_stats_lock_.unlock();
      return;
    }
    
    // Update the old data from last frame
    axes_old[0].set(&axes[0]);
    axes_old[1].set(&axes[1]);
    axes_old[2].set(&axes[2]);
    eigVals_old.set(&eigVals);
    
    // Perform the statistics for this frame
    math::PCA3DPoints(axes, &cov, &com, &eigVals, pts.at(0), 
                      static_cast<unsigned int>(npts)); 
    
    // Since there is a sign +1/-1 ambiguity in A.vec = val.vec we need to look 
    // at the axes that come back from the PCA and make sure they are consistant
    // with the previous frames (PCA will order based on eigenval size).
    orderAxes();
    calcBoundingBox();
 
    testForThumb();
    // testForOpenFingers();
    // approxHandUVDimensions();
    
    hand_stats_lock_.unlock();
  }
  
  void findClosestAxesOf3Axes(Float3* axis, Float3* axes_old, 
                              int* ind, bool* flipped) {
    float dotProds[3];
    dotProds[0] = Float3::dot(axis, &axes_old[0]);
    dotProds[1] = Float3::dot(axis, &axes_old[1]);
    dotProds[2] = Float3::dot(axis, &axes_old[2]);
    if (fabsf(dotProds[0]) >= fabsf(dotProds[1])) {
      if (fabsf(dotProds[0]) >= fabsf(dotProds[2])) {  // 0>=1 && 0>=2
        *ind = 0;  // Axis 0 is the closest
        if (dotProds[0] < 0) {
          *flipped = true;
        } else {
          *flipped = false;
        }
      } else {  // 0>=1 && 2<0
        *ind = 2;  // Axis 2 is the closest
        if (dotProds[2] < 0) { 
          *flipped = true;
        } else {
          *flipped = false;
        }
      }
    } else {  // 1>0
      if (fabsf(dotProds[1]) >= fabsf(dotProds[2])) {  // 1>0 && 1>=2
        *ind = 1;  // Axis 1 is the closest
        if (dotProds[1] < 0) {
          *flipped = true;
        } else {
          *flipped = false;
        }
      } else {  // 1>0 && 2>1
        *ind = 2;  // Axis 2 is the closest
        if (dotProds[2] < 0) {
          *flipped = true;
        } else {
          *flipped = false;
        }
      }
    }
  }
  
  void findClosestAxesOf2Axes(int axisA, int axisB, Float3* axis, 
                              Float3* axes_old, int* ind, bool* flipped) {
    float dotProds[2];
    dotProds[0] = Float3::dot(axis, &axes_old[axisA]);
    dotProds[1] = Float3::dot(axis, &axes_old[axisB]);
    if (fabsf(dotProds[0]) >= fabsf(dotProds[1])) {
      *ind = axisA;  // axisA is the closest
      if (dotProds[0] < 0) {
        *flipped = true;
      } else {
        *flipped = false;
      }
    } else {  // 1>0
      *ind = axisB;  // axisB is the closest
      if (dotProds[1] < 0) {
        *flipped = true;
      } else {
        *flipped = false;
      }
    }
  }
  
  // resolveAxisTripleEquality: assumes all axes are the same.  Check before 
  // calling.
  void HandStatistics::resolveAxisTripleEquality(int a, int b, int c, 
                                                 int* ind, bool* flipped) {
    int winningAxis = -1;
    int loserAxis1 = -1;
    int loserAxis2 = -1;
    float diffA = fabsf(eigVals.m[a]-eigVals_old.m[ind[a]]);
    float diffB = fabsf(eigVals.m[b]-eigVals_old.m[ind[b]]);
    float diffC = fabsf(eigVals.m[c]-eigVals_old.m[ind[c]]);
    if (diffA < diffB) { 
      if (diffA < diffC) {  // A wins, choose B and C
        winningAxis = a;
        loserAxis1 = b;
        loserAxis2 = c;
      } else {  // C wins, choose A and B
        winningAxis = c;
        loserAxis1 = a;
        loserAxis2 = b;
      }
    } else {
      if (diffB < diffC) {  // B wins, choose A and C
        winningAxis = b;
        loserAxis1 = a;
        loserAxis2 = c;
      } else {  // C wins, choose A and B
        winningAxis = c;
        loserAxis1 = a;
        loserAxis2 = b;
      }
    }
    // Now re-evaluate the closest axes of the two loosers with the axes that
    // are left over
    switch (ind[winningAxis]) {
      case 0:  // Loser axes choose choose from axis 1 and 2
        findClosestAxesOf2Axes(1, 2, &axes[loserAxis1], axes_old, 
                               &ind[loserAxis1], &flipped[loserAxis1]);
        findClosestAxesOf2Axes(1, 2, &axes[loserAxis2], axes_old, 
                               &ind[loserAxis2], &flipped[loserAxis2]);
        break;
      case 1:  // Loser axes choose choose from axis 0 and 2
        findClosestAxesOf2Axes(0, 2, &axes[loserAxis1], axes_old, 
                               &ind[loserAxis1], &flipped[loserAxis1]);
        findClosestAxesOf2Axes(0, 2, &axes[loserAxis2], axes_old, 
                               &ind[loserAxis2], &flipped[loserAxis2]);
        break;
      case 2:  // Loser axes choose choose from axis 1 and 1
        findClosestAxesOf2Axes(0, 1, &axes[loserAxis1], axes_old, 
                               &ind[loserAxis1], &flipped[loserAxis1]);
        findClosestAxesOf2Axes(0, 1, &axes[loserAxis2], axes_old, 
                               &ind[loserAxis2], &flipped[loserAxis2]);
        break;
    }
  }
  
  // Resolve equality between A and B, C is the previous winner, assumes A==B.  
  // Check before calling.
  void HandStatistics::resolveAxisDoubleEquality(int a, int b, int c, 
                                                 int* ind, bool* flipped) {
    double dotProd;
    int winningAxis = -1;
    int loserAxis = -1;
    int looserAxisAllocation = -1;
    
    // Axis with closest eigval wins out
    if (fabsf(eigVals.m[a]-eigVals_old.m[ind[a]]) <   
        fabsf(eigVals.m[b]-eigVals_old.m[ind[b]])) {
      winningAxis = a;
      loserAxis = b;
    } else {
      winningAxis = b;
      loserAxis = a;
    }
    // Now figure out which index to assign the looser
    switch (ind[c] + ind[winningAxis]) {
      case 1:  // 0 & 1
        looserAxisAllocation = 2;
        break;
      case 2:  // 0 & 2
        looserAxisAllocation = 1;
        break;
      case 3:  // 1 & 2
        looserAxisAllocation = 0;
        break;
    }
    ind[loserAxis] = looserAxisAllocation;
    dotProd = Float3::dot(&axes[loserAxis], &axes_old[looserAxisAllocation]);
    if (dotProd < 0) {
      flipped[loserAxis] = true;
    } else {
      flipped[loserAxis] = false;
    }
  }
  
  void HandStatistics::orderAxes() {
    int axes_indices[3] = {0, 1, 2};
    bool axes_flipped[3] = {false, false, false};
    // For each axis, find which of the previous frame's axes are closest to
    // perpendicular
    findClosestAxesOf3Axes(&axes[0], axes_old, &axes_indices[0], 
                           &axes_flipped[0]);
    findClosestAxesOf3Axes(&axes[1], axes_old, &axes_indices[1], 
                           &axes_flipped[1]);
    findClosestAxesOf3Axes(&axes[2], axes_old, &axes_indices[2], 
                           &axes_flipped[2]);
    
    // At this point, since we can flip all the current axes we may find that 
    // all axes are closest to the same previous axis --> Resolve this
    if ( axes_indices[0] == axes_indices[1] && 
         axes_indices[0] == axes_indices[2] )
      resolveAxisTripleEquality(0, 1, 2, axes_indices, axes_flipped);
    // Now only two axes may be the same: Resolve this, only does work as needed
    if ( axes_indices[0] == axes_indices[1] )
      resolveAxisDoubleEquality(0, 1, 2, axes_indices, axes_flipped);
    if ( axes_indices[0] == axes_indices[2] )
      resolveAxisDoubleEquality(0, 2, 1, axes_indices, axes_flipped);
    if ( axes_indices[1] == axes_indices[2] )
      resolveAxisDoubleEquality(1, 2, 0, axes_indices, axes_flipped);
    
    // Double check that we have resolved all possible axis collisions
    // Exit because this is a very bad error condition
    if (axes_indices[0] == axes_indices[1]) {
      printf("WARNING: axes_indices[0] == axes_indices[1]\n");
      // exit(-1);
    }
    if (axes_indices[0] == axes_indices[2]) {
      printf("WARNING: axes_indices[0] == axes_indices[2]\n");
      // exit(-1);
    }
    if (axes_indices[1] == axes_indices[2]) {
      printf("WARNING: axes_indices[1] == axes_indices[2]\n");
      // exit(-1);
    }
    
    if (axes_flipped[0])
      axes[0].scale(-1.0f);
    if (axes_flipped[1])
      axes[1].scale(-1.0f);
    if (axes_flipped[2])
      axes[2].scale(-1.0f);
    
    // Put the axes and the eigenvalues where they should be
    switch (axes_indices[0]) {
      case 0:
        axesTmp_[0].set(&axes[0]);
        eigValsTmp_[0]=eigVals[0];
        break;
      case 1:
        axesTmp_[1].set(&axes[0]);
        eigValsTmp_[1]=eigVals[0];
        break;
      case 2:
        axesTmp_[2].set(&axes[0]);
        eigValsTmp_[2]=eigVals[0];
        break;
    }
    switch (axes_indices[1]) {
      case 0:
        axesTmp_[0].set(&axes[1]);
        eigValsTmp_[0]=eigVals[1];
        break;
      case 1:
        axesTmp_[1].set(&axes[1]);
        eigValsTmp_[1]=eigVals[1];
        break;
      case 2:
        axesTmp_[2].set(&axes[1]);
        eigValsTmp_[2]=eigVals[1];
        break;
    }
    switch (axes_indices[2]) {
      case 0:
        axesTmp_[0].set(&axes[2]);
        eigValsTmp_[0]=eigVals[2];
        break;
      case 1:
        axesTmp_[1].set(&axes[2]);
        eigValsTmp_[1]=eigVals[2];
        break;
      case 2:
        axesTmp_[2].set(&axes[2]);
        eigValsTmp_[2]=eigVals[2];
        break;
    }
    axes[0].set(&axesTmp_[0]);
    axes[1].set(&axesTmp_[1]);
    axes[2].set(&axesTmp_[2]);
    eigVals.set(&eigValsTmp_);
  }
  
  int HandStatistics::computeOutMatrix() {
    hand_stats_lock_.lock();
    int i = 0;
    
    //--------------------------
    // scale in x, y, z
    //--------------------------   
    outmat[i++] = boxDimension[0];
    outmat[i++] = boxDimension[1];
    outmat[i++] = boxDimension[2];
    
    //--------------------------
    // axes of rotation
    //--------------------------   
    outmat[i++] = axes[0][0];
    outmat[i++] = axes[0][1];
    outmat[i++] = axes[0][2];
    
    outmat[i++] = axes[1][0];
    outmat[i++] = axes[1][1];
    outmat[i++] = axes[1][2];
    
    outmat[i++] = axes[2][0];
    outmat[i++] = axes[2][1];
    outmat[i++] = axes[2][2];
    
    //--------------------------
    // translate box by xyz
    //--------------------------   
    outmat[i++] = boxCenter[0];
    outmat[i++] = boxCenter[1];
    outmat[i++] = boxCenter[2];
    
    
    //--------------------------
    // copy hand center of mass
    //--------------------------   
    outmat[i++] = com[0];
    outmat[i++] = com[1];
    outmat[i++] = com[2];
    
    //--------------------------
    // copy highest point (in y)
    //--------------------------   
    outmat[i++] = highpt[0];
    outmat[i++] = highpt[1];
    outmat[i++] = highpt[2];
    
    //--------------------------
    // copy number of points in hand
    //--------------------------   
    outmat[i++] = static_cast<float>(npts);
    
    //--------------------------
    // copy number of fingers
    //--------------------------   
    outmat[i++] = static_cast<float>(num_fingers);
    
    //--------------------------
    // copy approx thumb angle
    //--------------------------   
    outmat[i++] = static_cast<float>(approx_thumb_angle);
    
    int ret_val = i * sizeof(outmat[0]);    

    hand_stats_lock_.unlock();

    return ret_val;
  }
  
  void HandStatistics::drawSolidCube() {

    glCullFace(GL_FRONT);
    glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    float a = 0.8f;
    glBegin(GL_QUADS);  // Draw The Cube Using quads
    glColor4f(1.0f, 0.0f, 0.0f, a);
    glNormal3d(0, 1, 0);
    glVertex3f(1.0f, 1.0f, -1.0f);  // Top Right Of The Quad (Top)
    glVertex3f(-1.0f, 1.0f, -1.0f);  // Top Left Of The Quad (Top)
    glVertex3f(-1.0f, 1.0f, 1.0f);  // Bottom Left Of The Quad (Top)
    glVertex3f(1.0f, 1.0f, 1.0f);  // Bottom Right Of The Quad (Top)
    
    glColor4f(1.0f, 1.0f, 0.0f, a);
    glNormal3d(0, -1, 0);
    glVertex3f(1.0f, -1.0f, 1.0f);  // Top Right Of The Quad (Bottom)
    glVertex3f(-1.0f, -1.0f, 1.0f);  // Top Left Of The Quad (Bottom)
    glVertex3f(-1.0f, -1.0f, -1.0f);  // Bottom Left Of The Quad (Bottom)
    glVertex3f(1.0f, -1.0f, -1.0f);  // Bottom Right Of The Quad (Bottom)
    
    glColor4f(1.0f, 0.0f, 1.0f, a);
    glNormal3d(0, 0, 1);
    glVertex3f(1.0f, 1.0f, 1.0f);  // Top Right Of The Quad (Front)
    glVertex3f(-1.0f, 1.0f, 1.0f);  // Top Left Of The Quad (Front)
    glVertex3f(-1.0f, -1.0f, 1.0f);  // Bottom Left Of The Quad (Front)
    glVertex3f(1.0f, -1.0f, 1.0f);  // Bottom Right Of The Quad (Front)
    
    glColor4f(0.0f, 1.0f, 1.0f, a);
    glNormal3d(0, 0, -1);
    glVertex3f(1.0f, -1.0f, -1.0f);  // Top Right Of The Quad (Back)
    glVertex3f(-1.0f, -1.0f, -1.0f);  // Top Left Of The Quad (Back)
    glVertex3f(-1.0f, 1.0f, -1.0f);  // Bottom Left Of The Quad (Back)
    glVertex3f(1.0f, 1.0f, -1.0f);  // Bottom Right Of The Quad (Back)
    
    glColor4f(0.0f, 0.0f, 1.0f, a);
    glNormal3d(-1, 0, 0);
    glVertex3f(-1.0f, 1.0f, 1.0f);  // Top Right Of The Quad (Left)
    glVertex3f(-1.0f, 1.0f, -1.0f);  // Top Left Of The Quad (Left)
    glVertex3f(-1.0f, -1.0f, -1.0f);  // Bottom Left Of The Quad (Left)
    glVertex3f(-1.0f, -1.0f, 1.0f);  // Bottom Right Of The Quad (Left)
    
    glColor4f(0.0f, 1.0f, 0.0f, a);
    glNormal3d(1, 0, 0);
    glVertex3f(1.0f, 1.0f, -1.0f);  // Top Right Of The Quad (Right)
    glVertex3f(1.0f, 1.0f, 1.0f);  // Top Left Of The Quad (Right)
    glVertex3f(1.0f, -1.0f, 1.0f);  // Bottom Left Of The Quad (Right)
    glVertex3f(1.0f, -1.0f, -1.0f);  // Bottom Right Of The Quad (Right)
    
    glEnd();  
    // glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
  }
  
  float HandStatistics::tmpMat_[16];
  void HandStatistics::drawCube() {
    glBegin(GL_LINES);
    
    // Sides of a cube
    
    glVertex3f(-1, -1, -1);  // bottom front
    glVertex3f(+1, -1, -1);
    
    glVertex3f(-1, -1, -1);  // bottom side left
    glVertex3f(-1, -1, +1);
    
    glVertex3f(-1, -1, -1);  // vertical front left 
    glVertex3f(-1, +1, -1);
    
    glVertex3f(+1, -1, +1);  // bottom back
    glVertex3f(-1, -1, +1);
    
    glVertex3f(+1, -1, +1);  // bottom right
    glVertex3f(+1, -1, -1);
    
    glVertex3f(+1, -1, +1);  // vertical back right
    glVertex3f(+1, +1, +1);
    
    glVertex3f(+1, -1, -1);  // vertical front right 
    glVertex3f(+1, +1, -1);
    
    glVertex3f(-1, -1, +1);  // vertical back left 
    glVertex3f(-1, +1, +1);
    
    glVertex3f(-1, +1, -1);  // top left
    glVertex3f(-1, +1, +1);
    
    glVertex3f(+1, +1, +1);  // top back
    glVertex3f(-1, +1, +1);
    
    glVertex3f(+1, +1, +1);  // top right
    glVertex3f(+1, +1, -1);
    
    glVertex3f(-1, +1, -1);  // top front
    glVertex3f(+1, +1, -1);
    
    glEnd();
  }
  
  void HandStatistics::drawOBB() {
    // Assume hand_stats_lock_ is already locked
    glPushMatrix();
    glLoadIdentity();
    math::calcOpenGLAffine(tmpMat_, axes, boxCenter);
    glMultMatrixf(tmpMat_);
    
    glScalef(boxDimension[0],
             boxDimension[1],
             boxDimension[2]);
    drawCube();
    glPopMatrix();
  }
  
  void HandStatistics::drawSolidOBB() {
    // Assume hand_stats_lock_ is already locked
    glPushMatrix();
    glLoadIdentity();
    math::calcOpenGLAffine(tmpMat_, axes, boxCenter);
    glMultMatrixf(tmpMat_);
    
    glScalef(boxDimension[0],
             boxDimension[1],
             boxDimension[2]);
    drawSolidCube();
    glPopMatrix();
  }
  
  // Static:
  float HandStatistics::OBBPointColors[8*3] = {0, 0, 0,
    0, 0, 1,
    0, 1, 0,
    0, 1, 1,
    1, 0, 0,  
    1, 0, 1,
    1, 1, 0,
    1, 1, 1};
  
  void HandStatistics::drawOBBPointsUVD() {
    // Assume hand_stats_lock_ is already locked
    GLboolean old_depthTest;
    glGetBooleanv(GL_DEPTH_TEST, &old_depthTest);
    glDisable(GL_DEPTH_TEST);
    GLfloat old_ps;
    glGetFloatv(GL_POINT_SIZE, &old_ps);
    glPointSize(8.0f);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glColorPointer(3, GL_FLOAT, 0, OBBPointColors);
    glVertexPointer(2, GL_FLOAT, 0, boxCornerUV);
    glDrawArrays(GL_POINTS, 0, 8);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glPointSize(old_ps);
    if (old_depthTest)
      glEnable(GL_DEPTH_TEST);
  }
  
  void HandStatistics::drawFingerPointUVD(const math::Float3* cur_color) {
    // Assume hand_stats_lock_ is already locked
    GLboolean old_depthTest;
    glGetBooleanv(GL_DEPTH_TEST, &old_depthTest);
    glDisable(GL_DEPTH_TEST);
    GLfloat old_ps;
    glGetFloatv(GL_POINT_SIZE, &old_ps);
    glPointSize(20.0f);
    glBegin(GL_POINTS);
    for (uint32_t i = 0; i < num_fingers; i++) {
      uint32_t cur_color = i % 8;
      glColor3f(OBBPointColors[cur_color*3],OBBPointColors[cur_color*3+1],
                OBBPointColors[cur_color*3+2]);
      glVertex2f(*fingers_uvd.at(3*i), *fingers_uvd.at(3*i+1));
    }
    glColor3fv(cur_color->m);
    glVertex2f(highpt_UV[0], highpt_UV[1]);    
    
    glEnd();
    
    /*
    snprintf(num_fingers_str, sizeof(num_fingers_str), "%d", num_fingers);
    // Also draw how many fingers are being held up below the finger point
    glPushMatrix();  // Push to 3
    glScalef(1.0f, -1.0f, 1.0f);
    glColor3fv(cur_color->m);
    App::renderStrokeFontString(highpt_UV[0], -(highpt_UV[1]-40), 
                                GLUT_STROKE_ROMAN,
                                reinterpret_cast<const unsigned char *>(num_fingers_str),
                                0.5f);
    glPopMatrix();
    */
    
    glPointSize(old_ps);
    if (old_depthTest)
      glEnable(GL_DEPTH_TEST);
  }
  
  const float HandStatistics::thumbTestPoints_[4] = {-1.0f, -0.5f, 0.5f, 1.0f};
  
  // testForThumb - 2 potential methods:
  // 1. Using points within a sliced region of the 2nd longest axis.  Perform 
  //    PCA to test if the thumb is pointing out.  <-- USING THIS ONE
  // 2. Just take the extremal points along the segments and test is if vector
  //    formed between them is parallel to the 2nd longest axis.
  float angle[2];
  void HandStatistics::testForThumb() {   
    // ALREADY HAVE LOCK
    
    // OPTION 2: Find the thumb with more SVD
    Float3 cur_point;
    float dot_prod;  // project onto axis
    for (int j = 0; j < 2; j++) {  // Angle on both sides of the axis
      pts_thumb_.resize(0);
      // Go through each vertex and add points if they are within the bounds
      for (int i = 0; i < npts; i ++) {
        cur_point.set(pts.at(i*3));
        cur_point[0] -= boxCenter[0];
        cur_point[1] -= boxCenter[1];
        cur_point[2] -= boxCenter[2];
        // We have box center and vertex in object coords 
        // --> take subtraction to find vertex from box center.
        dot_prod = Float3::dot(&cur_point, &axes[axes_order[1]]) / 
                   boxDimension[axes_order[1]];
        if (dot_prod >= thumbTestPoints_[j*2] && 
            dot_prod <= thumbTestPoints_[j*2+1]) {
          pts_thumb_.pushBack(*pts.at(i*3));
          pts_thumb_.pushBack(*pts.at(i*3+1));
          pts_thumb_.pushBack(*pts.at(i*3+2));
        }
      }
      // Now do SVD on that point cloud of points within the region
      math::PCA3DPoints(axes_thumb_, &cov_thumb_, &com_thumb_, &eigVals_thumb_, 
                        pts_thumb_.at(0), 
                        static_cast<unsigned int>(pts_thumb_.size()/3));
      float max_eigVal = eigVals_thumb_[0];
      int i_axis = 0;
      if (eigVals_thumb_[1] > max_eigVal) {
        max_eigVal = eigVals_thumb_[1];
        i_axis = 1;
      }
      if (eigVals_thumb_[2] > max_eigVal) {
        max_eigVal = eigVals_thumb_[2];
        i_axis = 2;
      }
      
      dot_prod = Float3::dot(&axes_thumb_[i_axis], &axes[axes_order[0]]);
      angle[j] = fabsf(acosf(dot_prod) - M_PI_2);
    }
    if (angle[0] < angle[1]) {
      approx_thumb_angle = M_PI_2 - angle[0];
    } else {
      approx_thumb_angle = M_PI_2 - angle[1];
    }
  }
  
  void HandStatistics::approxHandUVDimensions() {
    convertPointsToUVD();  // Just in case it hasn't been called (there is a dirty flag)    
    // Already have lock
    // Project longest axis into UV space using orthogonal projection
    Float3 axis_y(axes[axes_order[0]][0], axes[axes_order[0]][1], 0.0f);
    axis_y.normalize();
    static Float3 axis_z(0,0,1);
    Float3 axis_x;
    Float3::cross(&axis_x, &axis_y, &axis_z);
    axis_x.normalize();
    
    if (hand_ == HandType::HandRHand) {
    printf("R Hand axis_y:\n");
    axis_y.print();
    printf("R Hand axis_x:\n");
    axis_x.print();
    }
    
#if defined(DEBUG) || defined(_DEBUG)
    if (fabsf(axis_x[2]) > EPSILON || fabsf(axis_y[2]) > EPSILON) {
      throw std::runtime_error("approxHandUVDimensions new coords are not in XY plane.");
    }
#endif
    
    Float2 axis_uv_x(axis_x[0], axis_x[1]);
    Float2 axis_uv_y(axis_y[0], axis_y[1]);
    Float2 cur_point;
    uv_obb_max[0] = -std::numeric_limits<float>::infinity();
    uv_obb_max[1] = -std::numeric_limits<float>::infinity();
    uv_obb_min[0] = std::numeric_limits<float>::infinity();
    uv_obb_min[1] = std::numeric_limits<float>::infinity();    
    for (int i = 0; i < npts; i++) {
      cur_point.set(pts_UVD[3*i], pts_UVD[3*i+1]);
      float cur_y = Float2::dot(&axis_uv_y, &cur_point);
      float cur_x = Float2::dot(&axis_uv_x, &cur_point);
      if (cur_y > uv_obb_max[1]) {
        uv_obb_max[1] = cur_y;
      }
      if (cur_y < uv_obb_min[1]) {
        uv_obb_min[1] = cur_y;
      }      
      if (cur_x > uv_obb_max[0]) {
        uv_obb_max[0] = cur_x;
      }
      if (cur_x < uv_obb_min[0]) {
        uv_obb_min[0] = cur_x;
      }        
    }
    
    uv_obb_ratio = (uv_obb_max[1] - uv_obb_min[1]) / (uv_obb_max[0] - uv_obb_min[0]);
    if (hand_ == HandType::HandRHand) {
    printf("R Hand uv_obb_ratio = %f\n", uv_obb_ratio);
    }
  }
  
  // from http://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
  uint32_t nextPow2(uint32_t i) {
    i--;
    i |= i >> 1;
    i |= i >> 2;
    i |= i >> 4;
    i |= i >> 8;
    i |= i >> 16;
    i++;
    return i;
  }

  // From: http://graphics.stanford.edu/~seander/bithacks.html
  uint32_t log2(uint32_t v) {
    if (v == 1)
      return 0;
    const static uint32_t b[] = {0x2, 0xC, 0xF0, 0xFF00, 0xFFFF0000};
    const static uint32_t S[] = {1, 2, 4, 8, 16};
    int i;

    register uint32_t r = 0; // result of log2(v) will go here
    for (i = 4; i >= 0; i--) {
      if (v & b[i]) {
        v >>= S[i];
        r |= S[i];
      } 
    }
    return r;
  }

// #define VISUALIZE_POINTS

  inline int mod(int x, int m) {  // Handle negative modulus
    int r = x%m;
    return r<0 ? r+m : r;
  }

  void HandStatistics::testForOpenFingers() {
#ifdef _DEBUG
    if (!static_data_initialized_) {
      hand_stats_lock_.unlock();
      throw std::runtime_error("ERROR: HandStatistics: Static data is not initialized");
    }
#endif

    // ALREADY HAVE LOCK

    // Figure out which end of the longest axis the finger point is:
    // ie subtract off the center then project onto the longest axis
    float finger_proj = (highpt[0]-com[0])*axes[axes_order[0]][0] + 
                        (highpt[1]-com[1])*axes[axes_order[0]][1] +
                        (highpt[2]-com[2])*axes[axes_order[0]][2];
    float dot_axis[3];
    if (finger_proj > 0) {
      dot_axis[0] = axes[axes_order[0]][0];
      dot_axis[1] = axes[axes_order[0]][1];
      dot_axis[2] = axes[axes_order[0]][2];
    } else {
      dot_axis[0] = -axes[axes_order[0]][0];
      dot_axis[1] = -axes[axes_order[0]][1];
      dot_axis[2] = -axes[axes_order[0]][2];
    }

    static uint32_t num_boxes = dim_hand_pixels*dim_hand_pixels;
    memset(spherical_pixels_, false, sizeof(spherical_pixels_[0])*num_boxes);
    memset(spherical_pixels_rad_, 0, sizeof(spherical_pixels_rad_[0])*num_boxes);

#ifdef VISUALIZE_POINTS  // NOTE: THIS IS SLOW!!
    data_str::Vector<float> pts_tmp;
    pts_tmp.capacity(this->pts.capacity());
#endif
   
    Float3 cur_point;
    float dot_prod;
    Float3 spherical_axes[3];
    spherical_axes[0].set(dot_axis);
    //spherical_axes[1].set(&axes[axes_order[1]]);
    spherical_axes[1].set(1,0,0);
    spherical_axes[2].cross(&spherical_axes[0], &spherical_axes[1]);
    spherical_axes[2].normalize();
    // cross again to get orthonormal basis
    spherical_axes[1].cross(&spherical_axes[2], &spherical_axes[0]);
    spherical_axes[1].normalize();

    // Go through each vertex in the hand.
    // 1. If the point is within the front half of the half-box (cutting the
    //    longest axis in half).
    // 2. Project points within some shelled hemisphere onto the inner radius
    //    of the hemisphere.
    for (int i = 0; i < npts; i ++) {
      cur_point.set(pts.at(i*3));
      Float3::sub(&cur_point, &cur_point, &com);   
      float cur_rad = cur_point.length();
      dot_prod = cur_point[0]*dot_axis[0] + 
                 cur_point[1]*dot_axis[1] + 
                 cur_point[2]*dot_axis[2];

      if (cur_rad > finger_inner_rad_ && 
          (dot_prod / cur_rad) > OPEN_FINGER_DOT_PROD_CUTOFF) {
#ifdef VISUALIZE_POINTS
        pts_tmp.pushBack(*pts.at(i*3));
        pts_tmp.pushBack(*pts.at(i*3+1));
        pts_tmp.pushBack(*pts.at(i*3+2));
#endif

        // Point is within shell, project it's spherical coords onto the 2D 
        // texture.  A good overview of some possibilities are here:
        // http://kartoweb.itc.nl/geometrics/Map%20projections/body.htm

        // Stereographic projection
        // http://en.wikipedia.org/wiki/Stereographic_projection
        Float3 xyz_obb;
        xyz_obb[0] = Float3::dot(&spherical_axes[1], &cur_point);
        xyz_obb[1] = Float3::dot(&spherical_axes[2], &cur_point);
        xyz_obb[2] = -Float3::dot(&spherical_axes[0], &cur_point);
        cur_rad = xyz_obb.length();
        xyz_obb.scale(1.0f / cur_rad);  // normalize
        Float2 uv_obb;
        stereographicXYZ2UV(&uv_obb, &xyz_obb);
        
        // Now we warp the UV so that the pixel density is uniform
        uv_obb[0] = (2.0f / rescale_factor_) * atanf(uv_obb[0]); // after mapping -1 -> +1
        uv_obb[1] = (2.0f / rescale_factor_) * atanf(uv_obb[1]);
        
        uv_obb[0] = (uv_obb[0] + 1.0f) * u_quantization_;
        uv_obb[1] = (uv_obb[1] + 1.0f) * v_quantization_;
        int u = static_cast<int>(floor(uv_obb[0]));
        int v = static_cast<int>(floor(uv_obb[1]));

        uint32_t cur_index = u + v*dim_hand_pixels;
#ifdef _DEBUG
        if (u >= dim_hand_pixels || u < 0 || v >= dim_hand_pixels || v < 0) {
           throw std::runtime_error("Error: phi and theta are out of bounds!");
        }
#endif
        uint16_t cur_val = static_cast<uint16_t>(floorf(4095.0f*(cur_rad - finger_inner_rad_) / 
                                                        (finger_outer_rad_ - finger_inner_rad_)));
        cur_val = cur_val > 4095 ? 4095 : cur_val;
        if (spherical_pixels_[cur_index] < cur_val) {
          spherical_pixels_[cur_index] = cur_val;
        }
        if (spherical_pixels_rad_[cur_index] < cur_rad) {
          spherical_pixels_rad_[cur_index] = cur_rad;
        }
        
        if (ADJACENT_UV_FILL_RADIUS_N > 0 || ADJACENT_UV_FILL_RADIUS_P > 0) {
          // Now to ensure connectivity, fill in a few adjacent pixels
          int v_start =  max(v - ADJACENT_UV_FILL_RADIUS_N, 0);
          int v_finish = min(v + ADJACENT_UV_FILL_RADIUS_P, dim_hand_pixels);
          int u_start =  max(u - ADJACENT_UV_FILL_RADIUS_N, 0);
          int u_finish = min(u + ADJACENT_UV_FILL_RADIUS_P, dim_hand_pixels);
          
          for (int cur_v = v_start; cur_v <= v_finish; cur_v++) {
            for (int cur_u = u_start; cur_u <= u_finish; cur_u++) {
              cur_index = cur_u + cur_v * dim_hand_pixels;
#ifdef _DEBUG
              if (cur_index > num_boxes) {
                hand_stats_lock_.unlock();
                throw std::runtime_error("ERROR: U and V are outside of the bounds");
              }
#endif
              if (spherical_pixels_[cur_index] < cur_val) {
                spherical_pixels_[cur_index] = cur_val;
              }
              if (spherical_pixels_rad_[cur_index] < cur_rad) {
                spherical_pixels_rad_[cur_index] = cur_rad;
              }
            }
          }
        }
      }
    }

    // Now filter it

#ifdef VISUALIZE_POINTS
    this->pts = pts_tmp;
    npts = pts_tmp.size() / 3;
#endif

    // Blur the spherical map to reduce noise
    blur_effect_->blurX(spherical_pixels_, finger_blur_rad_, 
                        dim_hand_pixels, dim_hand_pixels);
    blur_effect_->blurY(spherical_pixels_, finger_blur_rad_, 
                        dim_hand_pixels, dim_hand_pixels);

    // Now perform 2D connected components (flood fill on the spherical_pixels_
    // texture)
    fingers.resize(0);
    fingers_pixel_sizes.resize(0);
    num_fingers = 0;
    Float3 finger_UVR;
   
    memset(pixel_on_queue_, 0, num_boxes * sizeof(pixel_on_queue_[0]));
    queue_head_ = 0;
    queue_tail_ = 0; // When queue_head_ == queue_tail_ the queue is empty
    for (int v = 0; v < dim_hand_pixels; v++) {
      for (int u = 0; u < dim_hand_pixels; u++) {
        int cur_index = u + v*dim_hand_pixels;
        if ((spherical_pixels_[cur_index] > finger_threshold_) && !pixel_on_queue_[cur_index]) {
          // We haven't already visited this pixel perform a floodfill 
          // starting from this pixel --> Potentially a new blob

          // Add the current pixel to the end of the queue
          pixel_queue_[queue_tail_] = cur_index;
          pixel_on_queue_[cur_index] = true;
          finger_UVR.set(u, v, spherical_pixels_rad_[cur_index]);
          queue_tail_++;
          uint32_t n_blob_pts = 0;

          while (queue_head_ != queue_tail_) {
            // Take the pixel off the head
            int cur_pixel = pixel_queue_[queue_head_];
            int cur_u = cur_pixel % dim_hand_pixels;
            int cur_v = cur_pixel / dim_hand_pixels;
            queue_head_++;
            n_blob_pts++;

            // Process the 8 surrounding neighbours
            processSphericalNieghbour(cur_u-1, cur_v-1, &finger_UVR);
            processSphericalNieghbour(cur_u-1, cur_v, &finger_UVR);
            processSphericalNieghbour(cur_u-1, cur_v+1, &finger_UVR);
            processSphericalNieghbour(cur_u, cur_v-1, &finger_UVR);
            processSphericalNieghbour(cur_u, cur_v+1, &finger_UVR);
            processSphericalNieghbour(cur_u+1, cur_v-1, &finger_UVR);
            processSphericalNieghbour(cur_u+1, cur_v, &finger_UVR);
            processSphericalNieghbour(cur_u+1, cur_v+1, &finger_UVR);
          }
          // Finished processing the entire blob
          if (n_blob_pts >= MIN_PTS_PER_FINGER_BLOB) {
            Float3 xyz_obb;
            float finger_XYZ[3];
            Float2 uv_obb(finger_UVR[0], finger_UVR[1]);
            uv_obb[0] = uv_obb[0] / u_quantization_ - 1.0f;
            uv_obb[1] = uv_obb[1] / v_quantization_ - 1.0f;  // -1 --> +1 warped
            
            uv_obb[0] = tanf(uv_obb[0] * (rescale_factor_ / 2.0f));
            uv_obb[1] = tanf(uv_obb[1] * (rescale_factor_ / 2.0f));  // UV stereographic coords
            
            stereographicUV2XYZ(&xyz_obb, &uv_obb);
            xyz_obb.scale(finger_UVR[2]);  // Multiply by the radius
            xyz_obb[2] *= -1.0f;
            
            // Now finger is in the spherical_axes space, so we need to do
            // the reverse translation
            finger_XYZ[0] = Float3Dot(xyz_obb.m, spherical_axes[1][0], 
                                      spherical_axes[2][0], spherical_axes[0][0]);
            finger_XYZ[1] = Float3Dot(xyz_obb.m, spherical_axes[1][1], 
                                      spherical_axes[2][1], spherical_axes[0][1]);
            finger_XYZ[2] = Float3Dot(xyz_obb.m, spherical_axes[1][2], 
                                      spherical_axes[2][2], spherical_axes[0][2]);
            
            // Finally, add back the com
            fingers.pushBack(finger_XYZ[0] + com[0]);
            fingers.pushBack(finger_XYZ[1] + com[1]);
            fingers.pushBack(finger_XYZ[2] + com[2]);
            fingers_pixel_sizes.pushBack(n_blob_pts);
            num_fingers++;
          }
        }
      }
    }
    // Order the fingers, left to right (along x axis), using insertion sort
    for (int i = 3; i < static_cast<int>(fingers.size()); i += 3) {
      int j = i;
      while (j > 2) {
        if (fingers[j] < fingers[j-3]) {
          // swap them
          float val = fingers[j-3];
          fingers[j-3] = fingers[j];
          fingers[j] = val;
          val = fingers[j-2];
          fingers[j-2] = fingers[j+1];
          fingers[j+1] = val;
          val = fingers[j-1];
          fingers[j-1] = fingers[j+2];
          fingers[j+2] = val;
          j -= 3;
        } else {
          break;
        }
      }
    }
  }

  void HandStatistics::processSphericalNieghbour(int u, int v, Float3* finger_UVR) {
    if (u < dim_hand_pixels && u >=0) {
      int cur_v = mod(v, dim_hand_pixels);
      int cur_index = u + cur_v * dim_hand_pixels;
      if ((!pixel_on_queue_[cur_index]) && (spherical_pixels_[cur_index] > finger_threshold_)) {
        // Add the pixel to the back of the queue
        pixel_queue_[queue_tail_] = cur_index;
        pixel_on_queue_[cur_index] = true;
        if (finger_UVR->m[2] < spherical_pixels_rad_[cur_index]) {
          finger_UVR->set(u, cur_v, spherical_pixels_rad_[cur_index]);
        }
        queue_tail_++;
      }
    }
  }

  void HandStatistics::calHandImage(XnUInt8* dst, uint32_t dst_dim) {
#ifdef _DEBUG
    if (dst_dim != dim_hand_pixels) {
      throw std::runtime_error("calHandImage() - Error, dst_dim != dim_hand_pixels");
    }
#endif
    hand_stats_lock_.lock();
    if (npts < min_npts) {
      for (uint32_t i = 0; i < dst_dim*dst_dim*3; i ++) {
        dst[i] = 255;
      }
    } else {
      for (uint32_t i = 0; i < dst_dim*dst_dim; i ++) {
        uint16_t cur_val_raw = spherical_pixels_[i];
        XnUInt8 cur_val = ((cur_val_raw>>4) > 255) ? 255 : static_cast<XnUInt8>((cur_val_raw>>4));
        if (cur_val_raw > finger_threshold_) {
          cur_val = 255 - cur_val;  // Inverse mapping
          dst[i*3] = cur_val;
          dst[i*3+1] = 0;
          dst[i*3+2] = 0;
        } else {
          cur_val = 255 - cur_val;  // Inverse mapping
          dst[i*3] = cur_val;
          dst[i*3+1] = cur_val;
          dst[i*3+2] = cur_val;
        }
      }
    }
    hand_stats_lock_.unlock();
  }
  
  void HandStatistics::resetHand() {
    hand_stats_lock_.lock();
    npts = 0;
    pts.resize(0);
    pts_UVD.resize(0);
    ptsOBB.resize(0);
    axes[0].set(1.0f, 0.0f, 0.0f);
    axes[1].set(0.0f, 1.0f, 0.0f);
    axes[2].set(0.0f, 0.0f, 1.0f);
    eigVals.set(1.0f, 1.0f, 1.0f);
    pts_thumb_.resize(0);
    hand_stats_lock_.unlock();
  }
  
  static const int img_width = 640;
  static const int img_height = 480;
  bool HandStatistics::saveHandData(const std::string& filename, 
                                    bool save_lhand, bool save_rhand) {
    bool save_data = false;
    kinect::KinectInterface* kinect = App::getKinect();
    kinect::HandStatistics* lHand = kinect->lHand_;
    kinect::HandStatistics* rHand = kinect->rHand_;
    kinect->lockData();
    if (save_lhand) {
      lHand->hand_stats_lock_.lock();
      if (lHand->npts >= min_npts) {
        save_data = true;
        lHand->convertPointsToUVD();
      }
    }
    if (save_rhand) {
      rHand->hand_stats_lock_.lock();
      if (rHand->npts >= min_npts) {
        save_data = true;
        rHand->convertPointsToUVD();
      }      
    }
    
    if (save_data == false) {
      if (save_lhand) {
        lHand->hand_stats_lock_.unlock();
      }
      if (save_rhand) {
        rHand->hand_stats_lock_.unlock();    
      }
      kinect->unlockData();
      return false;
    }
    
    if (pc_image == NULL) {
      pc_image = new float[img_width*img_height*3];
    }
    if (rgb_image == NULL) {
      rgb_image = new unsigned char[img_width*img_height*3];
    }    
    if (mask_image == NULL) {
      mask_image = new unsigned char[img_width*img_height];
    }
    if (depth_image == NULL) {
      depth_image = new unsigned short[img_width*img_height];
    }
    
    std::fill_n(pc_image, img_width*img_height*3, 
                std::numeric_limits<float>::quiet_NaN());
    std::fill_n(mask_image, img_width*img_height, 
                static_cast<char>(0));    
    
    // We need the point cloud data in UV space (so it alignes with RGB data)
    if (save_lhand) {
      lHand->convertPointsToUVD();
    }
    if (save_rhand) {
      rHand->convertPointsToUVD();
    }
    
    // Collect the left hand points
    if (save_lhand) {
      if (lHand->npts >= min_npts) {
        for (int i = 0; i < lHand->npts; i++) {
          int u = static_cast<int>(round(*lHand->pts_UVD.at(i*3)));
          int v = static_cast<int>(round(*lHand->pts_UVD.at(i*3 + 1)));
          int cur_index_output_array = (v*img_width + u);

          pc_image[cur_index_output_array*3] = *lHand->pts.at(i * 3);
          pc_image[cur_index_output_array*3+1] = *lHand->pts.at(i * 3 + 1);
          pc_image[cur_index_output_array*3+2] = *lHand->pts.at(i * 3 + 2);   
          mask_image[cur_index_output_array] |= 1;
        }
      }
    }
    
    // Collect the right hand points
    if (save_rhand) {
      if (rHand->npts >= min_npts) {
        for (int i = 0; i < rHand->npts; i++) {
          int u = static_cast<int>(round(*rHand->pts_UVD.at(i*3)));
          int v = static_cast<int>(round(*rHand->pts_UVD.at(i*3 + 1)));
          int cur_index_output_array = (v*img_width + u);

          pc_image[cur_index_output_array*3] = *rHand->pts.at(i * 3);
          pc_image[cur_index_output_array*3+1] = *rHand->pts.at(i * 3 + 1);
          pc_image[cur_index_output_array*3+2] = *rHand->pts.at(i * 3 + 2);   
          mask_image[cur_index_output_array] |= 2;
        }    
      }
    }
    
    // We're done with the hand specific data
    if (save_lhand) {
      lHand->hand_stats_lock_.unlock();
    }
    if (save_rhand) {
      rHand->hand_stats_lock_.unlock();
    }
    
    // Now copy over the rgb image
    XnRGB24Pixel* rgb_raw = kinect->getRGBImageUnsafe();
    for (int i = 0; i < img_width*img_height; i++) {
      rgb_image[i*3] = rgb_raw[i].nRed;
      rgb_image[i*3+1] = rgb_raw[i].nGreen;
      rgb_image[i*3+2] = rgb_raw[i].nBlue;
    }

    // Now copy over the depth image
    XnDepthPixel* p_depth = kinect->getDepthRawUnsafe();
    for (int i = 0; i < img_width*img_height; i++) {
      depth_image[i] = p_depth[i];
    }
    
    kinect->unlockData();
    
    // Now save both data arrays to file
    
#ifdef _WIN32
    std::string full_filename = filename;
#endif
#ifdef __APPLE__
    std::string full_path = file_io::GetHomePath() + std::string("Desktop/hand_depth_data");
    std::string full_filename = full_path + filename;
#endif
    std::ofstream file(full_filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + 
                               full_filename);
    }
    file.write(reinterpret_cast<const char*>(pc_image), 
               img_width*img_height*3 * sizeof(pc_image[0]));
    file.flush();
    file.write(reinterpret_cast<const char*>(rgb_image), 
               img_width*img_height*3 * sizeof(rgb_image[0]));    
    file.flush();
    file.write(reinterpret_cast<const char*>(mask_image), 
               img_width*img_height * sizeof(mask_image[0]));        
    file.flush();
    file.write(reinterpret_cast<const char*>(depth_image), 
               img_width*img_height * sizeof(depth_image[0]));        
    file.flush();
    file.close();
    
    return true;
  }

  // Format: File contains:
  // 1. Depth image, with MSB as a flag for the user pixels
  // 2. RGB image
  // The image is then compressed with miniz and exported to file.
  XnUserID aUsers[8];
  XnUInt16 nUsers = 8;  
  bool HandStatistics::saveHandDataForDecisionTree(const std::string& filename) {
    kinect::KinectInterface* kinect = App::getKinect();
    kinect->context_lock_.lock();
    kinect->lockData();
    
    uint16_t dummy_16;
    uint8_t dummy_8;
#if defined(WIN32) || defined(_WIN32)
    static_cast<void>(dummy_8);
    static_cast<void>(dummy_16);
#endif
    int data_size = img_width * img_height * sizeof(dummy_16) + 
                    img_width * img_height * 3 * sizeof(dummy_8);

    if (image_for_dt_ == NULL) {
      image_for_dt_ = (uint16_t*)malloc(data_size);
    }

    if (kinect->depth_generator_.GetDeviceMaxDepth() >= 0x8000) {
      throw std::runtime_error(std::string("warning max depth is less than 0x8000, ") + 
                               std::string("cannot mask the user pixels"));
    }
    
    if (!kinect->tracking_skeleton_) {
      printf("Cannot record data for decision tree unless skeleton tracking ");
      printf("is turned on\n");
      kinect->unlockData();
      kinect->context_lock_.unlock();
      return false;
    }
    
    if (kinect->num_users_tracked_ <= 0) {
      printf("Cannot record data for decision tree.  No users tracked!\n");
      kinect->unlockData();
      kinect->context_lock_.unlock();
      return false;
    }
    
    xn::SceneMetaData sceneMD;
    kinect->user_generator_.GetUserPixels(0, sceneMD);
    const XnLabel* pLabels = sceneMD.Data();

    // Now copy over the depth image and also flag the user pixels:
    XnDepthPixel* p_depth = kinect->getDepthRawUnsafe();
    for (int i = 0; i < img_width*img_height; i++) {
      image_for_dt_[i] = p_depth[i];
      bool user_pixel = pLabels[i] == 1;
      if (user_pixel) {
        image_for_dt_[i] |= 0x8000;  // Mark the second MSB
      }
    }
    
    // Now copy over the rgb image
    XnRGB24Pixel* rgb_raw = kinect->rgb;
    uint8_t* image_for_dt_rgb_ = reinterpret_cast<uint8_t*>(&image_for_dt_[img_width*img_height]);
    for (int i = 0; i < img_width*img_height; i++) {
      image_for_dt_rgb_[i*3] = static_cast<uint8_t>(rgb_raw[i].nRed);
      image_for_dt_rgb_[i*3+1] = static_cast<uint8_t>(rgb_raw[i].nGreen);
      image_for_dt_rgb_[i*3+2] = static_cast<uint8_t>(rgb_raw[i].nBlue);
    }
    
    kinect->unlockData();
    kinect->context_lock_.unlock();

    return saveCompressedArrayToFile(filename, data_size);
  }
  
  bool HandStatistics::saveHandDataWithHandPoints(const std::string& filename) {
    kinect::KinectInterface* kinect = App::getKinect();
    kinect::HandStatistics* lHand = kinect->lHand_;
    kinect::HandStatistics* rHand = kinect->rHand_;
    kinect->context_lock_.lock();
    kinect->lockData();
    
    bool save_data = false;
    lHand->hand_stats_lock_.lock();
    if (lHand->npts >= min_npts) {
      save_data = true;
      lHand->convertPointsToUVD();
    }
    rHand->hand_stats_lock_.lock();
    if (rHand->npts >= min_npts) {
      save_data = true;
      rHand->convertPointsToUVD();
    }
    
    // No hand data at all to save!
    if (!save_data) {
      lHand->hand_stats_lock_.unlock();
      rHand->hand_stats_lock_.unlock();
      kinect->context_lock_.unlock();
      kinect->unlockData();
      return false;
    }
  
    uint16_t dummy_16;
    uint8_t dummy_8;
#if defined(WIN32) || defined(_WIN32)
    static_cast<void>(dummy_8);
    static_cast<void>(dummy_16);
#endif
    int data_size = img_width * img_height * sizeof(dummy_16) +
      img_width * img_height * 3 * sizeof(dummy_8);  // Depth + RGB
    
    if (image_for_dt_ == NULL) {
      image_for_dt_ = (uint16_t*)malloc(data_size);
    }
    
    if (kinect->depth_generator_.GetDeviceMaxDepth() >= 0x8000) {
      throw std::runtime_error(std::string("warning max depth is less than 0x8000, ") +
                               std::string("cannot mask the user pixels"));
    }
    
    // Now copy over the depth image
    XnDepthPixel* p_depth = kinect->getDepthRawUnsafe();
    for (int i = 0; i < img_width*img_height; i++) {
      image_for_dt_[i] = p_depth[i];
    }
    
    // Now flag the hand pixels
    if (lHand->npts >= min_npts) {
      for (int i = 0; i < lHand->npts; i++) {
        int u = static_cast<int>(round(*lHand->pts_UVD.at(i*3)));
        int v = static_cast<int>(round(*lHand->pts_UVD.at(i*3 + 1)));
        int cur_index_output_array = (v*img_width + u);
        image_for_dt_[cur_index_output_array] |= 0x8000;  // Mark the second MSB
      }
    }
    if (rHand->npts >= min_npts) {
      for (int i = 0; i < rHand->npts; i++) {
        int u = static_cast<int>(round(*rHand->pts_UVD.at(i*3)));
        int v = static_cast<int>(round(*rHand->pts_UVD.at(i*3 + 1)));
        int cur_index_output_array = (v*img_width + u);
        image_for_dt_[cur_index_output_array] |= 0x8000;  // Mark the second MSB
      }
    }
    
    // Now copy over the rgb image
    XnRGB24Pixel* rgb_raw = kinect->rgb;
    uint8_t* image_for_dt_rgb_ = reinterpret_cast<uint8_t*>(&image_for_dt_[img_width*img_height]);
    for (int i = 0; i < img_width*img_height; i++) {
      image_for_dt_rgb_[i*3] = static_cast<uint8_t>(rgb_raw[i].nRed);
      image_for_dt_rgb_[i*3+1] = static_cast<uint8_t>(rgb_raw[i].nGreen);
      image_for_dt_rgb_[i*3+2] = static_cast<uint8_t>(rgb_raw[i].nBlue);
    }
    
    lHand->hand_stats_lock_.unlock();
    rHand->hand_stats_lock_.unlock();
    kinect->unlockData();
    kinect->context_lock_.unlock();

    return saveCompressedArrayToFile(filename, data_size);
  }
  
  bool HandStatistics::saveCompressedArrayToFile(const string& filename,
                                                 uint32_t data_size) {
    // ************************************************
    // Compress the array using fastlz
    // ************************************************
    // Also tried miniz and lzo libraries.  This one was the fastest and
    // gave OK results.
    if (image_for_dt_compressed_ == NULL) {
      // Note compressed image may be 5% larger if no compression is possible!
      image_for_dt_compressed_ = (uint16_t*)malloc(data_size * 2);
    }
    static const int compression_level = 1;  // 1 fast, 2 better compression
    int compressed_length = fastlz_compress_level(compression_level,
      reinterpret_cast<void*>(image_for_dt_), data_size,
      reinterpret_cast<void*>(image_for_dt_compressed_));
     
    // Now save the array to file
#ifdef _WIN32
    std::string full_filename = filename;
#endif
#ifdef __APPLE__
    std::string full_path = file_io::GetHomePath() +
      std::string("Desktop/KinectHands/hand_depth_data/");
    struct stat st;
    if (stat(full_path.c_str(), &st) != 0) {
      if (mkdir(full_path.c_str(), S_IRWXU|S_IRWXG) != 0) {
        printf("Error creating directory %s: %s\n", full_path.c_str(),
               strerror(errno));
        return false;
      } else {
        printf("%s created\n", full_path.c_str());
      }
    }
    std::string full_filename = full_path + filename;
#endif
    
    // Save the file compressed
    const char* arr_to_save = reinterpret_cast<char*>(image_for_dt_compressed_);
    int length_to_save = compressed_length;
    
    std::ofstream file(full_filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + filename);
    }
    file.write(reinterpret_cast<const char*>(arr_to_save), length_to_save);
    file.flush();
    file.close();
    return true;
  }
  
  void HandStatistics::updateHand() {
    
    if (kinect_->calculate_hand_points_) {
      findHandPoints();
    }
    if (kinect_->calculate_hand_statistics_) {
      calculateStatistics();
    }
    
#ifdef MULTI_THREADED
    // Tell the kinect class that we're done
    kinect_->hand_finished_lock_.lock();
    kinect_->num_hands_finished_++;
    kinect_->hand_finished_cv_.notify_all();
    kinect_->hand_finished_lock_.unlock();
#endif
  }

  const Float3 blue(0,0,1);
  const Float3 red(1,0,0);
  void HandStatistics::drawHandStatistics(bool draw_pts, bool draw_obb, bool draw_finger) {
    hand_stats_lock_.lock();
    if (npts > min_npts) {
      convertPointsToUVD();
      if (draw_pts) {
        if (hand_ == HandLHand) {
          drawPointsUVD(&blue);
        } else {
          drawPointsUVD(&red);
        }
      }
      if (draw_obb) {
        drawOBBPointsUVD();
      }
      if (draw_finger) {
        if (hand_ == HandLHand) {
          drawFingerPointUVD(&red);
        } else {
          drawFingerPointUVD(&blue);
        }
      }
    }
    hand_stats_lock_.unlock();
  }

}  // namespace kinect
