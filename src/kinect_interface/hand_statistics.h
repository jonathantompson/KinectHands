//
//  hand_statistics.h
//
//  Created by Jonathan Tompson on 3/16/12.
//  Copyright (c) 2012 NYU. All rights reserved.
//

#ifndef HAND_STATISTICS_HEADER
#define HAND_STATISTICS_HEADER

#include <mutex>
#include "rendering/opengl_include.h"
#include "math/math_types.h"
#include "effects.h"
#include "data_str/vector.h"
#include "XnCppWrapper.h"
#include "rendering/effects.h"

namespace xn { class DepthGenerator; }
using math::Vec3;

#define KINECT_FPS 30.0

// Small radius with which to do a min search IN WORLD COORDS
#define SMALL_HAND_RADIUS 50.0f
// Minimum small hand radius in UV coordinates
#define SMALL_HAND_RADIUS_MIN_UV 10
// Big radius with which to do the flood fill
#define BIG_HAND_RADIUS_SKELETON 200.0f
#define BIG_HAND_RADIUS 150.0f
// Threshold of connected component sweep IN WORLD COORDS
#define BACKGROUND_THRESH 75.0f
#define BACKGROUND_THRESH_SQ (BACKGROUND_THRESH * BACKGROUND_THRESH)
// Radius for figner point orientation search (in pixels)
#define FINGER_SEARCH_RADIUS 5

// Uncomment the next line to find the finger using plain old highest y value
// #define FIND_FINGER_LONGEST_AXIS

#define THUMB_ANGLE_THRESH_RAD M_PI_4
#define OPEN_FINGER_DOT_PROD_CUTOFF -0.4f  // (enough to get thumb)
#define ADJACENT_UV_FILL_RADIUS_N 0
#define ADJACENT_UV_FILL_RADIUS_P 1
#define MIN_PTS_PER_FINGER_BLOB 25

#define N_PTS_FILL 16
#define FILL_COARSE_RADIUS 5000  // This value is divided by depth in mm!
#define FILL_FINE_RADIUS 1000 

namespace kinect {
  class KinectInterface;
  
  typedef enum {
    HandLHand,
    HandRHand,
  } HandType;
  
  class HandStatistics {
  public:
    friend class KinectInterface;
    
    data_str::Vector<float> pts;
    data_str::Vector<float> pts_UVD;   
    data_str::Vector<float> ptsOBB; 
    XnPoint3D* pts_all_world;
    XnPoint3D* pts_all_uvd;
    
    int npts;
    const static int min_npts = 50;
    math::Float3 eigVals;
    math::Float3 eigVals_old;
    math::Float3 axes[3];  // principle axes (for aligning coordinate system)
    int axes_order[3];  // axes[axes_order[0]] = largest axis (along wrist to hand)
    math::Float3 axes_old[3];  // previous frame's axes
    math::Float3x3 cov;     // Covarience matrix
    math::Float3 com;     // center of mass
    float boxDimension[3];  // Bounding box lengths (half lengths!)
    float boxCenter[3];  // Bounding box center in world coords
    float boxCenterUVD[3];  // Boudning box center in UVD coords
    float boxCenterOBB[3];  // Bounding box center in the bounding box's coords
    float highpt[3];
    GLfloat highpt_UV[3];
    float outmat[30];
    GLfloat boxCornerUV[8*2];
    data_str::Vector<float> fingers;
    data_str::Vector<uint32_t> fingers_pixel_sizes;
    data_str::Vector<float> fingers_uvd;
    uint32_t num_fingers;
    char num_fingers_str[4];    
    float approx_thumb_angle;
    const static uint32_t dim_hand_pixels = 256;
    static float* pc_image;
    static unsigned char* rgb_image;
    static unsigned char* mask_image;
    static unsigned short* depth_image;
    static uint16_t* image_for_dt_;
    static uint16_t* image_for_dt_compressed_;
    math::Float2 uv_obb_min;  // oriented bounding box in screen space
    math::Float2 uv_obb_max;  // oriented bounding box in screen space
    float uv_obb_ratio;  // Ratio of obb axis sizes
    
    HandStatistics(KinectInterface* kinect, HandType hand);
    ~HandStatistics();
    
    static void initHandStatistics();  // Must be called on startup
    void updateHand();

    void drawPoints(const math::Float3* cur_color);
    void drawPointsUVD(const math::Float3* cur_color);
    void drawOBB();  // Draw a wireframe OBB
    void drawSolidOBB();  // Draw a solid OBB
    void drawOBBPointsUVD();  // Just draw 8 box corners in UVD space
    int computeOutMatrix();
    static void setHandSearchRadius(float rad) { hand_search_radius_ = rad; hand_search_radius_sq_ = rad*rad; }
    void resetHand();
    static bool saveHandData(const std::string& filename, bool save_lhand, 
                             bool save_rhand);
    static bool saveHandDataForDecisionTree(const std::string& filename);
    static bool saveHandDataWithHandPoints(const std::string& filename);
    void drawHandStatistics(bool draw_pts, bool draw_obb, bool draw_finger);
    void calHandImage(XnUInt8* dst, uint32_t dst_dim);  
    void convertPointsToUVD();
    
    static inline float getFingerInnerRad() { return finger_inner_rad_; }
    static inline void setFingerInnerRad(float val) { finger_inner_rad_ = val; }
    static inline float getFingerOuterRad() { return finger_outer_rad_; }
    static inline void setFingerOuterRad(float val) { finger_outer_rad_ = val; }    
    static inline uint16_t getFingerThreshold() { return finger_threshold_; }
    static inline void setFingerThreshold(uint16_t val) { finger_threshold_ = val; }
    static inline int getFingerBlurRad() { return finger_blur_rad_; }
    static inline void setFingerBlurRad(int val) { finger_blur_rad_ = val; }         
    
  private:
    static bool static_data_initialized_;
    KinectInterface* kinect_;
    HandType hand_;
    static float tmpMat_[16];
    static const float floodFillKernel_[N_PTS_FILL][2];   
    uint16_t spherical_pixels_[dim_hand_pixels*dim_hand_pixels];  // (u,v) = (phi,theta)
    float spherical_pixels_rad_[dim_hand_pixels*dim_hand_pixels];
    static float u_quantization_;
    static float v_quantization_;
    static float rescale_factor_;
    static const float thumbTestPoints_[4];
    data_str::Vector<float> pts_thumb_;
    math::Float3x3 cov_thumb_;
    math::Float3 com_thumb_;
    math::Float3 eigVals_thumb_;  
    math::Float3 axes_thumb_[3];
    math::Float3 axesTmp_[3];
    math::Float3 eigValsTmp_;
    float tmp_pt_world_[3];
    float tmp_pt_uvd_[3];
    std::mutex hand_stats_lock_;
    // Pixel queue for doing connected components
    uint8_t* pixel_on_queue_;
    int* pixel_queue_;
    int queue_head_;
    int queue_tail_;
    static float hand_search_radius_;
    static float hand_search_radius_sq_;
    Effects<uint16_t>* blur_effect_;
    static float finger_inner_rad_;
    static float finger_outer_rad_;
    static uint16_t finger_threshold_;
    static int finger_blur_rad_;    
    bool dirty_uvd_data_;
    
    void processNeighbour(int* nieghbourPtUV, int curPtIndex,
                          math::Float3* tmp, XnPoint3D* ptHand);
    bool searchUVForMinHandPoint(XnPoint3D* ptHand_UVD,
                                 float* minPt, 
                                 int radius);
    bool searchUVForClosePoints(float* pt_UVD,
                                float* pt_world, 
                                int radius,
                                int* numPoints);
    void searchUVForHandPoints(math::Float3* ptHand_UVD, 
                               math::Float3* ptHand_WORLD, int radius);
    void searchUVForGlovePoints();    
    void orderAxes();
    void resolveAxisDoubleEquality(int a, int b, int c, int* ind, 
                                   bool* flipped);
    void resolveAxisTripleEquality(int a, int b, int c, int* ind, 
                                   bool* flipped);
    void calcBoundingBox();
    void drawSolidCube();
    void drawCube();
    void calcBoxCorner(float* boxCorner, float s0, float s1, float s2);
    static float OBBPointColors[8*3];
    
    void findHandPointsFloodFill(XnPoint3D* ptHand_world, 
                                 XnPoint3D* ptHand_UVD);
    void testForThumb();
    void testForOpenFingers();
    void approxHandUVDimensions();
    void chooseHandColor(const math::Float3* cur_color);
    inline void processSphericalNieghbour(int u, int v, math::Float3* finger_UVR);
    void calculateStatistics();  
    void drawFingerPointUVD(const math::Float3* cur_color);
    void findHandPoints(); 
    static bool saveCompressedArrayToFile(const std::string& filename,
                                          uint32_t data_size);
  };
  
};  // namespace kinect

#endif
