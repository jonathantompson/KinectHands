//
//  contour.h
//  KinectHands
//
//  Created by Jonathan Tompson on 10/24/12.
//

#ifndef MARCHING_SQUARES_CONTOUR_HEADER
#define MARCHING_SQUARES_CONTOUR_HEADER

#define COST_KERNEL_LENGTH 0.025f  // As a fraction of the total contour length

#include "math/math_types.h"

namespace data_str { template <typename T> class Vector; }

namespace marching_squares {
  class Contour {
  public:
    Contour(const math::Float2& v1, uint32_t contour_index, uint32_t curr);
    Contour();
    math::Float2 v1;
    uint32_t next;  
    uint32_t prev;
    uint32_t curr;
    float cost;
    uint32_t heap_index;
    float length;
    float angle;
    uint32_t contour_index;  // Which top level boundry we are part of

    void calcCost(data_str::Vector<Contour>* contours, float contour_length);
    void calcLengthAndAngle(data_str::Vector<Contour>* contours);
    Contour& operator= (const Contour& a);
    void invalidateContour();
    
  private:
    static math::Float2 v;
    static math::Float2 w;
    static math::Float3 v_vec3;
    static math::Float3 w_vec3;
    static math::Float3 cur_norm;
    static const math::Float3 contour_normal;
  };
};  // namespace marching_squares

#endif  // MARCHING_SQUARES_CONTOUR_HEADER
