//
//  contour.h
//  KinectHands
//
//  Created by Jonathan Tompson on 6/21/12.
//

#ifndef CONTOUR_SIMPLIFICATION_CONTOUR_HEADER
#define CONTOUR_SIMPLIFICATION_CONTOUR_HEADER

#define COST_KERNEL_LENGTH 0.025f  // As a fraction of the total contour length

#include "math/math_types.h"

namespace data_str { template <typename T> class Vector; }

namespace contour_simplification {
  class Contour {
  public:
    Contour(const math::Float3* v1, uint32_t contour_index, uint32_t curr);
    Contour();
    math::Float3 v1;
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
    static math::Float3 v;
    static math::Float3 w;
    static math::Float3 cur_norm;
    static const math::Float3 contour_normal;
  };
};  // namespace contour_simplification

#endif  // CONTOUR_SIMPLIFICATION_CONTOUR_HEADER
