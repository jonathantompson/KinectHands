;//
//  contour.cpp
//  KinectHands
//
//  Created by Jonathan Tompson on 6/21/12.
//  Implements the winged edge data structure
//

#include "contour_simplification/contour.h"
#include "data_str/vector.h"
#include "contour_simplification/contour_simplification.h"

using data_str::Vector;
using math::Float3;
using std::runtime_error;
using std::string;

namespace contour_simplification {
  math::Float3 Contour::v;
  math::Float3 Contour::w;
  math::Float3 Contour::cur_norm;
  const Float3 Contour::contour_normal(0, 0, -1);


  Contour::Contour(const Float3* v1, uint32_t contour_index, uint32_t curr) {
    this->v1 = *v1;
    this->contour_index = contour_index;
    next = MAX_UINT32;
    prev = MAX_UINT32;
    this->curr = curr;
  }
  
  Contour::Contour() {
    next = MAX_UINT32;
    prev = MAX_UINT32;
    contour_index = MAX_UINT32;
    curr = MAX_UINT32;
  }

  Contour& Contour::operator= (const Contour& a) {
    if (this == &a) {
      return *this;
    }
    this->v1 = a.v1;
    this->next = a.next;  
    this->prev = a.prev;  
    this->cost = a.cost;  
    this->curr = a.curr;
    this->contour_index = a.contour_index;
    // Forget the heap index, length and angle (these are just temp variables)
    return *this;
  }
  
  void Contour::calcLengthAndAngle(Vector<Contour>* contours) {
    v.sub(&contours->at(next)->v1, &v1);  // Vector from this point to the next
    w.sub(&v1, &contours->at(prev)->v1);  // Vector from last point to this
    length = v.length();
#if defined(DEBUG) || defined(_DEBUG)
    if (Float3::equal(&contours->at(next)->v1, &v1)) {
      throw std::runtime_error("ERROR: adjacent contour vertices are equal!");
    }
#endif
    v.normalize();
    w.normalize();
    float dot = Float3::dot(&v, &w);
    dot = dot < -1 ? -1 : dot;
    dot = dot > 1 ? 1 : dot;
    angle = acosf(dot);
    // But we need the signed angle, so cross the two and compare to the contour
    // normal
    Float3::cross(&cur_norm, &v, &w);
    if (Float3::dot(&cur_norm, const_cast<Float3*>(&contour_normal)) < 0) {
      angle *= -1.0f;
    }
    // Angle is -pi --> pi
  }
  
  void Contour::calcCost(Vector<Contour>* contours, float contour_length) {
    float delta_angle = angle + contours->at(next)->angle;
    
    // Integrate dAngle / dLength over a constant radius
    cost = fabsf(delta_angle) * (length * length);
  }
  
  void Contour::invalidateContour() {
    next = MAX_UINT32;
    prev = MAX_UINT32;
    curr = MAX_UINT32;
    contour_index = MAX_UINT32;
  }
  
}  // namespace contour_simplification