// File:      plane.h
// Author:    Jonathan Tompson
// e-mail:    jonathantompson@gmail.com

// A plane class

#ifndef MATH_PLANE_HEADER
#define MATH_PLANE_HEADER

#include "math/math_types.h"

namespace math {
  template <class T>
  class quaternion;

  template <class T>
  class Plane {
   public:
    Vec3<T> normal;
    T dist;  // distance below origin - the D from plane equasion Ax+By+Cz+D=0

    // Constructors
    Plane(Vec3<T>* normal, double dist);
    Plane();

    // Setters
    void set(Plane* p);

    // Operations
    void FlipPlane();
    // void Transform(Vec3<T>* position, quaternion<T>* orientation);
    static bool Coplanar(Plane* A, Plane* B);
  };

  template <class T>
  Plane<T>::Plane(Vec3<T>* normal, double dist) {
    this->normal.set(normal);
    this->dist = dist;
  };

  template <class T>
  Plane<T>::Plane() {
    this->normal.zeros();
    this->dist = 0;
  };

  template <class T>
  void Plane<T>::FlipPlane() {
    dist *= -1;
    normal.scale(-1);
  };

  template <class T>
  void Plane<T>::set(Plane* p) {
    this->normal.set(&p->normal);
    this->dist = p->dist;
  };

  template <class T>
  bool Plane<T>::Coplanar(Plane* A, Plane* B) {
    if ((A->normal[0] == B->normal[0]  && A->normal[1] == B->normal[1]  && 
      A->normal[2] == B->normal[2]  && A->dist == B->dist) ||
       (A->normal[0] == -B->normal[0] && A->normal[1] == -B->normal[1] && 
       A->normal[2] == -B->normal[2] && A->dist == -B->dist))
      return true;
    else
      return false;
  };
  
  /*
  template <class T>
  void plane::Transform(Vec3<T>* position, quaternion<T>* orientation) {
    //   Transforms the plane to the space defined by the 
    //   given position/orientation.
    static Vec3<T> newnormal;
    static Vec3<T> origin;

    newnormal = Inverse(orientation)*normal;
    origin = Inverse(orientation)*(-normal*dist - position);

    normal = newnormal;
    dist = -dot(newnormal, origin);
  }
  */

};  // end namespace math

#endif
