//
//  vec3.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#ifndef MATH_VEC3_HEADER
#define MATH_VEC3_HEADER

#include <float.h>
#include <math.h>
#include "alignment/data_align.h"

#ifndef EPSILON
#define EPSILON (2 * FLT_EPSILON)  // 2 times machine precision for float
#endif

namespace math {
  template <class T>
  class Mat3x3;

  // Note Vectors are all column vectors:
  //          | m(0,0) |   | 0 |
  //   Vec3 = | m(1,0) | = | 1 |
  //          | m(2,0) |   | 2 |
  template <class T>
  class DATA_ALIGN(ALIGNMENT) Vec3 {
  public:
    Vec3();
    explicit Vec3(T* data);
    explicit Vec3(Vec3* data);
    Vec3(T x, T y, T z);

    // Getter methods
    void print();  // Print to std::cout
    T operator[](int i) const { 
      return m[i]; 
    } 
    T & operator[](int i) { 
      return m[i]; 
    }  

    // Setter methods
    void zeros();
    void ones();
    void set(T x, T y, T z);
    void set(T* data);
    inline void set(Vec3* data) { this->set(data->m); }

    // Math operations
    inline void normalize() { normalize(this); }
    inline T length() { return length(this); }
    inline void scale(T s) { scale(this, s); }
    inline T dot(Vec3* a) { return dot(this, a); }
    inline void cross(Vec3* a, Vec3* b) { cross(this, a, b); }
    inline void sub(Vec3* a, Vec3* b) { sub(this, a, b); }
    inline void add(Vec3* a, Vec3* b) { add(this, a, b); }
    inline void pairwiseMult(Vec3* a, Vec3* b) { pairwiseMult(this, a, b); }
    inline void mult(Mat3x3<T>* a, Vec3* b) { mult(this, a, b); }
    inline void trans_mult(Vec3* a, Mat3x3<T>* b) { trans_mult(this, a, b); }
    inline void trans_mult(Mat3x3<T>* a, Vec3* b) { trans_mult(this, a, b); }
    void accum(T* a);  // this += accum
    inline bool equal(Vec3* a) { return equal(this, a); }

    // Static Math operations
    static void normalize(Vec3* a);
    static T length(Vec3* a);
    static void scale(Vec3* a, T s);
    static T dot(Vec3* a, Vec3* b);  // a.b
    static void cross(Vec3* ret, Vec3* a, Vec3* b);  // ret = axb
    static void sub(Vec3* ret, Vec3* a, Vec3* b);  // ret = a-b
    static void add(Vec3* ret, Vec3* a, Vec3* b);  // ret = a+b
    static void pairwiseMult(Vec3* ret, Vec3* a, Vec3* b);  // ret = a.*b
    static void mult(Vec3* ret, Mat3x3<T>* a, Vec3* b);  // ret = a*b
    static void trans_mult(Vec3* ret, Vec3* a, Mat3x3<T>* b);  // this = a^t*b
    static void trans_mult(Vec3* ret, Mat3x3<T>* a, Vec3* b);  // this = a^t*b
    static bool equal(Vec3* a, Vec3* b);

    T m[3];  // Not private --> Avoid some overhead with getter setter methods
    char pad[ALIGNMENT - ((3*sizeof(T)) % ALIGNMENT)];
  };  // end class Vec3

  // Vec3 Constructors
  template <class T>
  Vec3<T>::Vec3() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
  };
  template <class T>
  Vec3<T>::Vec3(T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
  };
  template <class T>
  Vec3<T>::Vec3(Vec3* data) {
    m[0] = data->m[0];
    m[1] = data->m[1];
    m[2] = data->m[2];
  };
  template <class T>
  Vec3<T>::Vec3(T x, T y, T z) {
    m[0] = x;
    m[1] = y;
    m[2] = z;
  };

  // Setter methods
  template <class T>
  void Vec3<T>::zeros() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
  };
  template <class T>
  void Vec3<T>::ones() {
    m[0] = (T)1;
    m[1] = (T)1;
    m[2] = (T)1;
  };
  template <class T>
  void Vec3<T>::set(T x, T y, T z) {
    m[0] = x;
    m[1] = y;
    m[2] = z;
  };
  template <class T>
  void Vec3<T>::set(T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
  };

  // Getter Methods
  template <class T>
  void Vec3<T>::print() {
    printf("| %+.4e |\n| %+.4e |\n| %+.4e |\n", m[0], m[1], m[2]);
  };

  // Math operations
  template <class T>
  void Vec3<T>::accum(T* a) {
    m[0] += a[0];
    m[1] += a[1];
    m[2] += a[2];
  };

  // Static Math operations
  template <class T>
  void Vec3<T>::normalize(Vec3* a) {
    T oneOverLength = (T)1.0 / sqrt(a->m[0] * a->m[0] + 
      a->m[1] * a->m[1] + 
      a->m[2] * a->m[2]);
    a->m[0] *= oneOverLength;
    a->m[1] *= oneOverLength;
    a->m[2] *= oneOverLength;
  };
  template <class T>
  T Vec3<T>::length(Vec3* a) {
    return sqrt(a->m[0] * a->m[0] + 
      a->m[1] * a->m[1] + 
      a->m[2] * a->m[2]);
  };
  template <class T>
  void Vec3<T>::scale(Vec3* a, T s) {
    a->m[0] *= s;
    a->m[1] *= s;
    a->m[2] *= s;
  };
  template <class T>
  T Vec3<T>::dot(Vec3* a, Vec3* b) {
    return a->m[0] * b->m[0] + 
      a->m[1] * b->m[1] +
      a->m[2] * b->m[2];
  };
  template <class T>
  void Vec3<T>::cross(Vec3* ret, Vec3* a, Vec3* b) {
    ret->m[0] = a->m[1] * b->m[2] - b->m[1] * a->m[2];
    ret->m[1] = a->m[2] * b->m[0] - b->m[2] * a->m[0];
    ret->m[2] = a->m[0] * b->m[1] - b->m[0] * a->m[1];
  };
  template <class T>
  void Vec3<T>::sub(Vec3* ret, Vec3* a, Vec3* b) {
    ret->m[0] = a->m[0] - b->m[0];
    ret->m[1] = a->m[1] - b->m[1];
    ret->m[2] = a->m[2] - b->m[2];
  }
  template <class T>
  void Vec3<T>::add(Vec3* ret, Vec3* a, Vec3* b) {
    ret->m[0] = a->m[0] + b->m[0];
    ret->m[1] = a->m[1] + b->m[1];
    ret->m[2] = a->m[2] + b->m[2];
  };
  template <class T>
  void Vec3<T>::pairwiseMult(Vec3* ret, Vec3* a, Vec3* b) {
    ret->m[0] = a->m[0] * b->m[0];
    ret->m[1] = a->m[1] * b->m[1];
    ret->m[2] = a->m[2] * b->m[2];
  };
  template <class T>
  void Vec3<T>::mult(Vec3* ret, Mat3x3<T>* a, Vec3* b) {
    ret->m[0] = a->m[0]*b->m[0] + a->m[1]*b->m[1] + a->m[2]*b->m[2];
    ret->m[1] = a->m[3]*b->m[0] + a->m[4]*b->m[1] + a->m[5]*b->m[2];
    ret->m[2] = a->m[6]*b->m[0] + a->m[7]*b->m[1] + a->m[8]*b->m[2];
  };
  template <class T>
  void Vec3<T>::trans_mult(Vec3* ret, Vec3* a, Mat3x3<T>* b) {
    ret->m[0] = a->m[0]*b->m[0] + a->m[1]*b->m[3] + a->m[2]*b->m[6];
    ret->m[1] = a->m[0]*b->m[1] + a->m[1]*b->m[4] + a->m[2]*b->m[7];
    ret->m[2] = a->m[0]*b->m[2] + a->m[1]*b->m[5] + a->m[2]*b->m[8];
  };
  template <class T>
  void Vec3<T>::trans_mult(Vec3* ret, Mat3x3<T>* a, Vec3* b) {
    ret->m[0] = a->m[0]*b->m[0] + a->m[3]*b->m[1] + a->m[6]*b->m[2];
    ret->m[1] = a->m[1]*b->m[0] + a->m[4]*b->m[1] + a->m[7]*b->m[2];
    ret->m[2] = a->m[2]*b->m[0] + a->m[5]*b->m[1] + a->m[8]*b->m[2];
  };
  template <class T>
  bool Vec3<T>::equal(Vec3* a, Vec3 *b) {
    for (int i = 0; i < 3; i ++)
      if (abs(a->m[i] - b->m[i]) > EPSILON)
        return false;
    return true;
  };

};  // end namespace math

#endif
