//
//  vec2.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#ifndef MATH_VEC2_HEADER
#define MATH_VEC2_HEADER

#include <float.h>
#include "alignment/data_align.h"

#ifndef EPSILON
#define EPSILON (2 * FLT_EPSILON)  // 2 times machine precision for float
#endif

namespace math {
  template <class T>
  class Mat2x2;

  // Note Vectors are all column vectors (OpenGL uses column vectors):
  //          | m(0,0) |   | 0 |
  //   Vec2 = | m(1,0) | = | 1 |
  template <class T>
  class DATA_ALIGN(ALIGNMENT) Vec2 {
  public:
    Vec2();
    explicit Vec2(T* data);
    explicit Vec2(Vec2* data);
    Vec2(T x, T y);

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
    void set(T x, T y);
    void set(T* data);
    inline void set(Vec2* data) { this->set(data->m); }

    // Math operations
    inline void normalize() { normalize(this); }
    inline T length() { return length(this); }
    inline void scale(T s) { scale(this, s); }
    inline T dot(Vec2* a) { return dot(this, a); }
    inline void sub(Vec2* a, Vec2* b) { sub(this, a, b); }
    inline void add(Vec2* a, Vec2* b) { add(this, a, b); }
    inline void pairwiseMult(Vec2* a, Vec2* b) { pairwiseMult(this, a, b); }
    inline void mult(Mat2x2<T>* a, Vec2* b) { mult(this, a, b); }
    inline void accum(T* a);  // this += accum
    inline bool equal(Vec2* a) { return equal(this, a); }

    // Static Math operations
    static void normalize(Vec2* a);
    static T length(Vec2* a);
    static void scale(Vec2* a, T s);
    static T dot(Vec2* a, Vec2* b);  // a.b
    static void sub(Vec2* ret, Vec2* a, Vec2* b);  // ret = a-b
    static void add(Vec2* ret, Vec2* a, Vec2* b);  // ret = a+b
    static void pairwiseMult(Vec2* ret, Vec2* a, Vec2* b);  // ret = a.*b
    static void mult(Vec2* ret, Mat2x2<T>* a, Vec2* b);  // ret = a*b
    static bool equal(Vec2* a, Vec2* b);

    T m[2];  // Not private --> Avoid some overhead with getter setter methods
    char pad[ALIGNMENT - ((2*sizeof(T)) % ALIGNMENT)];
  };  // end class Vec2

  // Vec2 Constructors
  template <class T>
  Vec2<T>::Vec2() {
    m[0] = (T)0;
    m[1] = (T)0;
  };

  template <class T>
  Vec2<T>::Vec2(T* data) {
    m[0] = data[0];
    m[1] = data[1];
  };

  template <class T>
  Vec2<T>::Vec2(Vec2* data) {
    m[0] = data->m[0];
    m[1] = data->m[1];
  };

  template <class T>
  Vec2<T>::Vec2(T x, T y) {
    m[0] = x;
    m[1] = y;
  };

  // Setter methods
  template <class T>
  void Vec2<T>::zeros() {
    m[0] = (T)0;
    m[1] = (T)0;
  };

  template <class T>
  void Vec2<T>::ones() {
    m[0] = (T)1;
    m[1] = (T)1;
  };

  template <class T>
  void Vec2<T>::set(T x, T y) {
    m[0] = x;
    m[1] = y;
  };

  template <class T>
  void Vec2<T>::set(T* data) {
    m[0] = data[0];
    m[1] = data[1];
  };

  // Getter Methods
  template <class T>
  void Vec2<T>::print() {
    printf("| %+.4e |\n| %+.4e |\n", m[0], m[1]);
  };

  // Math operations
  template <class T>
  void Vec2<T>::accum(T* a) {
    m[0] += a[0];
    m[1] += a[1];
  };

  // Static Math operations
  template <class T>
  void Vec2<T>::normalize(Vec2* a) {
    T oneOverLength = (T)1.0 / std::sqrt(a->m[0] * a->m[0] + a->m[1] * a->m[1]);
    a->m[0] *= oneOverLength;
    a->m[1] *= oneOverLength;
  };

  template <class T>
  T Vec2<T>::length(Vec2* a) {
    return std::sqrt(a->m[0] * a->m[0] + a->m[1] * a->m[1]);
  };

  template <class T>
  void Vec2<T>::scale(Vec2* a, T s) {
    a->m[0] *= s;
    a->m[1] *= s;
  };

  template <class T>
  T Vec2<T>::dot(Vec2* a, Vec2* b) {
    return a->m[0] * b->m[0] + a->m[1] * b->m[1];
  };

  template <class T>
  void Vec2<T>::sub(Vec2* ret, Vec2* a, Vec2* b) {
    ret->m[0] = a->m[0] - b->m[0];
    ret->m[1] = a->m[1] - b->m[1];
  };

  template <class T>
  void Vec2<T>::add(Vec2* ret, Vec2* a, Vec2* b) {
    ret->m[0] = a->m[0] + b->m[0];
    ret->m[1] = a->m[1] + b->m[1];
  };

  template <class T>
  void Vec2<T>::pairwiseMult(Vec2* ret, Vec2* a, Vec2* b) {
    ret->m[0] = a->m[0] * b->m[0];
    ret->m[1] = a->m[1] * b->m[1];
  };

  template <class T>
  void Vec2<T>::mult(Vec2* ret, Mat2x2<T>* a, Vec2* b) {
#ifdef ROW_MAJOR
    ret->m[0] = a->m[0]*b->m[0]  + a->m[1]*b->m[1];
    ret->m[1] = a->m[2]*b->m[0]  + a->m[3]*b->m[1];
#endif
#ifdef COLUMN_MAJOR
    ret->m[0] = a->m[0]*b->m[0]  + a->m[2]*b->m[1];
    ret->m[1] = a->m[1]*b->m[0]  + a->m[3]*b->m[1];
#endif
  };

  template <class T>
  bool Vec2<T>::equal(Vec2* a, Vec2 *b) {
    for (int i = 0; i < 2; i ++)
      if (abs(a->m[i] - b->m[i]) > EPSILON)
        return false;
    return true;
  };

};  // end namespace math

#endif
