//
//  vec4.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#ifndef MATH_VEC4_HEADER
#define MATH_VEC4_HEADER

#include <float.h>
#include "alignment/data_align.h"

#ifndef EPSILON
#define EPSILON (2 * FLT_EPSILON)  // 2 times machine precision for float
#endif

namespace math {
  template <class T>
  class Mat4x4;

  // Note Vectors are all column vectors (OpenGL uses column vectors):
  //          | m(0,0) |   | 0 |
  //   Vec4 = | m(1,0) | = | 1 |
  //          | m(2,0) |   | 2 |
  //          | m(3,0) |   | 3 |
  template <class T>
  class DATA_ALIGN(ALIGNMENT) Vec4 {
   public:
    Vec4();
    explicit Vec4(T* data);
    explicit Vec4(Vec4* data);
    Vec4(T x, T y, T z, T w);

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
    void set(T x, T y, T z, T w);
    void set(T* data);
    inline void set(Vec4* data) { this->set(data->m); }

    // Math operations
    inline void normalize() { normalize(this); }
    inline T length() { return length(this); }
    inline void scale(T s) { scale(this, s); }
    inline T dot(Vec4* a) { return dot(this, a); }
    inline void sub(Vec4* a, Vec4* b) { sub(this, a, b); }
    inline void add(Vec4* a, Vec4* b) { add(this, a, b); }
    inline void pairwiseMult(Vec4* a, Vec4* b) { pairwiseMult(this, a, b); }
    inline void mult(Mat4x4<T>* a, Vec4* b) { mult(this, a, b); }
    void accum(T* a);  // this += accum
    inline bool equal(Vec4* a) { return equal(this, a); }

    // Static Math operations
    static void normalize(Vec4* a);
    static T length(Vec4* a);
    static void scale(Vec4* a, T s);
    static T dot(Vec4* a, Vec4* b);  // a.b
    static void sub(Vec4* ret, Vec4* a, Vec4* b);  // ret = a-b
    static void add(Vec4* ret, Vec4* a, Vec4* b);  // ret = a+b
    static void pairwiseMult(Vec4* ret, Vec4* a, Vec4* b);  // ret = a.*b
    static void mult(Vec4* ret, Mat4x4<T>* a, Vec4* b);  // ret = a*b
    static bool equal(Vec4* a, Vec4* b);

    T m[4];  // Not private --> Avoid some overhead with getter setter methods
    char pad[ALIGNMENT - ((4*sizeof(T)) % ALIGNMENT)];
  };  // end class Vec4

  // Vec4 Constructors
  template <class T>
  Vec4<T>::Vec4() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
  };

  template <class T>
  Vec4<T>::Vec4(T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
  };

  template <class T>
  Vec4<T>::Vec4(Vec4* data) {
    m[0] = data->m[0];
    m[1] = data->m[1];
    m[2] = data->m[2];
    m[3] = data->m[3];
  };

  template <class T>
  Vec4<T>::Vec4(T x, T y, T z, T w) {
    m[0] = x;
    m[1] = y;
    m[2] = z;
    m[3] = w;
  };

  // Setter methods
  template <class T>
  void Vec4<T>::zeros() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
  };

  template <class T>
  void Vec4<T>::ones() {
    m[0] = (T)1;
    m[1] = (T)1;
    m[2] = (T)1;
    m[3] = (T)1;
  };

  template <class T>
  void Vec4<T>::set(T x, T y, T z, T w) {
    m[0] = x;
    m[1] = y;
    m[2] = z;
    m[3] = w;
  };

  template <class T>
  void Vec4<T>::set(T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
  };

  // Getter Methods
  template <class T>
  void Vec4<T>::print() {
    printf("| %+.4e |\n| %+.4e |\n| %+.4e |\n| %+.4e |\n", 
      m[0], m[1], m[2], m[3]);
  };

  // Math operations
  template <class T>
  void Vec4<T>::accum(T* a) {
    m[0] += a[0];
    m[1] += a[1];
    m[2] += a[2];
    m[3] += a[3];
  };

  // Static Math operations
  template <class T>
  void Vec4<T>::normalize(Vec4* a) {
    T oneOverLength = (T)1.0 / sqrt(a->m[0] * a->m[0] + 
      a->m[1] * a->m[1] + 
      a->m[2] * a->m[2] + 
      a->m[3] * a->m[3]);
    a->m[0] *= oneOverLength;
    a->m[1] *= oneOverLength;
    a->m[2] *= oneOverLength;
    a->m[3] *= oneOverLength;
  };

  template <class T>
  T Vec4<T>::length(Vec4* a) {
    return sqrt(a->m[0] * a->m[0] + 
      a->m[1] * a->m[1] + 
      a->m[2] * a->m[2] + 
      a->m[3] * a->m[3]);
  };

  template <class T>
  void Vec4<T>::scale(Vec4* a, T s) {
    a->m[0] *= s;
    a->m[1] *= s;
    a->m[2] *= s;
    a->m[3] *= s;
  };

  template <class T>
  T Vec4<T>::dot(Vec4* a, Vec4* b) {
    return a->m[0] * b->m[0] + 
      a->m[1] * b->m[1] +
      a->m[2] * b->m[2] +
      a->m[3] * b->m[3];
  };

  template <class T>
  void Vec4<T>::sub(Vec4* ret, Vec4* a, Vec4* b) {
    ret->m[0] = a->m[0] - b->m[0];
    ret->m[1] = a->m[1] - b->m[1];
    ret->m[2] = a->m[2] - b->m[2];
    ret->m[3] = a->m[3] - b->m[3];
  };

  template <class T>
  void Vec4<T>::add(Vec4* ret, Vec4* a, Vec4* b) {
    ret->m[0] = a->m[0] + b->m[0];
    ret->m[1] = a->m[1] + b->m[1];
    ret->m[2] = a->m[2] + b->m[2];
    ret->m[3] = a->m[3] + b->m[3];
  };

  template <class T>
  void Vec4<T>::pairwiseMult(Vec4* ret, Vec4* a, Vec4* b) {
    ret->m[0] = a->m[0] * b->m[0];
    ret->m[1] = a->m[1] * b->m[1];
    ret->m[2] = a->m[2] * b->m[2];
    ret->m[3] = a->m[3] * b->m[3];
  };

  template <class T>
  void Vec4<T>::mult(Vec4* ret, Mat4x4<T>* a, Vec4* b) {
#ifdef ROW_MAJOR
    ret->m[0] = a->m[0]*b->m[0]  + a->m[1]*b->m[1]  + a->m[2]*b->m[2]  
                + a->m[3]*b->m[3];
    ret->m[1] = a->m[4]*b->m[0]  + a->m[5]*b->m[1]  + a->m[6]*b->m[2]  
                + a->m[7]*b->m[3];
    ret->m[2] = a->m[8]*b->m[0]  + a->m[9]*b->m[1]  + a->m[10]*b->m[2] 
                + a->m[11]*b->m[3];
    ret->m[3] = a->m[12]*b->m[0] + a->m[13]*b->m[1] + a->m[14]*b->m[2] 
                + a->m[15]*b->m[3];
#endif
#ifdef COLUMN_MAJOR
    ret->m[0] = a->m[0]*b->m[0] + a->m[4]*b->m[1] + a->m[8]*b->m[2]  
                + a->m[12]*b->m[3];
    ret->m[1] = a->m[1]*b->m[0] + a->m[5]*b->m[1] + a->m[9]*b->m[2]  
                + a->m[13]*b->m[3];
    ret->m[2] = a->m[2]*b->m[0] + a->m[6]*b->m[1] + a->m[10]*b->m[2] 
                + a->m[14]*b->m[3];
    ret->m[3] = a->m[3]*b->m[0] + a->m[7]*b->m[1] + a->m[11]*b->m[2] 
                + a->m[15]*b->m[3];
#endif
  };

  template <class T>
  bool Vec4<T>::equal(Vec4* a, Vec4 *b) {
    for (int i = 0; i < 4; i ++)
      if (abs(a->m[i] - b->m[i]) > EPSILON)
        return false;
    return true;
  };

};  // end namespace math

#endif
