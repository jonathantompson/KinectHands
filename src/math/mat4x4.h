//
//  mat4x4.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#ifndef MATH_MAT4X4_HEADER
#define MATH_MAT4X4_HEADER

#include "alignment/data_align.h"

namespace math {
  // Note Matricies are all row-major:
  //            | m(0,0) m(0,1) m(0,2) m(0,3) |   |  0  1  2  3 |
  //   Mat4x4 = | m(1,0) m(1,1) m(1,2) m(1,3) | = |  4  5  6  7 |
  //            | m(2,0) m(2,1) m(2,2) m(2,3) |   |  8  9 10 11 |
  //            | m(3,0) m(3,1) m(3,2) m(3,3) |   | 12 13 14 15 |
  template <class T>
  class DATA_ALIGN(ALIGNMENT) Mat4x4 {
  public:
    Mat4x4();  // Default is identity
    explicit Mat4x4(T* data);
    explicit Mat4x4(Mat4x4* data);
    Mat4x4(T _00, T _01, T _02, T _03, 
      T _10, T _11, T _12, T _13,
      T _20, T _21, T _22, T _23,
      T _30, T _31, T _32, T _33);

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
    void identity();
    void set(T* data);
    inline void set(T data, int i, int j);  // m[i][j] == data
    void set(Mat4x4* data);
    void set(T s);

    // Math operations
    inline T det() { return det(this); }
    inline T trace() { return trace(this); }
    inline void scale(T s) { scale(this, this, s); }
    inline void scale(Mat4x4* a, T s) { scale(this, a, s); }
    inline void sub(Mat4x4* a, Mat4x4* b) { sub(this, a, b); }
    inline void add(Mat4x4* a, Mat4x4* b) { add(this, a, b); }
    inline void pairwiseMult(Mat4x4* a, Mat4x4* b) { pairwiseMult(this, a, b); }
    void transpose();  // this = this^t
    inline void transpose(Mat4x4 *a) { transpose(this, a); }
    inline bool equal(Mat4x4 *a) { return equal(this, a); }

    // Static Math operations
    static T det(Mat4x4* a);
    static T trace(Mat4x4* a);
    static void scale(Mat4x4* ret, Mat4x4* a, T s);  // ret = a*s
    static void sub(Mat4x4* ret, Mat4x4* a, Mat4x4* b);  // ret = a-b
    static void add(Mat4x4* ret, Mat4x4* a, Mat4x4* b);  // ret = a+b
    static void pairwiseMult(Mat4x4* ret, Mat4x4* a, Mat4x4* b);  // ret = a.*b
    static void inverse(Mat4x4* ret, Mat4x4* a);  // ret = a^-1
    static void inverseAffine(Mat4x4* ret, Mat4x4* a);  // ret = a^-1
    static void mult(Mat4x4* ret, Mat4x4* a, Mat4x4* b);  // ret = a*b
    static void transpose(Mat4x4* ret, Mat4x4 *a);  // ret = a^t
    static bool equal(Mat4x4* a, Mat4x4 *b);

    T m[16];  // Not private --> Avoid some overhead with getter setter methods
    char pad[ALIGNMENT - ((16*sizeof(T)) % ALIGNMENT)];
  };  // end class Mat4x4

  // Mat4x4 Constructors
  template <class T>
  Mat4x4<T>::Mat4x4() {
    m[0] = (T)1;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
    m[4] = (T)0;
    m[5] = (T)1;
    m[6] = (T)0;
    m[7] = (T)0;
    m[8] = (T)0;
    m[9] = (T)0;
    m[10] = (T)1;
    m[11] = (T)0;
    m[12] = (T)0;
    m[13] = (T)0;
    m[14] = (T)0;
    m[15] = (T)1;
  };
  template <class T>
  Mat4x4<T>::Mat4x4(T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
    m[4] = data[4];
    m[5] = data[5];
    m[6] = data[6];
    m[7] = data[7];
    m[8] = data[8];
    m[9] = data[9];
    m[10] = data[10];
    m[11] = data[11];
    m[12] = data[12];
    m[13] = data[13];
    m[14] = data[14];
    m[15] = data[15];
  };
  template <class T>
  Mat4x4<T>::Mat4x4(Mat4x4* data) {
    m[0] = data->m[0];
    m[1] = data->m[1];
    m[2] = data->m[2];
    m[3] = data->m[3];
    m[4] = data->m[4];
    m[5] = data->m[5];
    m[6] = data->m[6];
    m[7] = data->m[7];
    m[8] = data->m[8];
    m[9] = data->m[9];
    m[10] = data->m[10];
    m[11] = data->m[11];
    m[12] = data->m[12];
    m[13] = data->m[13];
    m[14] = data->m[14];
    m[15] = data->m[15];
  };
  template <class T>
  Mat4x4<T>::Mat4x4(T _00, T _01, T _02, T _03, 
    T _10, T _11, T _12, T _13,
    T _20, T _21, T _22, T _23,
    T _30, T _31, T _32, T _33) {
      m[0] = _00;
      m[1] = _01;
      m[2] = _02;
      m[3] = _03;
      m[4] = _10;
      m[5] = _11;
      m[6] = _12;
      m[7] = _13;
      m[8] = _20;
      m[9] = _21;
      m[10] = _22;
      m[11] = _23;
      m[12] = _30;
      m[13] = _31;
      m[14] = _32;
      m[15] = _33;
  };

  // Getter methods
  template <class T>
  void Mat4x4<T>::print() {
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[0],  m[1],  m[2],  m[3]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[4],  m[5],  m[6],  m[7]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[8],  m[9],  m[10], m[11]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[12], m[13], m[14], m[15]);
  };

  // Setter methods
  template <class T>
  void Mat4x4<T>::zeros() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
    m[4] = (T)0;
    m[5] = (T)0;
    m[6] = (T)0;
    m[7] = (T)0;
    m[8] = (T)0;
    m[9] = (T)0;
    m[10] = (T)0;
    m[11] = (T)0;
    m[12] = (T)0;
    m[13] = (T)0;
    m[14] = (T)0;
    m[15] = (T)0;
  };
  template <class T>
  void Mat4x4<T>::ones() {
    m[0] = (T)1;
    m[1] = (T)1;
    m[2] = (T)1;
    m[3] = (T)1;
    m[4] = (T)1;
    m[5] = (T)1;
    m[6] = (T)1;
    m[7] = (T)1;
    m[8] = (T)1;
    m[9] = (T)1;
    m[10] = (T)1;
    m[11] = (T)1;
    m[12] = (T)1;
    m[13] = (T)1;
    m[14] = (T)1;
    m[15] = (T)1;
  };
  template <class T>
  void Mat4x4<T>::identity() {
    m[0] = (T)1;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
    m[4] = (T)0;
    m[5] = (T)1;
    m[6] = (T)0;
    m[7] = (T)0;
    m[8] = (T)0;
    m[9] = (T)0;
    m[10] = (T)1;
    m[11] = (T)0;
    m[12] = (T)0;
    m[13] = (T)0;
    m[14] = (T)0;
    m[15] = (T)1;
  };
  template <class T>
  void Mat4x4<T>::set(T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
    m[4] = data[4];
    m[5] = data[5];
    m[6] = data[6];
    m[7] = data[7];
    m[8] = data[8];
    m[9] = data[9];
    m[10] = data[10];
    m[11] = data[11];
    m[12] = data[12];
    m[13] = data[13];
    m[14] = data[14];
    m[15] = data[15];
  };
  template <class T>
  void Mat4x4<T>::set(Mat4x4* data) {
    set(data->m);
  };
  template <class T>
  void Mat4x4<T>::set(T data, int i, int j) {
    m[i*3+j] = data;
  };
  template <class T>
  void Mat4x4<T>::set(T s) {
    m[0] = s;
    m[1] = s;
    m[2] = s;
    m[3] = s;
    m[4] = s;
    m[5] = s;
    m[6] = s;
    m[7] = s;
    m[8] = s;
    m[9] = s;
    m[10] = s;
    m[11] = s;
    m[12] = s;
    m[13] = s;
    m[14] = s;
    m[15] = s;
  };

  // Math operations
  template <class T>
  void Mat4x4<T>::transpose() {
    T temp;
    temp = m[1];  // Swap 1 and 4
    m[1] = m[4];
    m[4] = temp;
    temp = m[2];  // Swap 2 and 8
    m[2] = m[8];
    m[8] = temp;
    temp = m[3];  // Swap 3 and 12
    m[3] = m[12];
    m[12] = temp;
    temp = m[6];  // Swap 6 and 9
    m[6] = m[9];
    m[9] = temp;
    temp = m[7];  // Swap 7 and 13
    m[7] = m[13];
    m[13] = temp;
    temp = m[11];  // Swap 11 and 14
    m[11] = m[14];
    m[14] = temp;
  };

  // Static Math operations
  // 4x4 matrix determinant taken from here: 
  // http://www.geometrictools.com/LibMathematics/Algebra/Wm5Matrix4.inl
  template <class T>
  T Mat4x4<T>::det(Mat4x4* a) {
    T a0 = a->m[ 0]*a->m[ 5] - a->m[ 1]*a->m[ 4];
    T a1 = a->m[ 0]*a->m[ 6] - a->m[ 2]*a->m[ 4];
    T a2 = a->m[ 0]*a->m[ 7] - a->m[ 3]*a->m[ 4];
    T a3 = a->m[ 1]*a->m[ 6] - a->m[ 2]*a->m[ 5];
    T a4 = a->m[ 1]*a->m[ 7] - a->m[ 3]*a->m[ 5];
    T a5 = a->m[ 2]*a->m[ 7] - a->m[ 3]*a->m[ 6];
    T b0 = a->m[ 8]*a->m[13] - a->m[ 9]*a->m[12];
    T b1 = a->m[ 8]*a->m[14] - a->m[10]*a->m[12];
    T b2 = a->m[ 8]*a->m[15] - a->m[11]*a->m[12];
    T b3 = a->m[ 9]*a->m[14] - a->m[10]*a->m[13];
    T b4 = a->m[ 9]*a->m[15] - a->m[11]*a->m[13];
    T b5 = a->m[10]*a->m[15] - a->m[11]*a->m[14];
    return a0*b5 - a1*b4 + a2*b3 + a3*b2 - a4*b1 + a5*b0;
  };
  template <class T>
  T Mat4x4<T>::trace(Mat4x4* a) {  // Sum of elements on main diagonal
    return a->m[0] + a->m[5] + a->m[10] + a->m[15];
  };
  template <class T>
  void Mat4x4<T>::scale(Mat4x4* ret, Mat4x4* a, T s) {
    ret->m[0] = a->m[0] * s;
    ret->m[1] = a->m[1] * s;
    ret->m[2] = a->m[2] * s;
    ret->m[3] = a->m[3] * s;
    ret->m[4] = a->m[4] * s;
    ret->m[5] = a->m[5] * s;
    ret->m[6] = a->m[6] * s;
    ret->m[7] = a->m[7] * s;
    ret->m[8] = a->m[8] * s;
    ret->m[9] = a->m[9] * s;
    ret->m[10] = a->m[10] * s;
    ret->m[11] = a->m[11] * s;
    ret->m[12] = a->m[12] * s;
    ret->m[13] = a->m[13] * s;
    ret->m[14] = a->m[14] * s;
    ret->m[15] = a->m[15] * s;
  };
  template <class T>
  void Mat4x4<T>::sub(Mat4x4* ret, Mat4x4* a, Mat4x4* b) {
    ret->m[0] = a->m[0] - b->m[0];
    ret->m[1] = a->m[1] - b->m[1];
    ret->m[2] = a->m[2] - b->m[2];
    ret->m[3] = a->m[3] - b->m[3];
    ret->m[4] = a->m[4] - b->m[4];
    ret->m[5] = a->m[5] - b->m[5];
    ret->m[6] = a->m[6] - b->m[6];
    ret->m[7] = a->m[7] - b->m[7];
    ret->m[8] = a->m[8] - b->m[8];
    ret->m[9] = a->m[9] - b->m[9];
    ret->m[10] = a->m[10] - b->m[10];
    ret->m[11] = a->m[11] - b->m[11];
    ret->m[12] = a->m[12] - b->m[12];
    ret->m[13] = a->m[13] - b->m[13];
    ret->m[14] = a->m[14] - b->m[14];
    ret->m[15] = a->m[15] - b->m[15];
  };
  template <class T>
  void Mat4x4<T>::add(Mat4x4* ret, Mat4x4* a, Mat4x4* b) {
    ret->m[0] = a->m[0] + b->m[0];
    ret->m[1] = a->m[1] + b->m[1];
    ret->m[2] = a->m[2] + b->m[2];
    ret->m[3] = a->m[3] + b->m[3];
    ret->m[4] = a->m[4] + b->m[4];
    ret->m[5] = a->m[5] + b->m[5];
    ret->m[6] = a->m[6] + b->m[6];
    ret->m[7] = a->m[7] + b->m[7];
    ret->m[8] = a->m[8] + b->m[8];
    ret->m[9] = a->m[9] + b->m[9];
    ret->m[10] = a->m[10] + b->m[10];
    ret->m[11] = a->m[11] + b->m[11];
    ret->m[12] = a->m[12] + b->m[12];
    ret->m[13] = a->m[13] + b->m[13];
    ret->m[14] = a->m[14] + b->m[14];
    ret->m[15] = a->m[15] + b->m[15];
  };
  template <class T>
  void Mat4x4<T>::pairwiseMult(Mat4x4* ret, Mat4x4* a, Mat4x4* b) {
    ret->m[0] = a->m[0] * b->m[0];
    ret->m[1] = a->m[1] * b->m[1];
    ret->m[2] = a->m[2] * b->m[2];
    ret->m[3] = a->m[3] * b->m[3];
    ret->m[4] = a->m[4] * b->m[4];
    ret->m[5] = a->m[5] * b->m[5];
    ret->m[6] = a->m[6] * b->m[6];
    ret->m[7] = a->m[7] * b->m[7];
    ret->m[8] = a->m[8] * b->m[8];
    ret->m[9] = a->m[9] * b->m[9];
    ret->m[10] = a->m[10] * b->m[10];
    ret->m[11] = a->m[11] * b->m[11];
    ret->m[12] = a->m[12] * b->m[12];
    ret->m[13] = a->m[13] * b->m[13];
    ret->m[14] = a->m[14] * b->m[14];
    ret->m[15] = a->m[15] * b->m[15];
  };
  // 4x4 matrix invert taken from here: 
  // http://www.geometrictools.com/LibMathematics/Algebra/Wm5Matrix4.inl
  template <class T>
  void Mat4x4<T>::inverse(Mat4x4* ret, Mat4x4* a) {
    T a0 = a->m[ 0]*a->m[ 5] - a->m[ 1]*a->m[ 4];
    T a1 = a->m[ 0]*a->m[ 6] - a->m[ 2]*a->m[ 4];
    T a2 = a->m[ 0]*a->m[ 7] - a->m[ 3]*a->m[ 4];
    T a3 = a->m[ 1]*a->m[ 6] - a->m[ 2]*a->m[ 5];
    T a4 = a->m[ 1]*a->m[ 7] - a->m[ 3]*a->m[ 5];
    T a5 = a->m[ 2]*a->m[ 7] - a->m[ 3]*a->m[ 6];
    T b0 = a->m[ 8]*a->m[13] - a->m[ 9]*a->m[12];
    T b1 = a->m[ 8]*a->m[14] - a->m[10]*a->m[12];
    T b2 = a->m[ 8]*a->m[15] - a->m[11]*a->m[12];
    T b3 = a->m[ 9]*a->m[14] - a->m[10]*a->m[13];
    T b4 = a->m[ 9]*a->m[15] - a->m[11]*a->m[13];
    T b5 = a->m[10]*a->m[15] - a->m[11]*a->m[14];

    T det = a0*b5 - a1*b4 + a2*b3 + a3*b2 - a4*b1 + a5*b0;
    if (abs(det) > EPSILON) {
      ret->m[ 0] = + a->m[ 5]*b5 - a->m[ 6]*b4 + a->m[ 7]*b3;
      ret->m[ 4] = - a->m[ 4]*b5 + a->m[ 6]*b2 - a->m[ 7]*b1;
      ret->m[ 8] = + a->m[ 4]*b4 - a->m[ 5]*b2 + a->m[ 7]*b0;
      ret->m[12] = - a->m[ 4]*b3 + a->m[ 5]*b1 - a->m[ 6]*b0;
      ret->m[ 1] = - a->m[ 1]*b5 + a->m[ 2]*b4 - a->m[ 3]*b3;
      ret->m[ 5] = + a->m[ 0]*b5 - a->m[ 2]*b2 + a->m[ 3]*b1;
      ret->m[ 9] = - a->m[ 0]*b4 + a->m[ 1]*b2 - a->m[ 3]*b0;
      ret->m[13] = + a->m[ 0]*b3 - a->m[ 1]*b1 + a->m[ 2]*b0;
      ret->m[ 2] = + a->m[13]*a5 - a->m[14]*a4 + a->m[15]*a3;
      ret->m[ 6] = - a->m[12]*a5 + a->m[14]*a2 - a->m[15]*a1;
      ret->m[10] = + a->m[12]*a4 - a->m[13]*a2 + a->m[15]*a0;
      ret->m[14] = - a->m[12]*a3 + a->m[13]*a1 - a->m[14]*a0;
      ret->m[ 3] = - a->m[ 9]*a5 + a->m[10]*a4 - a->m[11]*a3;
      ret->m[ 7] = + a->m[ 8]*a5 - a->m[10]*a2 + a->m[11]*a1;
      ret->m[11] = - a->m[ 8]*a4 + a->m[ 9]*a2 - a->m[11]*a0;
      ret->m[15] = + a->m[ 8]*a3 - a->m[ 9]*a1 + a->m[10]*a0;

      T invDet = (T)1/det;
      ret->scale(invDet);
    } else {
      ret->zeros();
    }
  };
  template <class T>
  void Mat4x4<T>::mult(Mat4x4* ret, Mat4x4* a, Mat4x4* b) {
    ret->m[0] = a->m[0]*b->m[0] + a->m[1]*b->m[4] + a->m[2]*b->m[8] + 
      a->m[3]*b->m[12];
    ret->m[1] = a->m[0]*b->m[1] + a->m[1]*b->m[5] + a->m[2]*b->m[9] + 
      a->m[3]*b->m[13];
    ret->m[2] = a->m[0]*b->m[2] + a->m[1]*b->m[6] + a->m[2]*b->m[10] + 
      a->m[3]*b->m[14];
    ret->m[3] = a->m[0]*b->m[3] + a->m[1]*b->m[7] + a->m[2]*b->m[11] + 
      a->m[3]*b->m[15];
    ret->m[4] = a->m[4]*b->m[0] + a->m[5]*b->m[4] + a->m[6]*b->m[8] + 
      a->m[7]*b->m[12];
    ret->m[5] = a->m[4]*b->m[1] + a->m[5]*b->m[5] + a->m[6]*b->m[9] + 
      a->m[7]*b->m[13];
    ret->m[6] = a->m[4]*b->m[2] + a->m[5]*b->m[6] + a->m[6]*b->m[10] + 
      a->m[7]*b->m[14];
    ret->m[7] = a->m[4]*b->m[3] + a->m[5]*b->m[7] + a->m[6]*b->m[11] + 
      a->m[7]*b->m[15];
    ret->m[8] = a->m[8]*b->m[0] + a->m[9]*b->m[4] + a->m[10]*b->m[8] + 
      a->m[11]*b->m[12];
    ret->m[9] = a->m[8]*b->m[1] + a->m[9]*b->m[5] + a->m[10]*b->m[9] + 
      a->m[11]*b->m[13];
    ret->m[10] = a->m[8]*b->m[2] + a->m[9]*b->m[6] + a->m[10]*b->m[10] + 
      a->m[11]*b->m[14];
    ret->m[11] = a->m[8]*b->m[3] + a->m[9]*b->m[7] + a->m[10]*b->m[11] + 
      a->m[11]*b->m[15];
    ret->m[12] = a->m[12]*b->m[0] + a->m[13]*b->m[4] + a->m[14]*b->m[8] + 
      a->m[15]*b->m[12];
    ret->m[13] = a->m[12]*b->m[1] + a->m[13]*b->m[5] + a->m[14]*b->m[9] + 
      a->m[15]*b->m[13];
    ret->m[14] = a->m[12]*b->m[2] + a->m[13]*b->m[6] + a->m[14]*b->m[10] + 
      a->m[15]*b->m[14];
    ret->m[15] = a->m[12]*b->m[3] + a->m[13]*b->m[7] + a->m[14]*b->m[11] + 
      a->m[15]*b->m[15];
  };
  template <class T>
  void Mat4x4<T>::transpose(Mat4x4* ret, Mat4x4* a) {
    ret->m[0] = a->m[0];
    ret->m[1] = a->m[4];
    ret->m[2] = a->m[8];
    ret->m[3] = a->m[12];
    ret->m[4] = a->m[1];
    ret->m[5] = a->m[5];
    ret->m[6] = a->m[9];
    ret->m[7] = a->m[13];
    ret->m[8] = a->m[2];
    ret->m[9] = a->m[6];
    ret->m[10] = a->m[10];
    ret->m[11] = a->m[14];
    ret->m[12] = a->m[3];
    ret->m[13] = a->m[7];
    ret->m[14] = a->m[11];
    ret->m[15] = a->m[15];
  };
  template <class T>
  bool Mat4x4<T>::equal(Mat4x4* a, Mat4x4 *b) {
    for (int i = 0; i < 16; i ++)
      if (abs(a->m[i] - b->m[i]) > EPSILON)
        return false;
    return true;
  };

  //            | m(0,0) m(0,1) m(0,2) m(0,3) |   |  0  1  2  3 |
  //   Mat4x4 = | m(1,0) m(1,1) m(1,2) m(1,3) | = |  4  5  6  7 |
  //            | m(2,0) m(2,1) m(2,2) m(2,3) |   |  8  9 10 11 |
  //            | m(3,0) m(3,1) m(3,2) m(3,3) |   | 12 13 14 15 |

};  // end namespace math

#endif
