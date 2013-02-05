//
//  mat3x3.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#ifndef MATH_MAT3X3_HEADER
#define MATH_MAT3X3_HEADER

#include "alignment/data_align.h"

namespace math {
  // Note Matricies are all row-major:
  //            | m(0,0) m(0,1) m(0,2) |   | 0 1 2 |
  //   Mat3x3 = | m(1,0) m(1,1) m(1,2) | = | 3 4 5 |
  //            | m(2,0) m(2,1) m(2,2) |   | 6 7 8 |
  template <class T>
  class DATA_ALIGN(ALIGNMENT) Mat3x3 {
  public:
    Mat3x3();  // Default is identity
    explicit Mat3x3(T* data);
    explicit Mat3x3(Mat3x3* data);
    Mat3x3(T _00, T _01, T _02, T _10, T _11, T _12, T _20, T _21, T _22);

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
    void set(Mat3x3* data);
    void set(T s);

    // Math operations
    inline T det() { return det(this); }
    inline T trace() { return trace(this); }
    inline void scale(T s) { scale(this, this, s); }
    inline void scale(Mat3x3* a, T s) { scale(this, a, s); }
    inline void sub(Mat3x3* a, Mat3x3* b) { sub(this, a, b); }
    inline void add(Mat3x3* a, Mat3x3* b) { add(this, a, b); }
    inline void pairwiseMult(Mat3x3* a, Mat3x3* b) { pairwiseMult(this, a, b); }
    void transpose();  // this = this^t
    inline void transpose(Mat3x3 *a) { transpose(this, a); }
    inline bool equal(Mat3x3 *a) { return equal(this, a); }

    // Static Math operations
    static T det(Mat3x3* a);
    static T trace(Mat3x3* a);
    static void scale(Mat3x3* ret, Mat3x3* a, T s);  // ret = a*s
    static void sub(Mat3x3* ret, Mat3x3* a, Mat3x3* b);  // ret = a-b
    static void add(Mat3x3* ret, Mat3x3* a, Mat3x3* b);  // ret = a+b
    static void pairwiseMult(Mat3x3* ret, Mat3x3* a, Mat3x3* b);  // ret = a.*b
    static void inverse(Mat3x3* ret, Mat3x3* a);  // ret = a^-1
    static void mult(Mat3x3* ret, Mat3x3* a, Mat3x3* b);  // ret = a*b
    static void transpose(Mat3x3* ret, Mat3x3 *a);  // ret = a^t
    static bool equal(Mat3x3* a, Mat3x3 *b);

    T m[9];  // Not private --> Avoid some overhead with getter setter methods
    char pad[ALIGNMENT - ((9*sizeof(T)) % ALIGNMENT)];
  };  // end class Mat3x3

  // Mat3x3 Constructors
  template <class T>
  Mat3x3<T>::Mat3x3() {
    m[0] = (T)1;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
    m[4] = (T)1;
    m[5] = (T)0;
    m[6] = (T)0;
    m[7] = (T)0;
    m[8] = (T)1;
  };
  template <class T>
  Mat3x3<T>::Mat3x3(T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
    m[4] = data[4];
    m[5] = data[5];
    m[6] = data[6];
    m[7] = data[7];
    m[8] = data[8];
  };
  template <class T>
  Mat3x3<T>::Mat3x3(Mat3x3* data) {
    m[0] = data->m[0];
    m[1] = data->m[1];
    m[2] = data->m[2];
    m[3] = data->m[3];
    m[4] = data->m[4];
    m[5] = data->m[5];
    m[6] = data->m[6];
    m[7] = data->m[7];
    m[8] = data->m[8];
  };
  template <class T>
  Mat3x3<T>::Mat3x3(T _00, T _01, T _02,
    T _10, T _11, T _12,
    T _20, T _21, T _22) {
      m[0] = _00;
      m[1] = _01;
      m[2] = _02;
      m[3] = _10;
      m[4] = _11;
      m[5] = _12;
      m[6] = _20;
      m[7] = _21;
      m[8] = _22;
  };

  // Getter methods
  template <class T>
  void Mat3x3<T>::print() {
    printf("| %+.4e  %+.4e  %+.4e |\n", m[0], m[1], m[2]);
    printf("| %+.4e  %+.4e  %+.4e |\n", m[3], m[4], m[5]);
    printf("| %+.4e  %+.4e  %+.4e |\n", m[6], m[7], m[8]);
  };

  // Setter methods
  template <class T>
  void Mat3x3<T>::zeros() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
    m[4] = (T)0;
    m[5] = (T)0;
    m[6] = (T)0;
    m[7] = (T)0;
    m[8] = (T)0;
  };
  template <class T>
  void Mat3x3<T>::ones() {
    m[0] = (T)1;
    m[1] = (T)1;
    m[2] = (T)1;
    m[3] = (T)1;
    m[4] = (T)1;
    m[5] = (T)1;
    m[6] = (T)1;
    m[7] = (T)1;
    m[8] = (T)1;
  };
  template <class T>
  void Mat3x3<T>::identity() {
    m[0] = (T)1;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
    m[4] = (T)1;
    m[5] = (T)0;
    m[6] = (T)0;
    m[7] = (T)0;
    m[8] = (T)1;
  };
  template <class T>
  void Mat3x3<T>::set(T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
    m[4] = data[4];
    m[5] = data[5];
    m[6] = data[6];
    m[7] = data[7];
    m[8] = data[8];
  };
  template <class T>
  void Mat3x3<T>::set(Mat3x3* data) {
    set(data->m);
  };
  template <class T>
  void Mat3x3<T>::set(T data, int i, int j) {
    m[i*3+j] = data;
  };
  template <class T>
  void Mat3x3<T>::set(T s) {
    m[0] = s;
    m[1] = s;
    m[2] = s;
    m[3] = s;
    m[4] = s;
    m[5] = s;
    m[6] = s;
    m[7] = s;
    m[8] = s;
  };

  // Math operations
  template <class T>
  void Mat3x3<T>::transpose() {
    T temp;
    temp = m[1];
    m[1] = m[3];
    m[3] = temp;
    temp = m[2];
    m[2] = m[6];
    m[6] = temp;
    temp = m[5];
    m[5] = m[7];
    m[7] = temp;
  };

  // Static Math operations
  template <class T>
  T Mat3x3<T>::det(Mat3x3* a) {
    return a->m[0] * (a->m[8] * a->m[4] - a->m[7] * a->m[5]) 
         - a->m[3] * (a->m[8] * a->m[1] - a->m[7] * a->m[2]) 
         + a->m[6] * (a->m[5] * a->m[1] - a->m[4] * a->m[2]);
  };
  template <class T>
  T Mat3x3<T>::trace(Mat3x3* a) {  // Sum of elements on main diagonal
    return a->m[0] + a->m[4] + a->m[8];
  };
  template <class T>
  void Mat3x3<T>::scale(Mat3x3* ret, Mat3x3* a, T s) {
    ret->m[0] = a->m[0] * s;
    ret->m[1] = a->m[1] * s;
    ret->m[2] = a->m[2] * s;
    ret->m[3] = a->m[3] * s;
    ret->m[4] = a->m[4] * s;
    ret->m[5] = a->m[5] * s;
    ret->m[6] = a->m[6] * s;
    ret->m[7] = a->m[7] * s;
    ret->m[8] = a->m[8] * s;
  };
  template <class T>
  void Mat3x3<T>::sub(Mat3x3* ret, Mat3x3* a, Mat3x3* b) {
    ret->m[0] = a->m[0] - b->m[0];
    ret->m[1] = a->m[1] - b->m[1];
    ret->m[2] = a->m[2] - b->m[2];
    ret->m[3] = a->m[3] - b->m[3];
    ret->m[4] = a->m[4] - b->m[4];
    ret->m[5] = a->m[5] - b->m[5];
    ret->m[6] = a->m[6] - b->m[6];
    ret->m[7] = a->m[7] - b->m[7];
    ret->m[8] = a->m[8] - b->m[8];
  };
  template <class T>
  void Mat3x3<T>::add(Mat3x3* ret, Mat3x3* a, Mat3x3* b) {
    ret->m[0] = a->m[0] + b->m[0];
    ret->m[1] = a->m[1] + b->m[1];
    ret->m[2] = a->m[2] + b->m[2];
    ret->m[3] = a->m[3] + b->m[3];
    ret->m[4] = a->m[4] + b->m[4];
    ret->m[5] = a->m[5] + b->m[5];
    ret->m[6] = a->m[6] + b->m[6];
    ret->m[7] = a->m[7] + b->m[7];
    ret->m[8] = a->m[8] + b->m[8];
  };
  template <class T>
  void Mat3x3<T>::pairwiseMult(Mat3x3* ret, Mat3x3* a, Mat3x3* b) {
    ret->m[0] = a->m[0] * b->m[0];
    ret->m[1] = a->m[1] * b->m[1];
    ret->m[2] = a->m[2] * b->m[2];
    ret->m[3] = a->m[3] * b->m[3];
    ret->m[4] = a->m[4] * b->m[4];
    ret->m[5] = a->m[5] * b->m[5];
    ret->m[6] = a->m[6] * b->m[6];
    ret->m[7] = a->m[7] * b->m[7];
    ret->m[8] = a->m[8] * b->m[8];
  };
  template <class T>
  void Mat3x3<T>::inverse(Mat3x3* ret, Mat3x3* a) {
    T det = a->det();
    ret->m[0] = a->m[8]*a->m[4]-a->m[7]*a->m[5];
    ret->m[1] = -(a->m[8]*a->m[1]-a->m[7]*a->m[2]);
    ret->m[2] = a->m[5]*a->m[1]-a->m[4]*a->m[2];
    ret->m[3] = -(a->m[8]*a->m[3]-a->m[6]*a->m[5]);
    ret->m[4] = a->m[8]*a->m[0]-a->m[6]*a->m[2];
    ret->m[5] = -(a->m[5]*a->m[0]-a->m[3]*a->m[2]);
    ret->m[6] = a->m[7]*a->m[3]-a->m[6]*a->m[4];
    ret->m[7] = -(a->m[7]*a->m[0]-a->m[6]*a->m[1]);
    ret->m[8] = a->m[4]*a->m[0]-a->m[3]*a->m[1];
    ret->scale(1.0f/det);
  };
  template <class T>
  void Mat3x3<T>::mult(Mat3x3* ret, Mat3x3* a, Mat3x3* b) {
    ret->m[0] = a->m[0]*b->m[0] + a->m[1]*b->m[3] + a->m[2]*b->m[6];
    ret->m[1] = a->m[0]*b->m[1] + a->m[1]*b->m[4] + a->m[2]*b->m[7];
    ret->m[2] = a->m[0]*b->m[2] + a->m[1]*b->m[5] + a->m[2]*b->m[8];
    ret->m[3] = a->m[3]*b->m[0] + a->m[4]*b->m[3] + a->m[5]*b->m[6];
    ret->m[4] = a->m[3]*b->m[1] + a->m[4]*b->m[4] + a->m[5]*b->m[7];
    ret->m[5] = a->m[3]*b->m[2] + a->m[4]*b->m[5] + a->m[5]*b->m[8];
    ret->m[6] = a->m[6]*b->m[0] + a->m[7]*b->m[3] + a->m[8]*b->m[6];
    ret->m[7] = a->m[6]*b->m[1] + a->m[7]*b->m[4] + a->m[8]*b->m[7];
    ret->m[8] = a->m[6]*b->m[2] + a->m[7]*b->m[5] + a->m[8]*b->m[8];
  };
  template <class T>
  void Mat3x3<T>::transpose(Mat3x3* ret, Mat3x3* a) {
    ret->m[0] = a->m[0];
    ret->m[1] = a->m[3];
    ret->m[2] = a->m[6];
    ret->m[3] = a->m[1];
    ret->m[4] = a->m[4];
    ret->m[5] = a->m[7];
    ret->m[6] = a->m[2];
    ret->m[7] = a->m[5];
    ret->m[8] = a->m[8];
  };
  template <class T>
  bool Mat3x3<T>::equal(Mat3x3* a, Mat3x3 *b) {
    for (int i = 0; i < 9; i ++)
      if (abs(a->m[i] - b->m[i]) > EPSILON)
        return false;
    return true;
  };

};  // end namespace math

#endif
