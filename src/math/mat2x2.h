//
//  mat2x2.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#ifndef MATH_MAT2X2_HEADER
#define MATH_MAT2X2_HEADER

#include "alignment/data_align.h"

namespace math {
  // Note Matricies are all row-major:
  //            | m(0,0) m(0,1) |   | 0 1 |
  //   Mat2x2 = | m(1,0) m(1,1) | = | 2 3 |
  template <class T>
  class DATA_ALIGN(ALIGNMENT) Mat2x2 {
  public:
    Mat2x2();  // Default is identity
    explicit Mat2x2(T* data);
    explicit Mat2x2(Mat2x2* data);
    Mat2x2(T _00, T _01, T _10, T _11);

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
    void set(Mat2x2* data);
    void set(T s);

    // Math operations
    inline T det() { return det(this); }
    inline T trace() { return trace(this); }
    inline void scale(T s) { scale(this, this, s); }
    inline void scale(Mat2x2* a, T s) { scale(this, a, s); }
    inline void sub(Mat2x2* a, Mat2x2* b) { sub(this, a, b); }
    inline void add(Mat2x2* a, Mat2x2* b) { add(this, a, b); }
    inline void pairwiseMult(Mat2x2* a, Mat2x2* b) { pairwiseMult(this, a, b); }
    void transpose();  // this = this^t
    inline void transpose(Mat2x2 *a) { transpose(this, a); }
    inline bool equal(Mat2x2 *a) { return equal(this, a); }

    // Static Math operations
    static T det(Mat2x2* a);
    static T trace(Mat2x2* a);
    static void scale(Mat2x2* ret, Mat2x2* a, T s);  // ret = a*s
    static void sub(Mat2x2* ret, Mat2x2* a, Mat2x2* b);  // ret = a-b
    static void add(Mat2x2* ret, Mat2x2* a, Mat2x2* b);  // ret = a+b
    static void pairwiseMult(Mat2x2* ret, Mat2x2* a, Mat2x2* b);  // ret = a.*b
    static void inverse(Mat2x2* ret, Mat2x2* a);  // ret = a^-1
    static void mult(Mat2x2* ret, Mat2x2* a, Mat2x2* b);  // ret = a*b
    static void transpose(Mat2x2* ret, Mat2x2 *a);  // ret = a^t
    static bool equal(Mat2x2* a, Mat2x2 *b);

    T m[4];  // Not private --> Avoid some overhead with getter setter methods
    char pad[ALIGNMENT - ((4*sizeof(T)) % ALIGNMENT)]; 
  };  // end class Mat2x2

  // Mat2x2 Constructors
  template <class T>
  Mat2x2<T>::Mat2x2() {
    m[0] = (T)1;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)1;
  };
  template <class T>
  Mat2x2<T>::Mat2x2(T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
  };
  template <class T>
  Mat2x2<T>::Mat2x2(Mat2x2* data) {
    m[0] = data->m[0];
    m[1] = data->m[1];
    m[2] = data->m[2];
    m[3] = data->m[3];
  };
  template <class T>
  Mat2x2<T>::Mat2x2(T _00, T _01, T _10, T _11) {
      m[0] = _00;
      m[1] = _01;
      m[2] = _10;
      m[3] = _11;
  };

  // Getter methods
  template <class T>
  void Mat2x2<T>::print() {
    printf("| %+.4e  %+.4e |\n", m[0], m[1]);
    printf("| %+.4e  %+.4e |\n", m[2], m[3]);
  };

  // Setter methods
  template <class T>
  void Mat2x2<T>::zeros() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
  };
  template <class T>
  void Mat2x2<T>::ones() {
    m[0] = (T)1;
    m[1] = (T)1;
    m[2] = (T)1;
    m[3] = (T)1;
  };
  template <class T>
  void Mat2x2<T>::identity() {
    m[0] = (T)1;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)1;
  };
  template <class T>
  void Mat2x2<T>::set(T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
  };
  template <class T>
  void Mat2x2<T>::set(Mat2x2* data) {
    set(data->m);
  };
  template <class T>
  void Mat2x2<T>::set(T data, int i, int j) {
    m[i*3+j] = data;
  };
  template <class T>
  void Mat2x2<T>::set(T s) {
    m[0] = s;
    m[1] = s;
    m[2] = s;
    m[3] = s;
  };

  // Math operations
  template <class T>
  void Mat2x2<T>::transpose() {
    T temp;
    temp = m[1];
    m[1] = m[2];
    m[2] = temp;
  };

  // Static Math operations
  template <class T>
  T Mat2x2<T>::det(Mat2x2* a) {
    return a->m[0]*a->m[3] - a->m[1]*a->m[2];
  };
  template <class T>
  T Mat2x2<T>::trace(Mat2x2* a) {  // Sum of elements on main diagonal
    return a->m[0] + a->m[3];
  };
  template <class T>
  void Mat2x2<T>::scale(Mat2x2* ret, Mat2x2* a, T s) {
    ret->m[0] = a->m[0] * s;
    ret->m[1] = a->m[1] * s;
    ret->m[2] = a->m[2] * s;
    ret->m[3] = a->m[3] * s;
  };
  template <class T>
  void Mat2x2<T>::sub(Mat2x2* ret, Mat2x2* a, Mat2x2* b) {
    ret->m[0] = a->m[0] - b->m[0];
    ret->m[1] = a->m[1] - b->m[1];
    ret->m[2] = a->m[2] - b->m[2];
    ret->m[3] = a->m[3] - b->m[3];
  };
  template <class T>
  void Mat2x2<T>::add(Mat2x2* ret, Mat2x2* a, Mat2x2* b) {
    ret->m[0] = a->m[0] + b->m[0];
    ret->m[1] = a->m[1] + b->m[1];
    ret->m[2] = a->m[2] + b->m[2];
    ret->m[3] = a->m[3] + b->m[3];
  };
  template <class T>
  void Mat2x2<T>::pairwiseMult(Mat2x2* ret, Mat2x2* a, Mat2x2* b) {
    ret->m[0] = a->m[0] * b->m[0];
    ret->m[1] = a->m[1] * b->m[1];
    ret->m[2] = a->m[2] * b->m[2];
    ret->m[3] = a->m[3] * b->m[3];
  };
  template <class T>
  void Mat2x2<T>::inverse(Mat2x2* ret, Mat2x2* a) {
    T det = a->det();
    ret->m[0] = a->m[3];
    ret->m[1] = -a->m[1];
    ret->m[2] = -a->m[2];
    ret->m[3] = a->m[0];
    ret->scale(1.0f/det);
  };
  template <class T>
  void Mat2x2<T>::mult(Mat2x2* ret, Mat2x2* a, Mat2x2* b) {
    ret->m[0] = a->m[0]*b->m[0] + a->m[1]*b->m[2];
    ret->m[1] = a->m[0]*b->m[1] + a->m[1]*b->m[3];
    ret->m[2] = a->m[2]*b->m[0] + a->m[3]*b->m[2];
    ret->m[3] = a->m[2]*b->m[1] + a->m[3]*b->m[3];
  };
  template <class T>
  void Mat2x2<T>::transpose(Mat2x2* ret, Mat2x2* a) {
    ret->m[0] = a->m[0];
    ret->m[1] = a->m[2];
    ret->m[2] = a->m[1];
    ret->m[3] = a->m[3];
  };
  template <class T>
  bool Mat2x2<T>::equal(Mat2x2* a, Mat2x2 *b) {
    for (int i = 0; i < 4; i ++)
      if (std::abs(a->m[i] - b->m[i]) > EPSILON)
        return false;
    return true;
  };

};  // end namespace math

#endif
