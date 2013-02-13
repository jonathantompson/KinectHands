//
//  mat4x4.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#ifndef MATH_MAT4X4_HEADER
#define MATH_MAT4X4_HEADER

#include <iomanip>
#include "alignment/data_align.h"
#include "math/math_base.h"
extern "C" {
  #include "math/decompose.h"
}

#ifndef PI_OVER_360
#define PI_OVER_360 0.0087266462599716478846184538424431
#endif

namespace math {
  // Note Matricies are all row-major:
  //            | m(0,0) m(0,1) m(0,2) m(0,3) |   |  0  1  2  3 |
  //   Mat4x4 = | m(1,0) m(1,1) m(1,2) m(1,3) | = |  4  5  6  7 |
  //            | m(2,0) m(2,1) m(2,2) m(2,3) |   |  8  9 10 11 |
  //            | m(3,0) m(3,1) m(3,2) m(3,3) |   | 12 13 14 15 |
  //      or column-major (COLUMN_MAJOR is defined):
  //            | m(0,0) m(1,0) m(2,0) m(3,0) |   | 0  4  8  12 |
  //   Mat2x2 = | m(0,1) m(1,1) m(2,1) m(3,1) | = | 1  5  9  13 |
  //            | m(0,2) m(1,2) m(2,2) m(3,2) |   | 2  6  10 14 |
  //            | m(0,3) m(1,3) m(2,3) m(3,3) |   | 3  7  11 15 |
  template <class T>
  class DATA_ALIGN(ALIGNMENT) Mat4x4 {
  public:
    Mat4x4();  // Default is identity
    explicit Mat4x4(T* data);
    explicit Mat4x4(Mat4x4* data);
    explicit Mat4x4(Mat3x3<T>* data);
    Mat4x4(T _00, T _01, T _02, T _03,
           T _10, T _11, T _12, T _13,
           T _20, T _21, T _22, T _23,
           T _30, T _31, T _32, T _33);
    
    // Getter methods
    void print();  // printf
    void printStdCout();  // Print to std::cout
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
    Mat4x4<T>& operator= (const Mat4x4<T>& other);  // O(n) - copy
    T& operator()(const int i_row, const int j_col);
    
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
    inline bool approxEqual(Mat4x4 *a) { return approxEqual(this, a); }
    void leftMultTranslation(T x, T y, T z);  // A = M_trans * A
    void leftMultScale(T x, T y, T z);  // A = M_scale * A
    void rightMultTranslation(T x, T y, T z);  // A = A * M_trans
    void rightMultScale(T x, T y, T z);  // A = A * M_scale
    inline void rotateMatXAxis(T angle) { rotateMatXAxis(this, angle); }
    inline void rotateMatYAxis(T angle) { rotateMatYAxis(this, angle); }
    inline void rotateMatZAxis(T angle) { rotateMatZAxis(this, angle); }
    inline void rotateMatAxisAngle(const Vec3<T>* axis, T angle);
    inline void scaleMat(T x, T y, T z);
    inline void translationMat(T x, T y, T z);
    inline void glProjection(T znear, T zfar, T fov,
                             T screen_width, T screen_height);
    inline void glOrthoProjection(T znear, T zfar, T left, T right,
                                  T bottom, T top);
    inline void glLookAt(const Vec3<T>* up, const Vec3<T>* forward,
                         const Vec3<T>* pos);
    inline void affineInverse(Mat4x4* a) { affineInverse(this, a); }
    inline void affineRotationTranslationInverse(Mat4x4* a);
    inline void affineRotationInverse(Mat4x4* a);
    // decompose Translation * Scale * Rot (IN THAT ORDER!), a is affine 
    // and invertible
    inline void decompose(Vec3<T>* translation, Mat4x4* rotation, Vec3<T>* scale);  
    inline void extractRotation(Mat4x4* rotation);
    inline void getTranslation(Vec3<T>* translation);
    inline void inverse();
    
    // Static Math operations
    static T det(Mat4x4* a);
    static T trace(Mat4x4* a);
    static void scale(Mat4x4* ret, Mat4x4* a, T s);  // ret = a*s
    static void sub(Mat4x4* ret, Mat4x4* a, Mat4x4* b);  // ret = a-b
    static void add(Mat4x4* ret, Mat4x4* a, Mat4x4* b);  // ret = a+b
    static void pairwiseMult(Mat4x4* ret, Mat4x4* a, Mat4x4* b);  // ret = a.*b
    static void inverse(Mat4x4* ret, Mat4x4* a);  // ret = a^-1
    static void mult(Mat4x4* ret, Mat4x4* a, Mat4x4* b);  // ret = a*b
    static void transpose(Mat4x4* ret, Mat4x4 *a);  // ret = a^t
    static bool equal(Mat4x4* a, Mat4x4 *b);
    static bool approxEqual(Mat4x4* a, Mat4x4 *b);
    static void rotateMatXAxis(Mat4x4* ret, T angle);
    static void rotateMatYAxis(Mat4x4* ret, T angle);
    static void rotateMatZAxis(Mat4x4* ret, T angle);
    static void rotateMatAxisAngle(Mat4x4* ret, const Vec3<T>* axis, T angle);
    static void scaleMat(Mat4x4* ret, T x, T y, T z);
    static void translationMat(Mat4x4* ret, T x, T y, T z);
    static void glProjection(Mat4x4* a, T znear, T zfar, T fov,
                             T screen_width, T screen_height);
    static void glOrthoProjection(Mat4x4* a, T znear, T zfar, T left, T right,
                                  T bottom, T top);
    static void glLookAt(Mat4x4* view, const Vec3<T>* up,
                         const Vec3<T>* forward, const Vec3<T>* pos);
    static void affineInverse(Mat4x4* ret, Mat4x4* a);
    static void affineRotationTranslationInverse(Mat4x4* ret, Mat4x4* a);
    static void affineRotationInverse(Mat4x4* ret, Mat4x4* a);
    // decompose into: Rot * Scale * Translation (IN THAT ORDER!), a is affine 
    // and invertible
    static void decompose(Mat4x4* a, Vec3<T>* translation, Mat4x4* rotation, 
      Vec3<T>* scale);  // Matrix must be affine and invertible!
    static void extractRotation(Mat4x4* a, Mat4x4* rotation);
    static void getTranslation(Mat4x4* a, Vec3<T>* translation); 
    static void euler2RotMat(Mat4x4* a, const float x_angle, 
      const float y_angle, const float z_angle);
    
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
  Mat4x4<T>::Mat4x4(Mat3x3<T>* data) {
    m[0] = data->m[0];
    m[1] = data->m[1];
    m[2] = data->m[2];
    m[3] = (T)0;
    m[4] = data->m[3];
    m[5] = data->m[4];
    m[6] = data->m[5];
    m[7] = (T)0;
    m[8] = data->m[6];
    m[9] = data->m[7];
    m[10] = data->m[8];
    m[11] = (T)0;
    m[12] = (T)0;
    m[13] = (T)0;
    m[14] = (T)0;
    m[15] = (T)1;
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
#ifdef ROW_MAJOR
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[0],  m[1],  m[2],  m[3]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[4],  m[5],  m[6],  m[7]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[8],  m[9],  m[10], m[11]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[12], m[13], m[14], m[15]);
#endif
#ifdef COLUMN_MAJOR
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[0],  m[4],  m[8],  m[12]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[1],  m[5],  m[9],  m[13]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[2],  m[6],  m[10], m[14]);
    printf("| %+.4e  %+.4e  %+.4e  %+.4e |\n", m[3],  m[7],  m[11], m[15]);
#endif
  };

  template <class T>
  void Mat4x4<T>::printStdCout() {
    std::cout.setf(std::ios::showpos);
    std::cout << std::setprecision(8);
    std::cout << std::fixed;
#ifdef ROW_MAJOR
    std::cout << "| " << m[0]  << " " << m[1]  << " " << m[2]  << " " << m[3]  << " |" << std::endl;
    std::cout << "| " << m[4]  << " " << m[5]  << " " << m[6]  << " " << m[7]  << " |" << std::endl;
    std::cout << "| " << m[8]  << " " << m[9]  << " " << m[10] << " " << m[11] << " |" << std::endl;
    std::cout << "| " << m[12] << " " << m[13] << " " << m[14] << " " << m[15] << " |" << std::endl;
#endif
#ifdef COLUMN_MAJOR
    std::cout << "| " << m[0]  << " " << m[4]  << " " << m[8]  << " " << m[12] << " |" << std::endl;
    std::cout << "| " << m[1]  << " " << m[5]  << " " << m[9]  << " " << m[13] << " |" << std::endl;
    std::cout << "| " << m[2]  << " " << m[6]  << " " << m[10] << " " << m[14] << " |" << std::endl;
    std::cout << "| " << m[3]  << " " << m[7]  << " " << m[11] << " " << m[15] << " |" << std::endl;
#endif
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

  template <class T>
  Mat4x4<T>& Mat4x4<T>::operator= (const Mat4x4<T>& other) {
    if (this != &other) {  // protect against invalid self-assignment
      m[0] = other.m[0];
      m[1] = other.m[1];
      m[2] = other.m[2];
      m[3] = other.m[3];
      m[4] = other.m[4];
      m[5] = other.m[5];
      m[6] = other.m[6];
      m[7] = other.m[7];
      m[8] = other.m[8];
      m[9] = other.m[9];
      m[10] = other.m[10];
      m[11] = other.m[11];
      m[12] = other.m[12];
      m[13] = other.m[13];
      m[14] = other.m[14];
      m[15] = other.m[15];
    }
    // by convention, always return *this
    return *this;
  }
  
  template <class T>
  T& Mat4x4<T>::operator()(const int i_row, const int j_col) {
#ifdef ROW_MAJOR
    return m[i_row * 4 + j_col];
#endif
#ifdef COLUMN_MAJOR
    return m[i_row + j_col * 4];
#endif
  }

  template <class T>
  void Mat4x4<T>::rightMultTranslation(T x, T y, T z) {
#ifdef ROW_MAJOR
    m[3] = m[3] + x*m[0] + y*m[1] + z*m[2];
    m[7] = m[7] + x*m[4] + y*m[5] + z*m[6];
    m[11] = m[11] + x*m[8] + y*m[9] + z*m[10];
#endif
#ifdef COLUMN_MAJOR
    m[12] = m[12] + x*m[0] + y*m[4] + z*m[8];
    m[13] = m[13] + x*m[1] + y*m[5] + z*m[9];
    m[14] = m[14] + x*m[2] + y*m[6] + z*m[10];
#endif
  };
  
  template <class T>
  void Mat4x4<T>::leftMultTranslation(T x, T y, T z) {
#ifdef ROW_MAJOR
    m[3] = m[3] + x;
    m[7] = m[7] + y;
    m[11] = m[11] + z;
#endif
#ifdef COLUMN_MAJOR
    m[12] = m[12] + x;
    m[13] = m[13] + y;
    m[14] = m[14] + z;
#endif
  };
  
  template <class T>
  void Mat4x4<T>::rightMultScale(T x, T y, T z) {
#ifdef ROW_MAJOR
    m[0] *= x;
    m[1] *= y;
    m[2] *= z;
    m[4] *= x;
    m[5] *= y;
    m[6] *= z;
    m[8] *= x;
    m[9] *= y;
    m[10] *= z;
    m[12] *= x;
    m[13] *= y;
    m[14] *= z;
#endif
#ifdef COLUMN_MAJOR
    m[0] *= x;
    m[1] *= x;
    m[2] *= x;
    m[3] *= x;
    m[4] *= y;
    m[5] *= y;
    m[6] *= y;
    m[7] *= y;
    m[8] *= z;
    m[9] *= z;
    m[10] *= z;
    m[11] *= z;
#endif
  };
  
  template <class T>
  void Mat4x4<T>::leftMultScale(T x, T y, T z) {
#ifdef ROW_MAJOR
    m[0] *= x;
    m[1] *= x;
    m[2] *= x;
    m[3] *= x;
    m[4] *= y;
    m[5] *= y;
    m[6] *= y;
    m[7] *= y;
    m[8] *= z;
    m[9] *= z;
    m[10] *= z;
    m[11] *= z;
#endif
#ifdef COLUMN_MAJOR
    m[0] *= x;
    m[1] *= y;
    m[2] *= z;
    m[4] *= x;
    m[5] *= y;
    m[6] *= z;
    m[8] *= x;
    m[9] *= y;
    m[10] *= z;
    m[12] *= x;
    m[13] *= y;
    m[14] *= z;
#endif
  };
  
  template <class T>
  void Mat4x4<T>::rotateMatAxisAngle(const Vec3<T>* axis, T angle) {
    rotateMatAxisAngle(this, axis, angle);
  };
  
  template <class T>
  void Mat4x4<T>::scaleMat(T x, T y, T z) {
    scaleMat(this, x, y, z);
  };
  
  template <class T>
  void Mat4x4<T>::translationMat(T x, T y, T z) {
    translationMat(this, x, y, z);
  };
  
  template <class T>
  void Mat4x4<T>::glProjection(T znear, T zfar, T fov,
                               T screen_width, T screen_height) {
    glProjection(this, znear, zfar, fov, screen_width, screen_height);
  };
  
  template <class T>
  void Mat4x4<T>::glOrthoProjection(T znear, T zfar,
                                    T left, T right, T bottom, T top) {
    glOrthoProjection(this, znear, zfar, left, right, bottom, top);
  };
  
  template <class T>
  void Mat4x4<T>::glLookAt(const Vec3<T>* up, const Vec3<T>* forward,
                           const Vec3<T>* pos) {
    glLookAt(this, up, forward, pos);
  };
  
  template <class T>
  void Mat4x4<T>::affineRotationTranslationInverse(Mat4x4* a) {
    affineRotationTranslationInverse(this, a);
  }
  
  template <class T>
  void Mat4x4<T>::affineRotationInverse(Mat4x4* a) {
    affineRotationInverse(this, a);
  }

  template <class T>
  void Mat4x4<T>::decompose(Vec3<T>* translation, Mat4x4* rotation, 
      Vec3<T>* scale) {
    decompose(this, translation, rotation, scale);
  }

  template <class T>
  void Mat4x4<T>::extractRotation(Mat4x4* rotation) {
    extractRotation(this, rotation);
  }

  template <class T>
  void Mat4x4<T>::getTranslation(Vec3<T>* translation) {
    getTranslation(this, translation);
  }

  template <class T>
  void Mat4x4<T>::inverse() {
    Mat4x4<T> temp;
    Mat4x4<T>::inverse(&temp, this);
    this->set(&temp);
  } 
  
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
  // det(A) = det(A^t) (so row and column major have the same determinent)
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
#ifdef ROW_MAJOR
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
    if (fabs(det) > EPSILON) {
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
#endif
#ifdef COLUMN_MAJOR
    T a0 = a->m[0]*a->m[5] - a->m[4]*a->m[1];
    T a1 = a->m[0]*a->m[9] - a->m[8]*a->m[1];
    T a2 = a->m[0]*a->m[13] - a->m[12]*a->m[1];
    T a3 = a->m[4]*a->m[9] - a->m[8]*a->m[5];
    T a4 = a->m[4]*a->m[13] - a->m[12]*a->m[5];
    T a5 = a->m[8]*a->m[13] - a->m[12]*a->m[9];
    T b0 = a->m[2]*a->m[7] - a->m[6]*a->m[3];
    T b1 = a->m[2]*a->m[11] - a->m[10]*a->m[3];
    T b2 = a->m[2]*a->m[15] - a->m[14]*a->m[3];
    T b3 = a->m[6]*a->m[11] - a->m[10]*a->m[7];
    T b4 = a->m[6]*a->m[15] - a->m[14]*a->m[7];
    T b5 = a->m[10]*a->m[15] - a->m[14]*a->m[11];
    
    T det = a0*b5 - a1*b4 + a2*b3 + a3*b2 - a4*b1 + a5*b0;
    if (fabs(det) > EPSILON) {
      ret->m[0] = + a->m[5]*b5 - a->m[9]*b4 + a->m[13]*b3;
      ret->m[1] = - a->m[1]*b5 + a->m[9]*b2 - a->m[13]*b1;
      ret->m[2] = + a->m[1]*b4 - a->m[5]*b2 + a->m[13]*b0;
      ret->m[3] = - a->m[1]*b3 + a->m[5]*b1 - a->m[9]*b0;
      ret->m[4] = - a->m[4]*b5 + a->m[8]*b4 - a->m[12]*b3;
      ret->m[5] = + a->m[0]*b5 - a->m[8]*b2 + a->m[12]*b1;
      ret->m[6] = - a->m[0]*b4 + a->m[4]*b2 - a->m[12]*b0;
      ret->m[7] = + a->m[0]*b3 - a->m[4]*b1 + a->m[8]*b0;
      ret->m[8] = + a->m[7]*a5 - a->m[11]*a4 + a->m[15]*a3;
      ret->m[9] = - a->m[3]*a5 + a->m[11]*a2 - a->m[15]*a1;
      ret->m[10] = + a->m[3]*a4 - a->m[7]*a2 + a->m[15]*a0;
      ret->m[11] = - a->m[3]*a3 + a->m[7]*a1 - a->m[11]*a0;
      ret->m[12] = - a->m[6]*a5 + a->m[10]*a4 - a->m[14]*a3;
      ret->m[13] = + a->m[2]*a5 - a->m[10]*a2 + a->m[14]*a1;
      ret->m[14] = - a->m[2]*a4 + a->m[6]*a2 - a->m[14]*a0;
      ret->m[15] = + a->m[2]*a3 - a->m[6]*a1 + a->m[10]*a0;
      
      T invDet = (T)1/det;
      ret->scale(invDet);
    } else {
      ret->zeros();
    }
#endif
  };
  
  template <class T>
  void Mat4x4<T>::mult(Mat4x4* ret, Mat4x4* a, Mat4x4* b) {
#ifdef ROW_MAJOR
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
#endif
#ifdef COLUMN_MAJOR
    ret->m[0] = a->m[0]*b->m[0] + a->m[4]*b->m[1] + a->m[8]*b->m[2] +
    a->m[12]*b->m[3];
    ret->m[4] = a->m[0]*b->m[4] + a->m[4]*b->m[5] + a->m[8]*b->m[6] +
    a->m[12]*b->m[7];
    ret->m[8] = a->m[0]*b->m[8] + a->m[4]*b->m[9] + a->m[8]*b->m[10] +
    a->m[12]*b->m[11];
    ret->m[12] = a->m[0]*b->m[12] + a->m[4]*b->m[13] + a->m[8]*b->m[14] +
    a->m[12]*b->m[15];
    ret->m[1] = a->m[1]*b->m[0] + a->m[5]*b->m[1] + a->m[9]*b->m[2] +
    a->m[13]*b->m[3];
    ret->m[5] = a->m[1]*b->m[4] + a->m[5]*b->m[5] + a->m[9]*b->m[6] +
    a->m[13]*b->m[7];
    ret->m[9] = a->m[1]*b->m[8] + a->m[5]*b->m[9] + a->m[9]*b->m[10] +
    a->m[13]*b->m[11];
    ret->m[13] = a->m[1]*b->m[12] + a->m[5]*b->m[13] + a->m[9]*b->m[14] +
    a->m[13]*b->m[15];
    ret->m[2] = a->m[2]*b->m[0] + a->m[6]*b->m[1] + a->m[10]*b->m[2] +
    a->m[14]*b->m[3];
    ret->m[6] = a->m[2]*b->m[4] + a->m[6]*b->m[5] + a->m[10]*b->m[6] +
    a->m[14]*b->m[7];
    ret->m[10] = a->m[2]*b->m[8] + a->m[6]*b->m[9] + a->m[10]*b->m[10] +
    a->m[14]*b->m[11];
    ret->m[14] = a->m[2]*b->m[12] + a->m[6]*b->m[13] + a->m[10]*b->m[14] +
    a->m[14]*b->m[15];
    ret->m[3] = a->m[3]*b->m[0] + a->m[7]*b->m[1] + a->m[11]*b->m[2] +
    a->m[15]*b->m[3];
    ret->m[7] = a->m[3]*b->m[4] + a->m[7]*b->m[5] + a->m[11]*b->m[6] +
    a->m[15]*b->m[7];
    ret->m[11] = a->m[3]*b->m[8] + a->m[7]*b->m[9] + a->m[11]*b->m[10] +
    a->m[15]*b->m[11];
    ret->m[15] = a->m[3]*b->m[12] + a->m[7]*b->m[13] + a->m[11]*b->m[14] +
    a->m[15]*b->m[15];
#endif
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

  template <class T>
  bool Mat4x4<T>::approxEqual(Mat4x4* a, Mat4x4 *b) {
    for (int i = 0; i < 16; i ++)
      if (abs(a->m[i] - b->m[i]) > LOOSE_EPSILON)
        return false;
    return true;
  };
  
  template <class T>
  void Mat4x4<T>::rotateMatXAxis(Mat4x4* ret, T angle) {
    T cos_angle = cos(angle);
    T sin_angle = sin(angle);
#ifdef ROW_MAJOR
    ret->m[0] = (T)1;
    ret->m[1] = (T)0;
    ret->m[2] = (T)0;
    ret->m[3] = (T)0;
    ret->m[4] = (T)0;
    ret->m[5] = cos_angle;
    ret->m[6] = -sin_angle;
    ret->m[7] = (T)0;
    ret->m[8] = (T)0;
    ret->m[9] = sin_angle;
    ret->m[10] = cos_angle;
    ret->m[11] = (T)0;
    ret->m[12] = (T)0;
    ret->m[13] = (T)0;
    ret->m[14] = (T)0;
    ret->m[15] = (T)1;
#endif
#ifdef COLUMN_MAJOR
    ret->m[0] = (T)1;
    ret->m[1] = (T)0;
    ret->m[2] = (T)0;
    ret->m[3] = (T)0;
    ret->m[4] = (T)0;
    ret->m[5] = cos_angle;
    ret->m[6] = sin_angle;
    ret->m[7] = (T)0;
    ret->m[8] = (T)0;
    ret->m[9] = -sin_angle;
    ret->m[10] = cos_angle;
    ret->m[11] = (T)0;
    ret->m[12] = (T)0;
    ret->m[13] = (T)0;
    ret->m[14] = (T)0;
    ret->m[15] = (T)1;
#endif
  };
  
  template <class T>
  void Mat4x4<T>::rotateMatYAxis(Mat4x4* ret, T angle) {
    T cos_angle = cos(angle);
    T sin_angle = sin(angle);
#ifdef ROW_MAJOR
    ret->m[0] = cos_angle;
    ret->m[1] = (T)0;
    ret->m[2] = sin_angle;
    ret->m[3] = (T)0;
    ret->m[4] = (T)0;
    ret->m[5] = (T)1;
    ret->m[6] = (T)0;
    ret->m[7] = (T)0;
    ret->m[8] = -sin_angle;
    ret->m[9] = (T)0;
    ret->m[10] = cos_angle;
    ret->m[11] = (T)0;
    ret->m[12] = (T)0;
    ret->m[13] = (T)0;
    ret->m[14] = (T)0;
    ret->m[15] = (T)1;
#endif
#ifdef COLUMN_MAJOR
    ret->m[0] = cos_angle;
    ret->m[1] = (T)0;
    ret->m[2] = -sin_angle;
    ret->m[3] = (T)0;
    ret->m[4] = (T)0;
    ret->m[5] = (T)1;
    ret->m[6] = (T)0;
    ret->m[7] = (T)0;
    ret->m[8] = sin_angle;
    ret->m[9] = (T)0;
    ret->m[10] = cos_angle;
    ret->m[11] = (T)0;
    ret->m[12] = (T)0;
    ret->m[13] = (T)0;
    ret->m[14] = (T)0;
    ret->m[15] = (T)1;
#endif
  };
  
  template <class T>
  void Mat4x4<T>::rotateMatZAxis(Mat4x4* ret, T angle) {
    T cos_angle = cos(angle);
    T sin_angle = sin(angle);
#ifdef ROW_MAJOR
    ret->m[0] = cos_angle;
    ret->m[1] = -sin_angle;
    ret->m[2] = (T)0;
    ret->m[3] = (T)0;
    ret->m[4] = sin_angle;
    ret->m[5] = cos_angle;
    ret->m[6] = (T)0;
    ret->m[7] = (T)0;
    ret->m[8] = (T)0;
    ret->m[9] = (T)0;
    ret->m[10] = (T)1;
    ret->m[11] = (T)0;
    ret->m[12] = (T)0;
    ret->m[13] = (T)0;
    ret->m[14] = (T)0;
    ret->m[15] = (T)1;
#endif
#ifdef COLUMN_MAJOR
    ret->m[0] = cos_angle;
    ret->m[1] = sin_angle;
    ret->m[2] = (T)0;
    ret->m[3] = (T)0;
    ret->m[4] = -sin_angle;
    ret->m[5] = cos_angle;
    ret->m[6] = (T)0;
    ret->m[7] = (T)0;
    ret->m[8] = (T)0;
    ret->m[9] = (T)0;
    ret->m[10] = (T)1;
    ret->m[11] = (T)0;
    ret->m[12] = (T)0;
    ret->m[13] = (T)0;
    ret->m[14] = (T)0;
    ret->m[15] = (T)1;
#endif
  };
  
  template <class T>
  void Mat4x4<T>::rotateMatAxisAngle(Mat4x4* ret, const Vec3<T>* axis,
                                     T angle) {
    // http://en.wikipedia.org/wiki/Rotation_matrix
    Vec3<T> axis_(const_cast<Vec3<T>*>(axis));
    axis_.normalize();  // just in case
    T xy = axis_.m[0] * axis_.m[1];
    T xz = axis_.m[0] * axis_.m[2];
    T yz = axis_.m[1] * axis_.m[2];
    T cos_angle = cos(angle);
    T one_minus_cos_angle = 1 - cos(angle);
    T sin_angle = sin(angle);
#ifdef ROW_MAJOR
    ret->m[0] = cos_angle + axis_.m[0]*axis_.m[0]*one_minus_cos_angle;
    ret->m[4] = xy*one_minus_cos_angle + axis_.m[2]*sin_angle;
    ret->m[8] = xz*one_minus_cos_angle - axis_.m[1]*sin_angle;
    ret->m[12] = (T)0;
    ret->m[1] = xy*one_minus_cos_angle - axis_.m[2]*sin_angle;
    ret->m[5] = cos_angle + axis_.m[1]*axis_.m[1]*one_minus_cos_angle;
    ret->m[9] = yz*one_minus_cos_angle + axis_.m[0]*sin_angle;
    ret->m[13] = (T)0;
    ret->m[2] = xz*one_minus_cos_angle + axis_.m[1]*sin_angle;
    ret->m[6] = yz*one_minus_cos_angle - axis_.m[0]*sin_angle;
    ret->m[10] = cos_angle + axis_.m[2]*axis_.m[2]*one_minus_cos_angle;
    ret->m[14] = (T)0;
    ret->m[3] = (T)0;
    ret->m[7] = (T)0;
    ret->m[11] = (T)0;
    ret->m[15] = (T)1;
#endif
#ifdef COLUMN_MAJOR
    ret->m[0] = cos_angle + axis_.m[0]*axis_.m[0]*one_minus_cos_angle;
    ret->m[1] = xy*one_minus_cos_angle + axis_.m[2]*sin_angle;
    ret->m[2] = xz*one_minus_cos_angle - axis_.m[1]*sin_angle;
    ret->m[3] = (T)0;
    ret->m[4] = xy*one_minus_cos_angle - axis_.m[2]*sin_angle;
    ret->m[5] = cos_angle + axis_.m[1]*axis_.m[1]*one_minus_cos_angle;
    ret->m[6] = yz*one_minus_cos_angle + axis_.m[0]*sin_angle;
    ret->m[7] = (T)0;
    ret->m[8] = xz*one_minus_cos_angle + axis_.m[1]*sin_angle;
    ret->m[9] = yz*one_minus_cos_angle - axis_.m[0]*sin_angle;
    ret->m[10] = cos_angle + axis_.m[2]*axis_.m[2]*one_minus_cos_angle;
    ret->m[11] = (T)0;
    ret->m[12] = (T)0;
    ret->m[13] = (T)0;
    ret->m[14] = (T)0;
    ret->m[15] = (T)1;
#endif
  }
  
  template <class T>
  void Mat4x4<T>::glProjection(Mat4x4* a, T znear, T zfar, T fov,
                               T screen_width, T screen_height) {
    T aspect = screen_width / screen_height;
    
    // Calculate the projection matrix, from:
    // http://www.geeks3d.com/20090729/howto-perspective-projection-matrix-in-opengl/
    T xymax = znear * tanf(fov * static_cast<T>(PI_OVER_360));
    T ymin = -xymax;
    T xmin = -xymax;
    
    T width = xymax - xmin;
    T height = xymax - ymin;
    
    T depth = zfar - znear;
    T q = -(zfar + znear) / depth;
    T qn = -2 * (zfar * znear) / depth;
    
    T w = 2 * znear / width;
    w = w / aspect;
    T h = 2 * znear / height;
    
    a->m[0]  = w;
    a->m[1]  = 0;
    a->m[2]  = 0;
    a->m[3]  = 0;
    
    a->m[4]  = 0;
    a->m[5]  = h;
    a->m[6]  = 0;
    a->m[7]  = 0;
    
    a->m[8]  = 0;
    a->m[9]  = 0;
    a->m[10] = q;
    a->m[11] = -1;
    
    a->m[12] = 0;
    a->m[13] = 0;
    a->m[14] = qn;
    a->m[15] = 0;
  };
  
  template <class T>
  void Mat4x4<T>::glOrthoProjection(Mat4x4* a, T znear, T zfar,
                                    T left, T right, T bottom, T top) {
    T tx = -(right + left) / (right - left);
    T ty = -(top + bottom) / (top - bottom);
    T tz = -(zfar + znear) / (zfar - znear);
    
    a->m[0] = (T) (2 / (right - left));
    a->m[1]  = 0;
    a->m[2]  = 0;
    a->m[3]  = 0;
    
    a->m[4]  = 0;
    a->m[5] = (T) (2 / (top - bottom));
    a->m[6]  = 0;
    a->m[7]  = 0;
    
    a->m[8]  = 0;
    a->m[9]  = 0;
    a->m[10] = (T) (-2 / (zfar - znear));
    a->m[11] = 0;
    
    a->m[12] = (T) tx;
    a->m[13] = (T) ty;
    a->m[14] = (T) tz;
    a->m[15] = (T) 1;
  };
  
  template <class T>
  void Mat4x4<T>::glLookAt(Mat4x4* view, const Vec3<T>* up,
                           const Vec3<T>* forward, const Vec3<T>* pos) {
    Vec3<T> side_;
    Vec3<T> up_(const_cast<Vec3<T>*>(up));
    Vec3<T> forward_(const_cast<Vec3<T>*>(forward));
    up_.normalize();  // Just in case
    forward_.normalize();  // Just in case
    
    // Create orthonormal basis
    Vec3<T>::cross(&side_, &forward_, &up_);
    side_.normalize();
    Vec3<T>::cross(&up_, &side_, &forward_);  // Cross again to ensure orthonorm
    up_.normalize();
    
    // View matrix is an inverse rotation matrix (therefore axis vectors are
    // the rows).  OpenGL is column major, so (0, 4, 8) is the first row.
    // Recal inverse of an orthonormal matrix is just the transpose.
    view->m[0] = side_.m[0];
    view->m[4] = side_.m[1];
    view->m[8] = side_.m[2];
    view->m[12] = -pos->m[0];
    
    view->m[1] = up_.m[0];
    view->m[5] = up_.m[1];
    view->m[9] = up_.m[2];
    view->m[13] = -pos->m[1];
    
    view->m[2] = -forward_.m[0];
    view->m[6] = -forward_.m[1];
    view->m[10] = -forward_.m[2];
    view->m[14] = -pos->m[2];
    
    view->m[3] = 0.0f;
    view->m[7] = 0.0f;
    view->m[11] = 0.0f;
    view->m[15] = 1.0f;
  };
  
  template <class T>
  void Mat4x4<T>::affineInverse(Mat4x4* ret, Mat4x4* a) {
    T det = a->m[0] * (a->m[10] * a->m[5] - a->m[9] * a->m[6])
    - a->m[4] * (a->m[10] * a->m[1] - a->m[9] * a->m[2])
    + a->m[8] * (a->m[6] * a->m[1] - a->m[5] * a->m[2]);
    T one_over_det = T(1) / det;
    // First calculate inverse of upper left
#ifdef ROW_MAJOR
    ret->m[0] = (a->m[10]*a->m[5]-a->m[9]*a->m[6]) * one_over_det;
    ret->m[1] = (-(a->m[10]*a->m[1]-a->m[9]*a->m[2])) * one_over_det;
    ret->m[2] = (a->m[6]*a->m[1]-a->m[5]*a->m[2]) * one_over_det;
    ret->m[4] = (-(a->m[10]*a->m[4]-a->m[8]*a->m[6])) * one_over_det;
    ret->m[5] = (a->m[10]*a->m[0]-a->m[8]*a->m[2]) * one_over_det;
    ret->m[6] = (-(a->m[6]*a->m[0]-a->m[4]*a->m[2])) * one_over_det;
    ret->m[8] = (a->m[9]*a->m[4]-a->m[8]*a->m[5]) * one_over_det;
    ret->m[9] = (-(a->m[9]*a->m[0]-a->m[8]*a->m[1])) * one_over_det;
    ret->m[10] = (a->m[5]*a->m[0]-a->m[4]*a->m[1]) * one_over_det;
#endif
#ifdef COLUMN_MAJOR
    ret->m[0] = (a->m[10]*a->m[5]-a->m[6]*a->m[9]) * one_over_det;
    ret->m[1] = (-(a->m[10]*a->m[1]-a->m[2]*a->m[9])) * one_over_det;
    ret->m[2] = (a->m[6]*a->m[1]-a->m[2]*a->m[5]) * one_over_det;
    ret->m[4] = (-(a->m[10]*a->m[4]-a->m[6]*a->m[8])) * one_over_det;
    ret->m[5] = (a->m[10]*a->m[0]-a->m[2]*a->m[8]) * one_over_det;
    ret->m[6] = (-(a->m[6]*a->m[0]-a->m[2]*a->m[4])) * one_over_det;
    ret->m[8] = (a->m[9]*a->m[4]-a->m[5]*a->m[8]) * one_over_det;
    ret->m[9] = (-(a->m[9]*a->m[0]-a->m[1]*a->m[8])) * one_over_det;
    ret->m[10] = (a->m[5]*a->m[0]-a->m[1]*a->m[4]) * one_over_det;
#endif
    // Now calculate the -M^inv * T
#ifdef ROW_MAJOR
    ret->m[3] = -(ret->m[0]*a->m[3] + ret->m[1]*a->m[7] + ret->m[2]*a->m[11]);
    ret->m[7] = -(ret->m[4]*a->m[3] + ret->m[5]*a->m[7] + ret->m[6]*a->m[11]);
    ret->m[11] = -(ret->m[8]*a->m[3] + ret->m[9]*a->m[7] + ret->m[10]*a->m[11]);
    ret->m[12] = (T)0;
    ret->m[13] = (T)0;
    ret->m[14] = (T)0;
#endif
#ifdef COLUMN_MAJOR
    ret->m[3] = (T)0;
    ret->m[7] = (T)0;
    ret->m[11] = (T)0;
    ret->m[12] = -(ret->m[0]*a->m[12] + ret->m[4]*a->m[13] +
                   ret->m[8]*a->m[14]);
    ret->m[13] = -(ret->m[1]*a->m[12] + ret->m[5]*a->m[13] +
                   ret->m[9]*a->m[14]);
    ret->m[14] = -(ret->m[2]*a->m[12] + ret->m[6]*a->m[13] +
                   ret->m[10]*a->m[14]);
#endif
    ret->m[15] = (T)1;
  };
  
  template <class T>
  void Mat4x4<T>::scaleMat(Mat4x4* ret, T x, T y, T z) {
    ret->m[0] = x;
    ret->m[1] = (T)0;
    ret->m[2] = (T)0;
    ret->m[3] = (T)0;
    ret->m[4] = (T)0;
    ret->m[5] = y;
    ret->m[6] = (T)0;
    ret->m[7] = (T)0;
    ret->m[8] = (T)0;
    ret->m[9] = (T)0;
    ret->m[10] = z;
    ret->m[11] = (T)0;
    ret->m[12] = (T)0;
    ret->m[13] = (T)0;
    ret->m[14] = (T)0;
    ret->m[15] = (T)1;
  };
  
  template <class T>
  void Mat4x4<T>::translationMat(Mat4x4* ret, T x, T y, T z) {
    ret->m[0] = (T)1;
    ret->m[1] = (T)0;
    ret->m[2] = (T)0;
    ret->m[4] = (T)0;
    ret->m[5] = (T)1;
    ret->m[6] = (T)0;
    ret->m[8] = (T)0;
    ret->m[9] = (T)0;
    ret->m[10] = (T)1;
#ifdef ROW_MAJOR
    ret->m[3] = x;
    ret->m[7] = y;
    ret->m[11] = z;
    ret->m[12] = (T)0;
    ret->m[13] = (T)0;
    ret->m[14] = (T)0;
#endif
#ifdef COLUMN_MAJOR
    ret->m[3] = (T)0;
    ret->m[7] = (T)0;
    ret->m[11] = (T)0;
    ret->m[12] = x;
    ret->m[13] = y;
    ret->m[14] = z;
#endif
    ret->m[15] = (T)1;
  };
  
  // Faster than affineInverse, but only for composite rotation + translation
  // matrices
  template <class T>
  void Mat4x4<T>::affineRotationTranslationInverse(Mat4x4* ret, Mat4x4* a) {
    // Inverse of the upper left rotation matrix is just it's transpose since it
    // is orthonormal
    ret->m[0] = a->m[0];
    ret->m[1] = a->m[4];
    ret->m[2] = a->m[8];
    ret->m[4] = a->m[1];
    ret->m[5] = a->m[5];
    ret->m[6] = a->m[9];
    ret->m[8] = a->m[2];
    ret->m[9] = a->m[6];
    ret->m[10] = a->m[10];
    // Now calculate the -M^inv * T
#ifdef ROW_MAJOR
    ret->m[3] = -(ret->m[0]*a->m[3] + ret->m[1]*a->m[7] + ret->m[2]*a->m[11]);
    ret->m[7] = -(ret->m[4]*a->m[3] + ret->m[5]*a->m[7] + ret->m[6]*a->m[11]);
    ret->m[11] = -(ret->m[8]*a->m[3] + ret->m[9]*a->m[7] + ret->m[10]*a->m[11]);
    ret->m[12] = (T)0;
    ret->m[13] = (T)0;
    ret->m[14] = (T)0;
#endif
#ifdef COLUMN_MAJOR
    ret->m[3] = (T)0;
    ret->m[7] = (T)0;
    ret->m[11] = (T)0;
    ret->m[12] = -(ret->m[0]*a->m[12] + ret->m[4]*a->m[13] +
                   ret->m[8]*a->m[14]);
    ret->m[13] = -(ret->m[1]*a->m[12] + ret->m[5]*a->m[13] +
                   ret->m[9]*a->m[14]);
    ret->m[14] = -(ret->m[2]*a->m[12] + ret->m[6]*a->m[13] +
                   ret->m[10]*a->m[14]);
#endif
    ret->m[15] = (T)1;
  };
  
  // Faster than affineInverse, but only for composite rotation matrices
  template <class T>
  void Mat4x4<T>::affineRotationInverse(Mat4x4* ret, Mat4x4* a) {
    // Inverse of the upper left rotation matrix is just it's transpose since it
    // is orthonormal
    ret->m[0] = a->m[0];
    ret->m[1] = a->m[4];
    ret->m[2] = a->m[8];
    ret->m[3] = (T)0;
    ret->m[4] = a->m[1];
    ret->m[5] = a->m[5];
    ret->m[6] = a->m[9];
    ret->m[7] = (T)0;
    ret->m[8] = a->m[2];
    ret->m[9] = a->m[6];
    ret->m[10] = a->m[10];
    ret->m[11] = (T)0;
    ret->m[12] = (T)0;
    ret->m[13] = (T)0;
    ret->m[14] = (T)0;
    ret->m[15] = (T)1;
  };

  template <class T>
  void Mat4x4<T>::extractRotation(Mat4x4* a, Mat4x4* rotation) {
    // Note "a" must be affine and invertible!
    // Uses polar decomposition to find an approximate rotation matrix:
    // http://callumhay.blogspot.com/2010/10/decomposing-affine-transforms.html
    // Also see: "Matrix Animation and Polar Decomposition", by Ken Shoemake 
    // and Tom Duff
    Mat4x4<T> Q;
    Mat4x4<T> S;
    PolarDecomposition(a, &Q, &S);

    // Rotation matrix is the Q matrix:
    rotation->set(&Q);
  }

  template <class T>
  void Mat4x4<T>::getTranslation(Mat4x4* a, Vec3<T>* translation) {
#ifdef ROW_MAJOR
    translation->m[0] = a->m[3];
    translation->m[1] = a->m[7];
    translation->m[2] = a->m[11];
#endif
#ifdef COLUMN_MAJOR
    translation->m[0] = a->m[12];
    translation->m[1] = a->m[13];
    translation->m[2] = a->m[14];
#endif
  };

  template <class T>
  void Mat4x4<T>::decompose(Mat4x4* a, Vec3<T>* translation, Mat4x4* rotation, 
      Vec3<T>* scale) {
    AffineParts parts;
    HMatrix A;
    for (uint32_t r = 0; r < 4; r++) { 
      for (uint32_t c = 0; c < 4; c++) {
        A[r][c] = (float)(*a)(r, c);
      }
    }
    decomp_affine(A, &parts);
    translation->set(parts.t.x, parts.t.y, parts.t.z);
    scale->set(parts.k.x, parts.k.y, parts.k.z);
    for (uint32_t r = 0; r < 3; r++) { 
      for (uint32_t c = 0; c < 3; c++) {
        (*rotation)(r, c) = parts.q_mat[r][c];
      }
    }
    rotation->m[3] = (T)0;
    rotation->m[7] = (T)0;
    rotation->m[11] = (T)0;
    rotation->m[12] = (T)0;
    rotation->m[13] = (T)0;
    rotation->m[14] = (T)0;
    rotation->m[15] = (T)1;
  }  

  template <class T>
  void Mat4x4<T>::euler2RotMat(Mat4x4* a, const float x_angle, 
    const float y_angle, const float z_angle) {
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
    T c1 = cos(x_angle);
    T s1 = sin(x_angle);
    T c2 = cos(y_angle);
    T s2 = sin(y_angle);
    T c3 = cos(z_angle);
    T s3 = sin(z_angle);
#ifdef COLUMN_MAJOR
    a->m[0] = c1*c2;
    a->m[4] = -c1*s2*c3 + s1*s3;
    a->m[8] = c1*s2*s3 + s1*c3;
    a->m[12] = (T)0;
    a->m[1] = s2;
    a->m[5] = c2*c3;
    a->m[9] = -c2*s3;
    a->m[13] = (T)0;
    a->m[2] = -s1*c2;
    a->m[6] = s1*s2*c3 + c1*s3;
    a->m[10] = -s1*s2*s3 + c1*c3;
    a->m[14] = (T)0;
    a->m[3] = (T)0;
    a->m[7] = (T)0;
    a->m[11] = (T)0;
    a->m[15] = (T)1;
#endif
#ifdef ROW_MAJOR
    a->m[0] = c1*c2;
    a->m[1] = -c1*s2*c3 + s1*s3;
    a->m[2] = c1*s2*s3 + s1*c3;
    a->m[3] = (T)0;
    a->m[4] = s2;
    a->m[5] = c2*c3;
    a->m[6] = -c2*s3;
    a->m[7] = (T)0;
    a->m[8] = -s1*c2;
    a->m[9] = s1*s2*c3 + c1*s3;
    a->m[10] = -s1*s2*s3 + c1*c3;
    a->m[11] = (T)0;
    a->m[12] = (T)0;
    a->m[13] = (T)0;
    a->m[14] = (T)0;
    a->m[15] = (T)1;
#endif
  }
};  // end namespace math

#endif
