//
//  quaternion.h
//
//  Created by Jonathan Tompson on 3/14/12.
//

#ifndef MATH_QUATERNION_HEADER
#define MATH_QUATERNION_HEADER

#include "math_types.h"
extern "C" {
  #include "math/decompose.h"
}

#ifndef M_PI
  #define M_PI    3.14159265358979323846
#endif

namespace math {
  template <class T>
  class DATA_ALIGN(ALIGNMENT) Quat {
  public:
    Quat();
    explicit Quat(T* data);
    explicit Quat(Quat* data);
    Quat(T x, T y, T z, T w);
    Quat(Vec3<T>* axis, T angle);
    Quat(Mat3x3<T>* mat);
    Quat(Mat4x4<T>* mat);

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
    void set(Quat* data);

    // Math operations
    inline void axisAngle2Quat(Vec3<T>* axis, T angle);
    inline void mat3x32Quat(Mat3x3<T>* mat);
    inline void mat4x42Quat(Mat4x4<T>* mat);
    inline void quat2AxisAngle(Vec3<T>* axis, T* angle);
    inline void quat2Mat3x3(Mat3x3<T>* mat);
    inline void quat2Mat4x4(Mat4x4<T>* mat);
    inline void mult(Quat* a, Quat* b);
    inline void scale(T s);
    inline void inverse(Quat* a);
    inline void inverse();
    inline void normalize(Quat* a);
    inline void normalize();
    inline T length();
    inline void mult(Vec3<T>* ret, Vec3<T>* b);
    inline T dot(Quat* b);
    inline void sub(Quat* a, Quat* b);
    inline void add(Quat* a, Quat* b);
    inline void slerp(Quat* a, Quat* b, T interp);
    inline bool equal(Quat* a);
    inline bool approxEqual(Quat* a);
    inline void eulerAngles2Quat(T r_xaxis, T r_yaxis, T r_zaxis);
    inline void quat2EulerAngles(T& r_xaxis, T& r_yaxis, T& r_zaxis);
    inline void xAxisRotation(T angle);
    inline void yAxisRotation(T angle);
    inline void zAxisRotation(T angle);
    inline void identity();

    // Static Math operations
    static void axisAngle2Quat(Quat* quat, Vec3<T>* axis, T angle);
    static void mat3x32Quat(Quat* quat, Mat3x3<T>* mat);
    static void mat4x42Quat(Quat* quat, Mat4x4<T>* mat);
    static void quat2AxisAngle(Quat* quat, Vec3<T>* axis, T* angle);
    static void quat2Mat3x3(Quat* quat, Mat3x3<T>* mat);
    static void quat2Mat4x4(Quat* quat, Mat4x4<T>* mat);
    static void mult(Quat* ret, Quat* a, Quat* b);
    static void scale(Quat* a, T s);
    static void inverse(Quat* a, Quat* b);
    static void normalize(Quat* ret, Quat* a);
    static T length(Quat* a);
    static void mult(Vec3<T>* ret, Quat* a, Vec3<T>* b);
    static T dot(Quat* a, Quat* b);
    static void sub(Quat* ret, Quat* a, Quat* b);
    static void add(Quat* ret, Quat* a, Quat* b);
    static void slerp(Quat* ret, Quat* a, Quat* b, T interp);
    static bool equal(Quat* a, Quat* b);
    static bool approxEqual(Quat* a, Quat* b);
    static void eulerAngles2Quat(Quat* quat, T r_xaxis, T r_yaxis, T r_zaxis);
    static void quat2EulerAngles(Quat* quat, T& r_xaxis, T& r_yaxis, T& r_zaxis);
    static void xAxisRotation(Quat* ret, T angle);
    static void yAxisRotation(Quat* ret, T angle);
    static void zAxisRotation(Quat* ret, T angle);
    static void identity(Quat* a);
    // decompose Translation * Scale * Rot (IN THAT ORDER!), a is affine 
    // and invertible
    static void decompose(Mat4x4<T>* affine_mat, Vec3<T>* translation, 
      Quat* rotation, Vec3<T>* scale); 
    static void decompose(Mat4x4<T>* affine_mat, Vec3<T>* translation, 
      Quat* rotation);

    // Dual quaternion conversion for linear blend skinning:
    // Explained here: http://isg.cs.tcd.ie/projects/DualQuaternions/
    static void quatTrans2UnitDualQuat(Quat* q, Vec3<T>* t, T udq[2][4]);
    static void unitDualQuat2QuatTrans(T udq[2][4], Quat* q, Vec3<T>* t);
    // This version is for non-zero dual part
    static void dualQuat2QuatTrans(T dq[2][4], Quat* q, Vec3<T>* t); 

    T m[4];  // Not private --> Avoid some overhead with getter setter methods
  };

  // Quat Constructors
  template <class T>
  Quat<T>::Quat() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)1;
  };

  template <class T>
  Quat<T>::Quat(T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
  };

  template <class T>
  Quat<T>::Quat(Quat* data) {
    m[0] = data->m[0];
    m[1] = data->m[1];
    m[2] = data->m[2];
    m[3] = data->m[3];
  };

  template <class T>
  Quat<T>::Quat(T x, T y, T z, T w) {
    m[0] = x;
    m[1] = y;
    m[2] = z;
    m[3] = w;
  };

  template <class T>
  Quat<T>::Quat(Vec3<T>* axis, T angle) {
    axisAngle2Quat(this, axis, angle);
  };

  template <class T>
  Quat<T>::Quat(Mat3x3<T>* mat) {
    mat3x32Quat(this, mat);
  };

  template <class T>
  Quat<T>::Quat(Mat4x4<T>* mat) {
    mat4x42Quat(this, mat);
  };

  // Setter methods
  template <class T>
  void Quat<T>::zeros() {
    m[0] = (T)0;
    m[1] = (T)0;
    m[2] = (T)0;
    m[3] = (T)0;
  };

  template <class T>
  void Quat<T>::ones() {
    m[0] = (T)1;
    m[1] = (T)1;
    m[2] = (T)1;
    m[3] = (T)1;
  };

  template <class T>
  void Quat<T>::set(T x, T y, T z, T w) {
    m[0] = x;
    m[1] = y;
    m[2] = z;
    m[3] = w;
  };

  template <class T>
  void Quat<T>::set(T* data) {
    m[0] = data[0];
    m[1] = data[1];
    m[2] = data[2];
    m[3] = data[3];
  };

  template <class T>
  void Quat<T>::set(Quat* data) {
    this->set(data->m);
  };

  // Getter Methods
  template <class T>
  void Quat<T>::print() {
    printf("| %+.4e |\n| %+.4e |\n| %+.4e |\n| %+.4e |\n", 
      m[0], m[1], m[2], m[3]);
  };

  // Math operations
  template <class T>
  void Quat<T>::axisAngle2Quat(Vec3<T>* axis, T angle) {
    axisAngle2Quat(this, axis, angle);
  };

  template <class T>
  void Quat<T>::mat3x32Quat(Mat3x3<T>* mat) {
    mat3x32Quat(this, mat);
  };

  template <class T>
  void Quat<T>::mat4x42Quat(Mat4x4<T>* mat) {
    mat4x42Quat(this, mat);
  };

  template <class T>
  void Quat<T>::quat2AxisAngle(Vec3<T>* axis, T* angle) {
    quat2AxisAngle(this, axis, angle);
  };

  template <class T>
  void Quat<T>::quat2Mat3x3(Mat3x3<T>* mat) {
    quat2Mat3x3(this, mat);
  };

  template <class T>
  void Quat<T>::quat2Mat4x4(Mat4x4<T>* mat) {
    quat2Mat4x4(this, mat);
  };

  template <class T>
  void Quat<T>::mult(Quat* a, Quat* b) {
    mult(this, a, b);
  };

  template <class T>
  void Quat<T>::scale(T s) {
    scale(this, s);
  };

  template <class T>
  void Quat<T>::inverse() {
    inverse(this, this);
  };

  template <class T>
  void Quat<T>::inverse(Quat* a) {
    inverse(this, a);
  };

  template <class T>
  void Quat<T>::normalize() {
    normalize(this, this);
  };

  template <class T>
  void Quat<T>::normalize(Quat* a) {
    normalize(this, a);
  };

  template <class T>
  T Quat<T>::length() {
    return length(this);
  };

  template <class T>
  void Quat<T>::mult(Vec3<T>* ret, Vec3<T>* b) {
    mult(ret, this, b);
  };

  template <class T>
  T Quat<T>::dot(Quat<T>* b) {
    dot(this, b);
  };

  template <class T>
  void Quat<T>::sub(Quat<T>* a, Quat<T>* b) {
    sub(this, a, b);
  };

  template <class T>
  void Quat<T>::add(Quat<T>* a, Quat<T>* b) {
    sub(this, a, b);
  };

  template <class T>
  void Quat<T>::slerp(Quat* a, Quat* b, T interp ) {
    slerp(this, a, b, interp);
  };

  template <class T>
  bool Quat<T>::equal(Quat* a) {
    return equal(this, a);
  };

  template <class T>
  bool Quat<T>::approxEqual(Quat* a) {
    return approxEqual(this, a);
  };

  template <class T>
  void Quat<T>::eulerAngles2Quat(T r_xaxis, T r_yaxis, T r_zaxis) {
    eulerAngles2Quat(this, r_xaxis, r_yaxis, r_zaxis);
  }

  template <class T>
  void Quat<T>::quat2EulerAngles(T& r_xaxis, T& r_yaxis, T& r_zaxis) {
    quat2EulerAngles(this, r_xaxis, r_yaxis, r_zaxis);
  }

  template <class T>
  void Quat<T>::xAxisRotation(T angle) {
    xAxisRotation(this, angle);
  };

  template <class T>
  void Quat<T>::yAxisRotation(T angle) {
    yAxisRotation(this, angle);
  };

  template <class T>
  void Quat<T>::zAxisRotation(T angle) {
    zAxisRotation(this, angle);
  };

  template <class T>
  void Quat<T>::identity() {
    identity(this);
  };

  // Static Math operations
  template <class T>
  void Quat<T>::axisAngle2Quat(Quat* quat, Vec3<T>* axis, T angle) {
    // First store the normalized axis with the appropriate scale value
    T vecScale = std::sin(angle*(T)0.5) / axis->length();
    quat->m[0] = axis->m[0] * vecScale;
    quat->m[1] = axis->m[1] * vecScale;
    quat->m[2] = axis->m[2] * vecScale;
    // Now calcuate omega value
    quat->m[3] = std::cos(angle*(T)0.5);
  };

  template <class T>
  void Quat<T>::mat3x32Quat(Quat* quat, Mat3x3<T>* mat) {
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/...
#ifdef ROW_MAJOR
    T* m = mat->m;
    T trace = m[0] + m[4] + m[8];
    if (trace > 0) {
      T s = (T)0.5 / std::sqrt(trace + 1);
      quat->m[3] = (T)0.25 / s;
      quat->m[0] = (m[7] - m[5]) * s;
      quat->m[1] = (m[2] - m[6]) * s;
      quat->m[2] = (m[3] - m[1]) * s;
    } else {
      if (m[0] > m[4] && m[0] > m[8]) {
        T s = 2 * std::sqrt(1 + m[0] - m[4] - m[8]);
        T one_over_s = (T)1 / s;
        quat->m[3] = (m[7] - m[5]) * one_over_s;
        quat->m[0] = (T)0.25 * s;
        quat->m[1] = (m[1] + m[3]) * one_over_s;
        quat->m[2] = (m[2] + m[6]) * one_over_s;
      } else if (m[4] > m[8]) {
        T s = 2 * std::sqrt(1 + m[4] - m[0] - m[8]);
        T one_over_s = (T)1 / s;
        quat->m[3] = (m[2] - m[6]) * one_over_s;
        quat->m[0] = (m[1] + m[3]) * one_over_s;
        quat->m[1] = (T)0.25 * s;
        quat->m[2] = (m[5] + m[7]) * one_over_s;
      } else {
        T s = 2 * std::sqrt(1 + m[8] - m[0] - m[4]);
        T one_over_s = (T)1 / s;
        quat->m[3] = (m[3] - m[1]) * one_over_s;
        quat->m[0] = (m[2] + m[6]) * one_over_s;
        quat->m[1] = (m[5] + m[7]) * one_over_s;
        quat->m[2] = (T)0.25 * s;
      }
    }
#endif
#ifdef COLUMN_MAJOR
    T* m = mat->m;
    T trace = m[0] + m[4] + m[8];
    if (trace > 0) {
      T s = (T)0.5 / std::sqrt(trace + 1);
      quat->m[3] = (T)0.25 / s;
      quat->m[0] = (m[5] - m[7]) * s;
      quat->m[1] = (m[6] - m[2]) * s;
      quat->m[2] = (m[1] - m[3]) * s;
    } else {
      if (m[0] > m[4] && m[0] > m[8]) {
        T s = 2 * std::sqrt(1 + m[0] - m[4] - m[8]);
        T one_over_s = (T)1 / s;
        quat->m[3] = (m[5] - m[7]) * one_over_s;
        quat->m[0] = (T)0.25 * s;
        quat->m[1] = (m[3] + m[1]) * one_over_s;
        quat->m[2] = (m[6] + m[2]) * one_over_s;
      } else if (m[4] > m[8]) {
        T s = 2 * std::sqrt(1 + m[4] - m[0] - m[8]);
        T one_over_s = (T)1 / s;
        quat->m[3] = (m[6] - m[2]) * one_over_s;
        quat->m[0] = (m[3] + m[1]) * one_over_s;
        quat->m[1] = (T)0.25 * s;
        quat->m[2] = (m[7] + m[5]) * one_over_s;
      } else {
        T s = 2 * std::sqrt(1 + m[8] - m[0] - m[4]);
        T one_over_s = (T)1 / s;
        quat->m[3] = (m[1] - m[3]) * one_over_s;
        quat->m[0] = (m[6] + m[2]) * one_over_s;
        quat->m[1] = (m[7] + m[5]) * one_over_s;
        quat->m[2] = (T)0.25 * s;
      }
    }
#endif
  };

  template <class T>
  void Quat<T>::mat4x42Quat(Quat* quat, Mat4x4<T>* mat) {
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/...
#ifdef ROW_MAJOR
    T* m = mat->m;
    T trace = m[0] + m[5] + m[10];
    if (trace > 0) {
      T s = (T)0.5 / std::sqrt(trace + 1);
      quat->m[3] = (T)0.25 / s;
      quat->m[0] = (m[9] - m[6]) * s;
      quat->m[1] = (m[2] - m[8]) * s;
      quat->m[2] = (m[4] - m[1]) * s;
    } else {
      if (m[0] > m[5] && m[0] > m[10]) {
        T s = 2 * std::sqrt(1 + m[0] - m[5] - m[10]);
        T one_over_s = (T)1 / s;
        quat->m[3] = (m[9] - m[6]) * one_over_s;
        quat->m[0] = (T)0.25 * s;
        quat->m[1] = (m[1] + m[4]) * one_over_s;
        quat->m[2] = (m[2] + m[8]) * one_over_s;
      } else if (m[5] > m[10]) {
        T s = 2 * std::sqrt(1 + m[5] - m[0] - m[10]);
        T one_over_s = (T)1 / s;
        quat->m[3] = (m[2] - m[8]) * one_over_s;
        quat->m[0] = (m[1] + m[4]) * one_over_s;
        quat->m[1] = (T)0.25 * s;
        quat->m[2] = (m[6] + m[9]) * one_over_s;
      } else {
        T s = 2 * std::sqrt(1 + m[10] - m[0] - m[5]);
        T one_over_s = (T)1 / s;
        quat->m[3] = (m[4] - m[1]) * one_over_s;
        quat->m[0] = (m[2] + m[8]) * one_over_s;
        quat->m[1] = (m[6] + m[9]) * one_over_s;
        quat->m[2] = (T)0.25 * s;
      }
    }
#endif
#ifdef COLUMN_MAJOR
    T* m = mat->m;
    T trace = m[0] + m[5] + m[10];
    if (trace > 0) {
      T s = (T)0.5 / std::sqrt(trace + 1);
      quat->m[3] = (T)0.25 / s;
      quat->m[0] = (m[6] - m[9]) * s;
      quat->m[1] = (m[8] - m[2]) * s;
      quat->m[2] = (m[1] - m[4]) * s;
    } else {
      if (m[0] > m[5] && m[0] > m[10]) {
        T s = 2 * std::sqrt(1 + m[0] - m[5] - m[10]);
        T one_over_s = (T)1 / s;
        quat->m[3] = (m[6] - m[9]) * one_over_s;
        quat->m[0] = (T)0.25 * s;
        quat->m[1] = (m[4] + m[1]) * one_over_s;
        quat->m[2] = (m[8] + m[2]) * one_over_s;
      } else if (m[5] > m[10]) {
        T s = 2 * std::sqrt(1 + m[5] - m[0] - m[10]);
        T one_over_s = (T)1 / s;
        quat->m[3] = (m[8] - m[2]) * one_over_s;
        quat->m[0] = (m[4] + m[1]) * one_over_s;
        quat->m[1] = (T)0.25 * s;
        quat->m[2] = (m[9] + m[6]) * one_over_s;
      } else {
        T s = 2 * std::sqrt(1 + m[10] - m[0] - m[5]);
        T one_over_s = (T)1 / s;
        quat->m[3] = (m[1] - m[4]) * one_over_s;
        quat->m[0] = (m[8] + m[2]) * one_over_s;
        quat->m[1] = (m[9] + m[6]) * one_over_s;
        quat->m[2] = (T)0.25 * s;
      }
    }
#endif
  };

  template <class T>
  void Quat<T>::quat2AxisAngle(Quat* quat, Vec3<T>* axis, T* angle) {
    *angle = (T)2 * std::acos(quat->m[3]);
    if (abs(*angle) < EPSILON) {
      axis->set((T)1, (T)0, (T)0);
    } else {
      axis->set(quat->m[0], quat->m[1], quat->m[2]);
      axis->scale((T)1 / std::sin(*angle*0.5));
    }
  };

  template <class T>
  void Quat<T>::quat2Mat3x3(Quat* quat, Mat3x3<T>* mat) {
    // From David Eberly's book (game physics) page 721
    T* m = quat->m;  // For brevity: compiler should take care of this anyway
    T xx = m[0]*m[0];
    T yy = m[1]*m[1];
    T zz = m[2]*m[2];
    T xy = m[0]*m[1];
    T xz = m[0]*m[2];
    T xw = m[0]*m[3];
    T yz = m[1]*m[2];
    T yw = m[1]*m[3];
    T zw = m[2]*m[3];
    T* m_out = mat->m;
#ifdef ROW_MAJOR
    m_out[0] = 1-2*(yy+zz);
    m_out[1] = 2*(xy-zw);
    m_out[2] = 2*(xz+yw);
    m_out[3] = 2*(xy+zw); 
    m_out[4] = 1-2*(xx+zz); 
    m_out[5] = 2*(yz-xw);
    m_out[6] = 2*(xz-yw); 
    m_out[7] = 2*(yz+xw); 
    m_out[8] = 1-2*(xx+yy);
#endif
#ifdef COLUMN_MAJOR
    m_out[0] = 1-2*(yy+zz);
    m_out[3] = 2*(xy-zw);
    m_out[6] = 2*(xz+yw);
    m_out[1] = 2*(xy+zw); 
    m_out[4] = 1-2*(xx+zz); 
    m_out[7] = 2*(yz-xw);
    m_out[2] = 2*(xz-yw); 
    m_out[5] = 2*(yz+xw); 
    m_out[8] = 1-2*(xx+yy);
#endif
  };

  template <class T>
  void Quat<T>::quat2Mat4x4(Quat* quat, Mat4x4<T>* mat) {
    // From David Eberly's book (game physics) page 721
    // (or the same thing here: 
    //  http://renderfeather.googlecode.com/hg-history/034a1900d6e8b6c92440382658d2b01fc732c5de/Doc/optimized%20Matrix%20quaternion%20conversion.pdf)
    quat->normalize();  // Just in case.
    T* m = quat->m;  // For brevity: compiler should take care of this anyway
    T xx = m[0]*m[0];
    T yy = m[1]*m[1];
    T zz = m[2]*m[2];
    T xy = m[0]*m[1];
    T xz = m[0]*m[2];
    T xw = m[0]*m[3];
    T yz = m[1]*m[2];
    T yw = m[1]*m[3];
    T zw = m[2]*m[3];
    T* m_out = mat->m;
#ifdef ROW_MAJOR
    m_out[0] = 1-2*(yy+zz);
    m_out[1] = 2*(xy-zw);
    m_out[2] = 2*(xz+yw);
    m_out[3] = 0;
    m_out[4] = 2*(xy+zw); 
    m_out[5] = 1-2*(xx+zz); 
    m_out[6] = 2*(yz-xw);
    m_out[7] = 0;
    m_out[8] = 2*(xz-yw); 
    m_out[9] = 2*(yz+xw); 
    m_out[10] = 1-2*(xx+yy);
    m_out[11] = 0;
    m_out[12] = 0; 
    m_out[13] = 0; 
    m_out[14] = 0; 
    m_out[15] = 1;
#endif
#ifdef COLUMN_MAJOR
    m_out[0] = 1-2*(yy+zz);
    m_out[4] = 2*(xy-zw);
    m_out[8] = 2*(xz+yw);
    m_out[12] = 0;
    m_out[1] = 2*(xy+zw); 
    m_out[5] = 1-2*(xx+zz); 
    m_out[9] = 2*(yz-xw);
    m_out[13] = 0;
    m_out[2] = 2*(xz-yw); 
    m_out[6] = 2*(yz+xw); 
    m_out[10] = 1-2*(xx+yy);
    m_out[14] = 0;
    m_out[3] = 0; 
    m_out[7] = 0; 
    m_out[11] = 0; 
    m_out[15] = 1;
#endif
  };

  template <class T>
  void Quat<T>::mult(Quat* ret, Quat* a, Quat* b) {
    ret->m[3] = a->m[3]*b->m[3] - a->m[0]*b->m[0] - 
      a->m[1]*b->m[1] - a->m[2]*b->m[2];
    ret->m[0] = a->m[3]*b->m[0] + a->m[0]*b->m[3] + 
      a->m[1]*b->m[2] - a->m[2]*b->m[1]; 
    ret->m[1] = a->m[3]*b->m[1] - a->m[0]*b->m[2] + 
      a->m[1]*b->m[3] + a->m[2]*b->m[0]; 
    ret->m[2] = a->m[3]*b->m[2] + a->m[0]*b->m[1] - 
      a->m[1]*b->m[0] + a->m[2]*b->m[3]; 
  };

  template <class T>
  void Quat<T>::scale(Quat* a, T s) {
    a->m[0] *= s;
    a->m[1] *= s;
    a->m[2] *= s;
    a->m[3] *= s;
  };

  template <class T>
  void Quat<T>::inverse(Quat* a, Quat* b) {
    a->m[0] = -b->m[0];
    a->m[1] = -b->m[1];
    a->m[2] = -b->m[2];
    a->m[3] = b->m[3];
  };

  template <class T>
  void Quat<T>::normalize(Quat* a, Quat* b) {
    T one_over_length = (T)1 / length(b);
    a->m[0] = b->m[0] * one_over_length;
    a->m[1] = b->m[1] * one_over_length;
    a->m[2] = b->m[2] * one_over_length;
    a->m[3] = b->m[3] * one_over_length;
  };

  template <class T>
  T Quat<T>::length(Quat* a) {
    return std::sqrt(a->m[0]*a->m[0] + a->m[1]*a->m[1] + 
      a->m[2]*a->m[2] + a->m[3]*a->m[3]);
  };

  template <class T>
  void Quat<T>::mult(Vec3<T>* ret, Quat* a, Vec3<T>* b) {
    // The following is equivalent to converting to matrix them multiplying
    T qx2 = a->m[0]*a->m[0];
    T qy2 = a->m[1]*a->m[1];
    T qz2 = a->m[2]*a->m[2];

    T qxqy = a->m[0]*a->m[1];
    T qxqz = a->m[0]*a->m[2];
    T qxqw = a->m[0]*a->m[3];
    T qyqz = a->m[1]*a->m[2];
    T qyqw = a->m[1]*a->m[3];
    T qzqw = a->m[2]*a->m[3];

    ret->m[0] = (1-2*(qy2+qz2))*b->m[0] + (2*(qxqy-qzqw))*b->m[1] + 
      (2*(qxqz+qyqw))*b->m[2];
    ret->m[1] = (2*(qxqy+qzqw))*b->m[0] + (1-2*(qx2+qz2))*b->m[1] + 
      (2*(qyqz-qxqw))*b->m[2];
    ret->m[2] = (2*(qxqz-qyqw))*b->m[0] + (2*(qyqz+qxqw))*b->m[1] + 
      (1-2*(qx2+qy2))*b->m[2];
  };

  template <class T>
  T Quat<T>::dot(Quat* a, Quat* b) {
    return a->m[0] * b->m[0] + 
      a->m[1] * b->m[1] +
      a->m[2] * b->m[2] +
      a->m[3] * b->m[3];
  };

  template <class T>
  void Quat<T>::sub(Quat* ret, Quat* a, Quat* b) {
    ret->m[0] = a->m[0] - b->m[0];
    ret->m[1] = a->m[1] - b->m[1];
    ret->m[2] = a->m[2] - b->m[2];
    ret->m[3] = a->m[3] - b->m[3];
  };

  template <class T>
  void Quat<T>::add(Quat* ret, Quat* a, Quat* b) {
    ret->m[0] = a->m[0] + b->m[0];
    ret->m[1] = a->m[1] + b->m[1];
    ret->m[2] = a->m[2] + b->m[2];
    ret->m[3] = a->m[3] + b->m[3];
  };

  template <class T>
  void Quat<T>::slerp(Quat* ret, Quat* a, Quat* b, T interp ) {
    Quat temp(a);
    if (dot(temp, b) < 0) {
      temp.scale(-1);
    }
    T d = dot(temp, b);
    if (d >= 1) {
      ret->m[0] = temp->m[0];  // return a
      ret->m[1] = temp->m[1];
      ret->m[2] = temp->m[2];
      ret->m[3] = temp->m[3];
      return;
    }
    T theta = std::acos(d);
    if (abs(theta) < EPSILON) { 
      ret->m[0] = temp->m[0];  // return a
      ret->m[1] = temp->m[1];
      ret->m[2] = temp->m[2];
      ret->m[3] = temp->m[3];
      return;
    }
    T temp_scale = std::sin(theta-interp*theta)/std::sin(theta);
    T b_scale = std::sin(interp*theta)/std::sin(theta);
    ret->m[0] = temp->m[0]*temp_scale + b->m[0]*b_scale;
    ret->m[1] = temp->m[1]*temp_scale + b->m[1]*b_scale;
    ret->m[2] = temp->m[2]*temp_scale + b->m[2]*b_scale;
    ret->m[3] = temp->m[3]*temp_scale + b->m[3]*b_scale;
  };

  template <class T>
  bool Quat<T>::equal(Quat* a, Quat *b) {
    for (int i = 0; i < 4; i ++)
      if (abs(a->m[i] - b->m[i]) > EPSILON)
        return false;
    return true;
  };

  template <class T>
  bool Quat<T>::approxEqual(Quat* a, Quat *b) {
    for (int i = 0; i < 4; i ++)
      if (abs(a->m[i] - b->m[i]) > LOOSE_EPSILON)
        return false;
    return true;
  };

  template <class T>
  void Quat<T>::eulerAngles2Quat(Quat* quat, T r_xaxis, T r_yaxis, T r_zaxis) {
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // phi = x-axis, theta = y-axis, psi = r_zaxis 
    // Also, their q0 = our w
    T c1 = std::cos(r_xaxis/2);
    T s1 = std::sin(r_xaxis/2);
    T c2 = std::cos(r_yaxis/2);
    T s2 = std::sin(r_yaxis/2);
    T c3 = std::cos(r_zaxis/2);
    T s3 = std::sin(r_zaxis/2);
    T c1c2 = c1*c2;
    T s1s2 = s1*s2;
    quat->m[3] =c1c2*c3 - s1s2*s3;
    quat->m[0] =c1c2*s3 + s1s2*c3;
    quat->m[1] =s1*c2*c3 + c1*s2*s3;
    quat->m[2] =c1*s2*c3 - s1*c2*s3;
  }

  template <class T>
  void Quat<T>::quat2EulerAngles(Quat* quat, T& r_xaxis, T& r_yaxis, T& r_zaxis) {
    // Just in case:
    quat->normalize();

    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
    T test = quat->m[0]*quat->m[1] + quat->m[2]*quat->m[3];
    if (test > (T)0.499) { // singularity at north pole
      r_xaxis = (T)2.0 * atan2(quat->m[0],quat->m[3]);
      r_yaxis = (T)M_PI / (T)2.0;
      r_zaxis = (T)0;
      return;
    }
    if (test < (T)-0.499) { // singularity at south pole
      r_xaxis = (T)-2.0 * atan2(quat->m[0],quat->m[3]);
      r_yaxis = (T)-M_PI / (T)2.0;
      r_zaxis = (T)0;
      return;
    }
    T sqx = quat->m[0]*quat->m[0];
    T sqy = quat->m[1]*quat->m[1];
    T sqz = quat->m[2]*quat->m[2];
    r_xaxis = atan2(2*quat->m[1]*quat->m[3]-2*quat->m[0]*quat->m[2] , 1 - 2*sqy - 2*sqz);
    r_yaxis = asin(2*test);
    r_zaxis = atan2(2*quat->m[0]*quat->m[3]-2*quat->m[1]*quat->m[2] , 1 - 2*sqx - 2*sqz);
  };

  template <class T>
  void Quat<T>::xAxisRotation(Quat* a, T angle) {
    a->m[0] = sin(angle*(T)0.5);
    a->m[1] = (T)0;
    a->m[2] = (T)0;
    a->m[3] = cos(angle*(T)0.5);
  };

  template <class T>
  void Quat<T>::yAxisRotation(Quat* a, T angle) {
    a->m[0] = (T)0;
    a->m[1] = sin(angle*(T)0.5);
    a->m[2] = (T)0;
    a->m[3] = cos(angle*(T)0.5);
  };

  template <class T>
  void Quat<T>::zAxisRotation(Quat* a, T angle) {
    a->m[0] = (T)0;
    a->m[1] = (T)0;
    a->m[2] = sin(angle*(T)0.5);
    a->m[3] = cos(angle*(T)0.5);
  };  

  template <class T>
  void Quat<T>::identity(Quat* a) {
    a->m[0] = (T)0;
    a->m[1] = (T)0;
    a->m[2] = (T)0;
    a->m[3] = (T)1;
  };  

  template <class T>
  void Quat<T>::quatTrans2UnitDualQuat(Quat* q, Vec3<T>* t, T udq[2][4]) {
    // non-dual part (just copy q0):
    for (uint32_t i = 0; i < 4; i++) {
      udq[0][i] = q->m[i];
    }
    // dual part:
    udq[1][3] = (T)-0.5f * ( t->m[0]*q->m[0] + t->m[1]*q->m[1] + t->m[2]*q->m[2]);
    udq[1][0] =  (T)0.5f * ( t->m[0]*q->m[3] + t->m[1]*q->m[2] - t->m[2]*q->m[1]);
    udq[1][1] =  (T)0.5f * (-t->m[0]*q->m[2] + t->m[1]*q->m[3] + t->m[2]*q->m[0]);
    udq[1][2] =  (T)0.5f * ( t->m[0]*q->m[1] - t->m[1]*q->m[0] + t->m[2]*q->m[3]);
  }

  template <class T>
  void Quat<T>::unitDualQuat2QuatTrans(T udq[2][4], Quat* q, Vec3<T>* t) {
    // regular quaternion (just copy the non-dual part):
    for (uint32_t i = 0; i < 4; i++) {
      q->m[i] = udq[0][i];
    }
    // translation vector:
    t->m[0] = (T)2.0 * (-udq[1][3]*udq[0][0] + udq[1][0]*udq[0][3] - 
                         udq[1][1]*udq[0][2] + udq[1][2]*udq[0][1]);
    t->m[1] = (T)2.0 * (-udq[1][3]*udq[0][1] + udq[1][0]*udq[0][2] + 
                         udq[1][1]*udq[0][3] - udq[1][2]*udq[0][0]);
    t->m[2] = (T)2.0 * (-udq[1][3]*udq[0][2] - udq[1][0]*udq[0][1] + 
                         udq[1][1]*udq[0][0] + udq[1][2]*udq[0][3]);
  }

  // dq is a dual quaternion with non-zero dual part
  template <class T>
  void Quat<T>::dualQuat2QuatTrans(T dq[2][4], Quat* q, Vec3<T>* t) {
    T len = (T)0.0;
    for (uint32_t i = 0; i < 4; i++) {
      len += dq[0][i] * dq[0][i];
    }
    len = (T)sqrt(len); 
    for (uint32_t i = 0; i < 4; i++) {
      q->m[i] = dq[0][i] / len;
    }
    t->m[0] = (T)2.0*(-dq[1][3]*dq[0][0] + dq[1][0]*dq[0][3] - 
                       dq[1][1]*dq[0][2] + dq[1][2]*dq[0][1]) / len;
    t->m[1] = (T)2.0*(-dq[1][3]*dq[0][1] + dq[1][0]*dq[0][2] + 
                       dq[1][1]*dq[0][3] - dq[1][2]*dq[0][0]) / len;
    t->m[2] = (T)2.0*(-dq[1][3]*dq[0][2] - dq[1][0]*dq[0][1] + 
                       dq[1][1]*dq[0][0] + dq[1][2]*dq[0][3]) / len;
  }

  template <class T>
  void Quat<T>::decompose(Mat4x4<T>* affine_mat, Vec3<T>* translation, 
      Quat* rotation, Vec3<T>* scale) {
    AffineParts parts;
    HMatrix A;
    for (uint32_t r = 0; r < 4; r++) { 
      for (uint32_t c = 0; c < 4; c++) {
        A[r][c] = (float)(*affine_mat)(r, c);
      }
    }
    decomp_affine(A, &parts);
    translation->set(parts.t.x, parts.t.y, parts.t.z);
    scale->set(parts.k.x, parts.k.y, parts.k.z);
    rotation->set(parts.q.x, parts.q.y, parts.q.z, parts.q.w);
  }

  template <class T>
  void Quat<T>::decompose(Mat4x4<T>* affine_mat, Vec3<T>* translation, 
      Quat* rotation) {
    affine_mat->getTranslation(translation);
    Quat::mat4x42Quat(rotation, affine_mat);
  }

};  // namespace math

#endif
