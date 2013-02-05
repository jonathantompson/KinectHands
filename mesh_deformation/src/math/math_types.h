//
//  math_types.h
//
//  Created by Jonathan Tompson on 4/23/12.
//
//  Vectors and Matrix types are templates.  There's not runtime overheads
//

#ifndef MATH_MATH_TYPES_HEADER
#define MATH_MATH_TYPES_HEADER

#include <stdint.h>  // For uint8_t, uint16_t, etc
#include <float.h>
#include <cmath>

#ifndef EPSILON
  #define EPSILON (2 * FLT_EPSILON)  // 2 times machine precision for float
#endif

#ifndef LOOSE_EPSILON
  #define LOOSE_EPSILON 0.00001f
#endif

// Define either ROW_MAJOR (DirectX) or COLUMN_MAJOR (openGL)
// #define ROW_MAJOR
#define COLUMN_MAJOR

#include "math/vec2.h"
#include "math/vec3.h"
#include "math/vec4.h"
#include "math/mat2x2.h"
#include "math/mat3x3.h"
#include "math/mat4x4.h"
#include "math/quat.h"
#include "math/plane.h"

#ifndef M_E
  #define M_E     2.71828182845904523536
#endif
#ifndef M_PI
  #define M_PI    3.14159265358979323846
#endif
#ifndef M_PI_2
  #define M_PI_2  1.57079632679489661923
#endif
#ifndef M_PI_4
  #define M_PI_4  0.785398163397448309616
#endif
#ifndef PI_OVER_180
  #define PI_OVER_180 0.017453292519943295769236907684886  // 2*PI / 360
#endif
#ifndef PI_OVER_360
  #define PI_OVER_360 0.0087266462599716478846184538424431
#endif

#ifndef MAX_UINT32
  #define MAX_UINT32 0xffffffff
#endif
#ifndef MAX_UINT64
  #define MAX_UINT64 0xffffffffffffffff
#endif

namespace math {

  typedef Vec2<int> Int2;
  typedef Vec2<float> Float2;
  typedef Vec2<double> Double2;

  typedef Vec3<int> Int3;
  typedef Vec3<float> Float3;
  typedef Vec3<double> Double3;

  typedef Vec4<int> Int4;
  typedef Vec4<float> Float4;
  typedef Vec4<double> Double4;

  typedef Mat2x2<float> Float2x2;
  typedef Mat2x2<double> Double2x2;

  typedef Mat3x3<float> Float3x3;
  typedef Mat3x3<double> Double3x3;

  typedef Mat4x4<float> Float4x4;
  typedef Mat4x4<double> Double4x4;

  typedef Quat<float> FloatQuat;
  typedef Quat<double> DoubleQuat;
};  // namespace math

#if defined(ROW_MAJOR) && defined(COLUMN_MAJOR)
  #error define either ROW_MAJOR or COLUMN_MAJOR but not both
#endif

#if !defined(ROW_MAJOR) && !defined(COLUMN_MAJOR)
  #error define either ROW_MAJOR or COLUMN_MAJOR but not both
#endif

#endif
