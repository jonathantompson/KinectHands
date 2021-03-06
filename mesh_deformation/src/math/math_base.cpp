//
//  math_base.cpp
//
//  Created by Jonathan Tompson on 3/14/12.
//

#include <stdlib.h>
#include <cmath>
#include "math_base.h"
#include "math/math_types.h"

#if defined( _WIN32 )
#define rand_r rand
#else
#define rand_r rand
#endif

namespace math {
  bool seed = false;

  float CalcGaussianNoise(float mean, float stdDev) {
    if (seed == false) {
      srand(0);
      seed = true;
    }
#ifdef _WIN32
    float u1 = static_cast<float>(rand_r()) / 
      (static_cast<float>(RAND_MAX) + 1.0f);
    float u2 = static_cast<float>(rand_r()) / 
      (static_cast<float>(RAND_MAX) + 1.0f);
    float randStdNormal = std::sqrtf(-2.0f * std::logf(u1)) * 
      std::sinf(2.0f * static_cast<float>(M_PI) * u2);
    float randNormal = mean + stdDev * randStdNormal;
    return randNormal;
#else
    float u1 = static_cast<float>(rand_r()) / (static_cast<float>(RAND_MAX) + 
                                               1.0f);
    float u2 = static_cast<float>(rand_r()) / (static_cast<float>(RAND_MAX) + 
                                               1.0f);
    float randStdNormal = sqrtf(-2.0f * logf(u1)) * sinf(2.0f * 
                                                static_cast<float>(M_PI * u2));
    float randNormal = mean + stdDev * randStdNormal;
    return randNormal;
#endif
  }

  void calcOpenGLAffine(float* ret, Float3* axes, float* trans) {
    ret[0]  = axes[0][0];
    ret[1]  = axes[0][1];
    ret[2]  = axes[0][2];
    ret[3]  = 0.0f;
    ret[4]  = axes[1][0];
    ret[5]  = axes[1][1];
    ret[6]  = axes[1][2];
    ret[7]  = 0.0f;
    ret[8]  = axes[2][0];
    ret[9]  = axes[2][1];
    ret[10] = axes[2][2];
    ret[11] = 0.0f;
    ret[12] = trans[0];
    ret[13] = trans[1];
    ret[14] = trans[2];
    ret[15] = 1.0f;
  }
  void calcOpenGLAffine(float* ret, Float3* axes, Float3* trans) {
    calcOpenGLAffine(ret, axes, trans->m);
  }

  double Round(double a, double precision) {
    return std::floor(0.5+a/precision)*precision;
  }

  double Interpolate(const double &f0, const double &f1, double alpha) {
    return f0*(1-alpha) + f1*alpha;
  }

  inline float fabs_manual1(float x) {
    int y = (int&)x & 0x7FFFFFFF;
    return (float&)y;
  }

  inline float fabs_manual2(float g) {
    unsigned int *gg;
    gg=(unsigned int*)&g;
    *(gg)&=2147483647u;
    return g;
  }

}  // end namespace math
