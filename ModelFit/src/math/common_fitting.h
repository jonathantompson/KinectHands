//
//  common_fitting.h
//
//  Created by Jonathan Tompson on 8/23/12.
//
//  Just some common structures used for non-linear optimiation algorithms
//

#ifndef MATH_COMMON_FITTING_HEADER
#define MATH_COMMON_FITTING_HEADER

#include "jtil/math/math_types.h"

#if defined(WIN32) || defined(_WIN32)
  #define UNIFORM_INT_DISTRIBUTION std::tr1::uniform_int_distribution<int>
  #define UNIFORM_REAL_DISTRIBUTION std::tr1::uniform_real_distribution<float>
  #define MERSINE_TWISTER_ENG std::tr1::mt19937
  #define NORMAL_REAL_DISTRIBUTION std::tr1::normal_distribution<float>
#else
  #define UNIFORM_INT_DISTRIBUTION std::uniform_int_distribution<int>
  #define UNIFORM_REAL_DISTRIBUTION std::uniform_real_distribution<float>
  #define MERSINE_TWISTER_ENG std::mt19937
  #define NORMAL_REAL_DISTRIBUTION std::normal_distribution<float>
#endif

namespace jtil {
namespace math {

  // NOW USING CALLBACKS SO WE CAN WRAP MEMBER FUNCTIONS!
  typedef void (*EvalFuncPtr)(float* f, const float* coeff, const float* x);
  typedef void (*LSJacobianFuncPtr)(float* jacob, const float* coeff, 
    const float* x);  // Least Squares
  typedef float (*ResidueFuncPtr)(const float* y, const float* f_x, 
    const float* coeff);
  typedef void (*CoeffUpdateFuncPtr)(float* coeff);
  typedef void (*CoeffPreturbFuncPtr)(float* coeff);

  // For direct search methods, we are only trying to minimize f(c, x) which 
  // returns a single float (which is the objective function value).  Since,
  // the direct search optimizer knows nothing about the function data points
  // it only needs the coeff as input.
  typedef float (*ObjectiveFuncPtr)(const float* coeff);
  typedef void (*JacobianFuncPtr)(float* jacob, 
    const float* coeff);
  
  struct OptNode {
    float* coeff;
    float residue;
    OptNode& operator=(const OptNode &rhs) {
    if (this != &rhs) {
      this->coeff = rhs.coeff;
      this->residue = rhs.residue;
    }
    return *this;
  }
  };

  void PrintVec(float* mat, uint32_t size);

  struct SwarmNode {
  public:
    SwarmNode();
    ~SwarmNode();
    void resize(const uint32_t size);

    float* vel;  // num_coeffs
    float* pos;  // num_coeffs
    float* best_pos;  // num_coeffs
    float residue;
    float best_residue;
  };

};  // namespace math
};  // namespace jtil

#endif  // MATH_COMMON_SOLVER_HEADER
