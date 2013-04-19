//
//  optimization_test_functions.h
//
//  Created by Jonathan Tompson on 10/15/12.
//
//  Some functions for testing optimization routines.
//
//  You must call 'initOptimizationTestFunctions()' before using these functions
//

#ifndef MATH_OPTIMIZATION_TEST_FUNCTIONS_HEADER
#define MATH_OPTIMIZATION_TEST_FUNCTIONS_HEADER

#include "jtil/math/math_types.h"

#define NUM_COEFFS_ROSENBROCK 4
#define NUM_COEFFS_RASTRIGIN 4
#define NUM_COEFFS_ROTELLIPS 4
#define NUM_COEFFS_EXPONTIAL_FIT 4  // Don't change!
#define NUM_PTS_EXPONTIAL_FIT 100  // Don't change!
#define NUM_COEFFS_HW7_4A 3  // Don't change!
#define NUM_COEFFS_HW7_4B 4  // Don't change!

namespace jtil {
namespace math {
  void initOptimizationTestFunctions();

  // Extended Rosenbrock function
  extern float c_0_rosenbrock[];
  extern float c_answer_rosenbrock[];
  float extendedRosenbrock(const float* coeff);

  // Generalized Rastragin function
  extern float c_0_rastrigin[];
  extern float c_answer_rastrigin[];
  float generalizedRastrigin(const float* coeff);

  // Rotated ellipsoidal function
  extern float c_0_rotellips[];
  extern float c_answer_rotellips[];
  float rotatedEllipsoidal(const float* coeff);

  // Exponential fit function (see matlab code non_linear_least_sq.m)
  extern float y_vals_rand[];
  extern float x_vals_rand[];
  extern float c_0_exponential_fit[];
  extern float c_answer_exponential_fit[];
  float exponentialFit(const float* coeff);

  // Non-linear R^3 function from HW7 (Q4a) of Numerical Optimization - NYU
  extern float c_0_hw7_4a[];
  extern float c_answer_hw7_4a[];
  float hw7_4a(const float* coeff);
  void hw7_4a_jacob(float* jacob, const float* coeff);
  
  extern float c_0_hw7_4b[];
  extern float c_answer_hw7_4b[];
  float hw7_4b(const float* coeff);
  void hw7_4b_jacob(float* jacob, const float* coeff);

}  // namespace math
}  // namespace jtil

#endif  // MATH_OPTIMIZATION_TEST_FUNCTIONS_HEADER
