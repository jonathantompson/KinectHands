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

#include "math/math_types.h"
#include "Eigen"

#define NUM_COEFFS_ROSENBROCK 4
#define NUM_COEFFS_RASTRIGIN 4
#define NUM_COEFFS_ROTELLIPS 4
#define NUM_COEFFS_EXPONTIAL_FIT 4  // Don't change!
#define NUM_PTS_EXPONTIAL_FIT 100  // Don't change!
#define NUM_COEFFS_HW7_4A 3  // Don't change!
#define NUM_COEFFS_HW7_4B 4  // Don't change!

namespace math {
  void initOptimizationTestFunctions();

  // Extended Rosenbrock function
  extern Eigen::MatrixXf c_0_rosenbrock;
  extern Eigen::MatrixXf c_answer_rosenbrock;
  float extendedRosenbrock(const Eigen::MatrixXf& coeff);

  // Generalized Rastragin function
  extern Eigen::MatrixXf c_0_rastrigin;
  extern Eigen::MatrixXf c_answer_rastrigin;
  float generalizedRastrigin(const Eigen::MatrixXf& coeff);

  // Rotated ellipsoidal function
  extern Eigen::MatrixXf c_0_rotellips;
  extern Eigen::MatrixXf c_answer_rotellips;
  float rotatedEllipsoidal(const Eigen::MatrixXf& coeff);

  // Exponential fit function (see matlab code non_linear_least_sq.m)
  extern Eigen::MatrixXf y_vals_rand;
  extern Eigen::MatrixXf x_vals_rand;
  extern Eigen::MatrixXf c_0_exponential_fit;
  extern Eigen::MatrixXf c_answer_exponential_fit;
  float exponentialFit(const Eigen::MatrixXf& coeff);

  // Non-linear R^3 function from HW7 (Q4a) of Numerical Optimization - NYU
  extern Eigen::MatrixXf c_0_hw7_4a;
  extern Eigen::MatrixXf c_answer_hw7_4a;
  float hw7_4a(const Eigen::MatrixXf& coeff);
  void hw7_4a_jacob(Eigen::MatrixXf& jacob, const Eigen::MatrixXf& coeff);
  
  extern Eigen::MatrixXf c_0_hw7_4b;
  extern Eigen::MatrixXf c_answer_hw7_4b;
  float hw7_4b(const Eigen::MatrixXf& coeff);
  void hw7_4b_jacob(Eigen::MatrixXf& jacob, const Eigen::MatrixXf& coeff);

};  // namespace math

#endif  // MATH_OPTIMIZATION_TEST_FUNCTIONS_HEADER
