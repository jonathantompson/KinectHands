//
//  main.cpp
//
//  This is a c++ version of the non_linear_least_sq.m file...  The section
//  at the bottom (Leven. Marq.).
//  

#include <stdlib.h>
#include <cmath>
#include <thread>
#include <iostream>
#include <limits>
#include "math/nm_fitting.h"
#include "math/common_fitting.h"
#include "math/optimization_test_functions.h"

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif

using namespace std;
using namespace jtil::math;
using Eigen::MatrixXf;

int main(int argc, char *argv[]) { 
  initOptimizationTestFunctions();

  // *************************************************************
  // TRY MINIMIZING THE NON_LINEAR LEAST SQUARES
  cout << "Non-linear least squares fit:" << endl;
  NMFitting* solver_exp_fit = new NMFitting(NUM_COEFFS_EXPONTIAL_FIT);
  solver_exp_fit->max_iterations = 10000;
  solver_exp_fit->simplex_diameter_termination = 10*std::numeric_limits<float>::epsilon();

  MatrixXf coeff_step_size(1, NUM_COEFFS_EXPONTIAL_FIT);
  for (uint32_t i = 0; i < NUM_COEFFS_EXPONTIAL_FIT; i++) {
    coeff_step_size(i) = 0.001f;
  }

  MatrixXf ret_coeffs(1, NUM_COEFFS_EXPONTIAL_FIT);

  ret_coeffs = solver_exp_fit->fitModel(c_0_exponential_fit, coeff_step_size, NULL, 
    exponentialFit, NULL);

  cout << endl << "Final coeff values:" << endl;
  PrintEigenMatrix(ret_coeffs);
  cout << "  --> with func value = ";
  cout << exponentialFit(ret_coeffs) << endl;

  cout << endl << "Expecting:" << endl << "|";
  PrintEigenMatrix(c_answer_exponential_fit);
  cout << "  --> with func value = ";
  cout << exponentialFit(c_answer_exponential_fit) << endl << endl;
  delete solver_exp_fit;

  // *************************************************************
  // NOW TRY MINIMIZING THE EXTENDED ROSENBROCK FUNCTION
  cout << "Extended Rosenbrock Function:" << endl;

  NMFitting* solver_rosenbrock = new NMFitting(NUM_COEFFS_ROSENBROCK);
  solver_rosenbrock->max_iterations = 10000;
  solver_rosenbrock->simplex_diameter_termination = 10*std::numeric_limits<float>::epsilon();

  MatrixXf coeff_step_size_rosenbrock(1, NUM_COEFFS_ROSENBROCK);
  for (uint32_t i = 0; i < NUM_COEFFS_ROSENBROCK; i++) {
    coeff_step_size_rosenbrock(i) = 1;
  }

  MatrixXf ret_coeffs_rosenbrock(1, NUM_COEFFS_ROSENBROCK);

  ret_coeffs_rosenbrock = solver_rosenbrock->fitModel(c_0_rosenbrock, 
    coeff_step_size_rosenbrock, NULL, extendedRosenbrock, NULL);

  cout << endl << "Final coeff values:" << endl;
  PrintEigenMatrix(ret_coeffs_rosenbrock);
  cout << "  --> with func value = ";
  cout << extendedRosenbrock(ret_coeffs_rosenbrock) << endl;

  cout << endl << "Expecting:" << endl << "|";
  PrintEigenMatrix(c_answer_rosenbrock);
  cout << "  --> with func value = ";
  cout << extendedRosenbrock(c_answer_rosenbrock) << endl << endl;
  delete solver_rosenbrock;

  // *************************************************************
  // NOW TRY MINIMIZING THE GENERALIZED RASTRIGIN FUNCTION
  cout << "Generalized Rastrigin Function:" << endl;

  NMFitting* solver_rastrigin = new NMFitting(NUM_COEFFS_RASTRIGIN);
  solver_rastrigin->max_iterations = 10000;
  solver_rastrigin->simplex_diameter_termination = 10*std::numeric_limits<float>::epsilon();

  MatrixXf coeff_step_size_rastrigin(1, NUM_COEFFS_RASTRIGIN);
  for (uint32_t i = 0; i < NUM_COEFFS_RASTRIGIN; i++) {
    coeff_step_size_rastrigin(i) = 0.1f;
  }

  MatrixXf ret_coeffs_rastrigin(1, NUM_COEFFS_RASTRIGIN);

  ret_coeffs_rastrigin = solver_rastrigin->fitModel(c_0_rastrigin, 
    coeff_step_size_rastrigin, NULL, generalizedRastrigin, NULL);

  cout << endl << "Final coeff values:" << endl;
  PrintEigenMatrix(ret_coeffs_rastrigin);
  cout << "  --> with func value = ";
  cout << generalizedRastrigin(ret_coeffs_rastrigin) << endl;

  cout << endl << "Expecting:" << endl << "|";
  PrintEigenMatrix(c_answer_rastrigin);
  cout << "  --> with func value = ";
  cout << generalizedRastrigin(c_answer_rastrigin) << endl << endl;
  delete solver_rastrigin;

  // *************************************************************
  // NOW TRY MINIMIZING THE ROTATED ELLIPSOIDAL FUNCTION
  cout << "Rotated Ellipsoidal Function:" << endl;

  NMFitting* solver_rotellips = new NMFitting(NUM_COEFFS_ROTELLIPS);
  solver_rotellips->max_iterations = 100000;
  solver_rotellips->simplex_diameter_termination = 10*std::numeric_limits<float>::epsilon();

  MatrixXf coeff_step_size_rotellips(1, NUM_COEFFS_ROTELLIPS);
  for (uint32_t i = 0; i < NUM_COEFFS_ROTELLIPS; i++) {
    coeff_step_size_rotellips(i) = 1.0f;
  }

  MatrixXf ret_coeffs_rotellips(1, NUM_COEFFS_ROTELLIPS);

  ret_coeffs_rotellips = solver_rotellips->fitModel(c_0_rotellips, 
    coeff_step_size_rotellips, NULL, rotatedEllipsoidal, NULL);

  cout << endl << "Final coeff values:" << endl;
  PrintEigenMatrix(ret_coeffs_rotellips);
  cout << "  --> with func value = ";
  cout << rotatedEllipsoidal(ret_coeffs_rotellips) << endl;

  cout << endl << "Expecting:" << endl << "|";
  PrintEigenMatrix(c_answer_rotellips);
  cout << "  --> with func value = ";
  cout << rotatedEllipsoidal(c_answer_rotellips) << endl << endl;
  delete solver_rotellips;

#if defined(WIN32) || defined(_WIN32)
  cout << endl;
  system("PAUSE");
#endif
}
