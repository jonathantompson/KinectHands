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
#include "math/bfgs_fitting.h"
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
  // TRY MINIMIZING THE HW7 Q4a problem
  BFGSFitting* solver = new BFGSFitting(NUM_COEFFS_HW7_4A);
  solver->max_iterations = 1000;
  solver->delta_f_term = 1e-12f;
  solver->jac_2norm_term = 1e-12f;
  solver->delta_x_2norm_term = 1e-12f;

  MatrixXf ret_coeffs(1, NUM_COEFFS_HW7_4A);

  ret_coeffs = solver->fitModel(c_0_hw7_4a, NULL, 
    hw7_4a, hw7_4a_jacob, NULL);
  ret_coeffs.transpose();

  cout << endl << "Final coeff values:" << endl;
  PrintEigenMatrix(ret_coeffs);
  cout << "  --> with func value = ";
  cout << hw7_4a(ret_coeffs) << endl;

  cout << endl << "Expecting:" << endl << "|";
  PrintEigenMatrix(c_answer_hw7_4a);
  cout << "  --> with func value = ";
  cout << hw7_4a(c_answer_hw7_4a) << endl << endl;
  delete solver;
  
  // *************************************************************
  // TRY MINIMIZING THE HW7 Q4b problem
  BFGSFitting* solver2 = new BFGSFitting(NUM_COEFFS_HW7_4B);
  solver2->max_iterations = 1000;
  solver2->delta_f_term = 1e-12f;
  solver2->jac_2norm_term = 1e-12f;
  solver2->delta_x_2norm_term = 1e-12f;
  
  MatrixXf ret_coeffs2(1, NUM_COEFFS_HW7_4B);
  
  ret_coeffs2 = solver2->fitModel(c_0_hw7_4b, NULL,
    hw7_4b, hw7_4b_jacob, NULL);
  ret_coeffs2.transpose();
  
  cout << endl << "Final coeff values:" << endl;
  PrintEigenMatrix(ret_coeffs2);
  cout << "  --> with func value = ";
  cout << hw7_4a(ret_coeffs2) << endl;
  
  cout << endl << "Expecting:" << endl << "|";
  PrintEigenMatrix(c_answer_hw7_4b);
  cout << "  --> with func value = ";
  cout << hw7_4a(c_answer_hw7_4b) << endl << endl;
  delete solver2;

#if defined(WIN32) || defined(_WIN32)
  cout << endl;
  system("PAUSE");
#endif
}
