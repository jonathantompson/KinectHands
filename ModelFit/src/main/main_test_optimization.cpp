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
#include "jtil/math/pso.h"
#include "jtil/math/pso_parallel.h"
#include "jtil/math/bfgs.h"
#include "jtil/data_str/vector.h"
#include "math/optimization_test_functions.h"

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif

using namespace std;
using namespace jtil::math;
using namespace jtil::data_str;

float c_rad[NUM_COEFFS_EXPONTIAL_FIT] = {2, 2, 2, 2};

void coeffUpdateFunc(float* coeff) { 
}

void exponentialFitParallel(jtil::data_str::Vector<float>& residues, 
                        jtil::data_str::Vector<float*>& coeffs) {
  for (uint32_t i = 0; i < coeffs.size(); i++) {

    residues[i] = exponentialFit(coeffs[i]);
  }
}

bool angle_coeffs[NUM_COEFFS_EXPONTIAL_FIT];

int main(int argc, char *argv[]) { 
#if defined(_DEBUG) && defined(_WIN32)
  _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
  // _CrtSetBreakAlloc(184);
#endif

  // *************************************************************
  PSO* solver = new PSO(NUM_COEFFS_EXPONTIAL_FIT, 17);
  solver->max_iterations = 10000;
  solver->delta_coeff_termination = 1e-8f;

  float ret_coeffs[NUM_COEFFS_EXPONTIAL_FIT];

  cout << endl << "RESULTS FOR REGULAR VERSION" << endl;

  solver->minimize(ret_coeffs, c_0_exponential_fit, c_rad, NULL, 
    exponentialFit, NULL);

  cout << endl << "Final coeff values:" << endl;
  for (uint32_t i = 0; i < NUM_COEFFS_EXPONTIAL_FIT; i++) {
    std::cout << ret_coeffs[i] << " ";
  }

  cout << endl << "Expecting:" << endl;
  for (uint32_t i = 0; i < NUM_COEFFS_EXPONTIAL_FIT; i++) {
    std::cout << c_answer_exponential_fit[i] << " ";
  }
  delete solver;

  // *************************************************************
  memset(angle_coeffs, 0, sizeof(angle_coeffs[0]) * NUM_COEFFS_EXPONTIAL_FIT);
  PSOParallel* solver2 = new PSOParallel(NUM_COEFFS_EXPONTIAL_FIT, 64);

  float ret_coeffs2[NUM_COEFFS_EXPONTIAL_FIT];

  cout << endl << "RESULTS FOR MODELFIT VERSION" << endl;

  solver2->minimize(ret_coeffs2, c_0_exponential_fit, c_rad, angle_coeffs,
    exponentialFitParallel, &coeffUpdateFunc);

  cout << endl << "Final coeff values:" << endl;
  for (uint32_t i = 0; i < NUM_COEFFS_EXPONTIAL_FIT; i++) {
    std::cout << ret_coeffs2[i] << " ";
  }

  cout << endl << "Expecting:" << endl;
  for (uint32_t i = 0; i < NUM_COEFFS_EXPONTIAL_FIT; i++) {
    std::cout << c_answer_exponential_fit[i] << " ";
  }
  delete solver2;

  // *************************************************************
  // TRY MINIMIZING THE HW7 Q4a problem
  BFGS* solver_bfgs = new BFGS(NUM_COEFFS_HW7_4A);
  solver_bfgs->verbose = true;
  solver_bfgs->max_iterations = 1000;
  solver_bfgs->delta_f_term = 1e-12f;
  solver_bfgs->jac_2norm_term = 1e-12f;
  solver_bfgs->delta_x_2norm_term = 1e-12f;

  float ret_coeffs_bfgs[NUM_COEFFS_HW7_4A];

   cout << endl << "RESULTS FOR BFGS" << endl;

  solver_bfgs->minimize(ret_coeffs_bfgs, c_0_hw7_4a, NULL, hw7_4a, 
    hw7_4a_jacob, NULL);

  cout << endl << "Final coeff values:" << endl;
  for (uint32_t i = 0; i < NUM_COEFFS_HW7_4A; i++) {
    std::cout << ret_coeffs_bfgs[i] << " ";
  }
  cout << "  --> with func value = ";
  cout << hw7_4a(ret_coeffs_bfgs) << endl; 

  cout << endl << "Expecting:" << endl;
  for (uint32_t i = 0; i < NUM_COEFFS_HW7_4A; i++) {
    std::cout << c_answer_hw7_4a[i] << " ";
  }
  cout << "  --> with func value = ";
  cout << hw7_4a(c_answer_hw7_4a) << endl << endl;
  delete solver_bfgs;
  
  // *************************************************************
  // TRY MINIMIZING THE HW7 Q4b problem
  BFGS* solver_bfgs2 = new BFGS(NUM_COEFFS_HW7_4B);
  solver_bfgs2->verbose = true;
  solver_bfgs2->max_iterations = 1000;
  solver_bfgs2->delta_f_term = 1e-12f;
  solver_bfgs2->jac_2norm_term = 1e-12f;
  solver_bfgs2->delta_x_2norm_term = 1e-12f;
  
  float ret_coeffs_bfgs2[NUM_COEFFS_HW7_4B];
  
  solver_bfgs2->minimize(ret_coeffs_bfgs2, c_0_hw7_4b, NULL, hw7_4b, 
    hw7_4b_jacob, NULL);
  
  cout << endl << "Final coeff values:" << endl;
  for (uint32_t i = 0; i < NUM_COEFFS_HW7_4B; i++) {
    std::cout << ret_coeffs_bfgs2[i] << " ";
  }
  cout << "  --> with func value = ";
  cout << hw7_4a(ret_coeffs_bfgs2) << endl;
  
  cout << endl << "Expecting:" << endl;
  for (uint32_t i = 0; i < NUM_COEFFS_HW7_4B; i++) {
    std::cout << c_answer_hw7_4b[i] << " ";
  }
  cout << "  --> with func value = ";
  cout << hw7_4a(c_answer_hw7_4b) << endl << endl;
  delete solver_bfgs2;

  #if defined(WIN32) || defined(_WIN32)
  system("PAUSE");
  #endif
}
