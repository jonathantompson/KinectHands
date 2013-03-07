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
#include "Eigen"

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
  typedef void (*EvalFuncPtr)(Eigen::MatrixXf& f, const Eigen::MatrixXf& coeff, 
    const Eigen::MatrixXf& x);
  typedef void (*LSJacobianFuncPtr)(Eigen::MatrixXf& jacob, 
    const Eigen::MatrixXf& coeff, const Eigen::MatrixXf& x);  // Least Squares
  typedef float (*ResidueFuncPtr)(const Eigen::MatrixXf& y, 
    const Eigen::MatrixXf& f_x, const Eigen::MatrixXf& coeff);
  typedef void (*CoeffUpdateFuncPtr)(Eigen::MatrixXf& coeff);
  typedef void (*CoeffPreturbFuncPtr)(Eigen::MatrixXf& coeff);

  // For direct search methods, we are only trying to minimize f(c, x) which 
  // returns a single float (which is the objective function value).  Since,
  // the direct search optimizer knows nothing about the function data points
  // it only needs the coeff as input.
  typedef float (*ObjectiveFuncPtr)(const Eigen::MatrixXf& coeff);
  typedef void (*JacobianFuncPtr)(Eigen::MatrixXf& jacob, 
    const Eigen::MatrixXf& coeff);
  
  struct OptNode {
    Eigen::MatrixXf coeff;
    float residue;
    OptNode& operator=(const OptNode &rhs) {
    if (this != &rhs) {
      this->coeff = rhs.coeff;
      this->residue = rhs.residue;
    }
    return *this;
  }
  };

  void PrintEigenMatrix(Eigen::MatrixXf& mat);

  struct SwarmNode {
  public:
    Eigen::MatrixXf vel;  // <1, num_coeffs>
    Eigen::MatrixXf pos;  // <1, num_coeffs>
    Eigen::MatrixXf best_pos;  // <1, num_coeffs>
    float residue;
    float best_residue;

    void resize(uint32_t size);
  };

};  // namespace math
};  // namespace jtil

#endif  // MATH_COMMON_SOLVER_HEADER
