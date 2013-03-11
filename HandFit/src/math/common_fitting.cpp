#include <iostream>
#include "math/common_fitting.h"

using std::cout;
using std::endl;

namespace jtil {
namespace math {

  void PrintEigenMatrix(Eigen::MatrixXf& mat) {
    for (int32_t y = 0; y < mat.rows(); y++) {
      std::cout << "|";
      for (int32_t x = 0; x < mat.cols(); x++) {
        printf("%+.15e", mat(y, x));
        if (x != mat.cols() - 1) {
          cout << " ";
        }
      }
      std::cout << "|" << endl;
    }
  }

  void SwarmNode::resize(uint32_t size) {
    vel.resize(1, size);
    pos.resize(1, size);
    best_pos.resize(1, size);
    residue = std::numeric_limits<float>::infinity();
    best_residue = std::numeric_limits<float>::infinity();
  }

};  // namespace math
};  // namespace jtil
