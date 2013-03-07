#include <random>
#include <stdexcept>
#include <iostream>
#include "math/nm_fitting.h"
#include "Eigen"

using Eigen::MatrixXf;
using std::cout;
using std::endl;
using std::runtime_error;

namespace jtil {
namespace math {
  MERSINE_TWISTER_ENG NMFitting::eng;  // a core engine class
  NORMAL_REAL_DISTRIBUTION NMFitting::dist;

  const float NMFitting::delta_r_  =  1.0f;  // (reflection coefficient)
  const float NMFitting::delta_e_  =  2.0f;  // (expansion coefficient)
  const float NMFitting::delta_oc_ =  0.5f;  // (outside contraction coefficient)
  const float NMFitting::delta_ic_ = -0.5f;  // (inside contraction coefficient)
  const float NMFitting::delta_s_  =  0.5f;  // (shrink coefficient)

  const double NMFitting::gamma_e_r_ = 1.0;  // (diameter modifiction coeff for reflection)
  const double NMFitting::gamma_e_e_ = 2.0;  // (diameter modifiction coeff for expansions)

  NMFitting::NMFitting(uint32_t num_coeffs) {
    c_center_.resize(1, num_coeffs);
    c_reflect_.resize(1, num_coeffs);
    c_expansion_.resize(1, num_coeffs);
    c_contract_inside_.resize(1, num_coeffs);
    c_contract_outside_.resize(1, num_coeffs);
    c_contract_.resize(1, num_coeffs);
    Y_matrix_.resize(num_coeffs, num_coeffs);

    num_coeffs_ = num_coeffs;
    num_coeffs_factorial_ = 1.0;
    for (double i = 2; i <= num_coeffs_; i++) {
      num_coeffs_factorial_ *= i;
    }
    double np4_factorial_ = num_coeffs_factorial_;
    for (double i = num_coeffs_+1; i <= num_coeffs_+4; i++) {
      np4_factorial_ *= i;
    }
    
    // Allocate space for the N+1 probe points
    simplex_vert_ = new OptNode[num_coeffs_ + 1];
    rotated_simplex_ = new OptNode[num_coeffs_ + 1];
    for (uint32_t i = 0; i < num_coeffs_ + 1; i++) {
      simplex_vert_[i].coeff.resize(1, num_coeffs_);
      simplex_vert_[i].residue = std::numeric_limits<float>::infinity();
      rotated_simplex_[i].coeff.resize(1, num_coeffs_);
      rotated_simplex_[i].residue = std::numeric_limits<float>::infinity();
    }
    ordered_vert_ = new OptNode*[num_coeffs_ + 1];
    for (uint32_t i = 0; i < num_coeffs_ + 1; i++) {
      ordered_vert_[i] = &simplex_vert_[i];
    }

    // Some default parameters
    max_iterations = 1000;
    simplex_diameter_termination = 10*std::numeric_limits<float>::epsilon();
    simplex_von_constant = 1e-100;
  }

  NMFitting::~NMFitting() {
    delete[] simplex_vert_;
    delete[] ordered_vert_;
    delete[] rotated_simplex_;
  }

  // Lots of good info here: http://www.scholarpedia.org/article/Nelder-Mead_algorithm
  // and some more here: http://en.wikipedia.org/wiki/Nelder%E2%80%93Mead_method
  MatrixXf& NMFitting::fitModel(const MatrixXf& start_c,
                                const Eigen::MatrixXf& coeff_step_size,
                                const bool* angle_coeff,
                                ObjectiveFuncPtr obj_func,
                                CoeffUpdateFuncPtr coeff_update_func) {
    
    eng.seed();
    obj_func_ = obj_func;
    coeff_update_func_ = coeff_update_func;
    angle_coeff_ = angle_coeff;

    cout << "Starting Nelder-Mead optimization..." << endl;

    // Generate N+1 probe points.  x_0 is x_in to allow proper restarts and the
    // remaining n vertices are generated to obtain one of two standard shapes
    // of simplex S: S is right angled at x_0, based on coordinate axes:
    // x_j = x_0 + h_j * e_j.
    // Note that this forms a positive basis.
    simplex_vert_[0].coeff = start_c;
    for (uint32_t i = 0; i < num_coeffs_; i++) {
      simplex_vert_[i+1].coeff = start_c;
      simplex_vert_[i+1].coeff(i) = (simplex_vert_[i+1].coeff(i) + 
                                          fabsf(coeff_step_size(i)));
      if (coeff_update_func_ != NULL) {
        coeff_update_func_(simplex_vert_[i+1].coeff);
      }
    }

    // Calculate f(x,c) for all the n+1 pts
    for (uint32_t i = 0; i < num_coeffs_ + 1; i++) {
      simplex_vert_[i].residue = obj_func_(simplex_vert_[i].coeff);
    }

    // Do an initial O(n^2) sort of the simplex points:
    InsertionSortSimplexPts();

    // Calculate the starting simplex area (might be very, very small for large
    // dimensions (prop to 1/n!).
    double start_area = calcNormalizedVolume();
    double eta = start_area * 1e-10;  // I'm not sure what this should be

    uint64_t iteration_num = 0;
    OptNode* worst_node;
    OptNode* second_worst_node;
    OptNode* best_node;
    double diam, von;

    do {
      iteration_num++;

      // 1. order the probe points using O(N^2) worst case, but mostly O(N)
      InsertionSortSimplexPts();

      worst_node = ordered_vert_[num_coeffs_];
      second_worst_node = ordered_vert_[num_coeffs_ - 1];
      best_node = ordered_vert_[0];

      double Delta = calcSimplexDiameter();
      if (static_cast<float>(Delta) < simplex_diameter_termination) {
        break;
      } 

#ifdef NM_VERBOSE_SOLVER
      cout << "Iteration " << iteration_num << ":" << endl;
      cout << "   --> Best residue = " << best_node->residue << endl;
      cout << "   --> Worst residue = " << worst_node->residue << endl;
      cout << "   --> Vertex residues: <";
      for (uint32_t i = 0; i < num_coeffs_+1; i++) {
        cout << ordered_vert_[i]->residue;
        if (i != num_coeffs_) { cout << ", "; }
      }
      cout << ">" << endl;
      cout << "   --> diameter = " << Delta << endl;
#endif

      // 2. Calculate the center of gravity for all points except x_n+1
      for (uint32_t i = 0; i < num_coeffs_; i++) {
        if (angle_coeff != NULL && angle_coeff[i]) {
          // The current coefficient is an angle --> Add up the complex
          // representation.
          float real_sum = 0;
          float imag_sum = 0;
          for (uint32_t j = 0; j < num_coeffs_; j++) {
            real_sum += cos(ordered_vert_[j]->coeff(i));
            imag_sum += sin(ordered_vert_[j]->coeff(i));
          }
          real_sum /= static_cast<float>(num_coeffs_);
          imag_sum /= static_cast<float>(num_coeffs_);
          float interpreted_angle = atan2(imag_sum, real_sum);
          c_center_(i) = interpreted_angle;
        } else {
          // Otherwise the current coefficient is NOT an angle and so just
          // calculate the mean.
          c_center_(i) = 0;
          for (uint32_t j = 0; j < num_coeffs_; j++) {
            c_center_(i) += ordered_vert_[j]->coeff(i);
          }
          c_center_(i) /= static_cast<float>(num_coeffs_);
        }
      }

      // 3. Reflection:
      // c_reflect_ = c_center_ + delta_r_ * (c_center_ - worst_node->coeff);
      interpolateCoeff(c_reflect_, c_center_, delta_r_, c_center_, worst_node->coeff);
      calcVonAndDiamWithReplacement(c_reflect_, von, diam);
      float r_reflect;
      bool attempt_contraction = false;
      bool attempt_expansion = false;
      if (von >= eta && diam <= gamma_e_r_ * Delta) {
        // Geometry is OK, calculate the reflected vertex objective function
        r_reflect = obj_func_(c_reflect_);

        if (r_reflect <= second_worst_node->residue - forcingFunction(Delta)) {
          attempt_expansion = true;
        } else {
          attempt_contraction = true;
        }
      } else {
        // Perform a safeguard rotation (rotate simplex around best vertex)
        rotated_simplex_[0].coeff = ordered_vert_[0]->coeff;
        rotated_simplex_[0].residue = ordered_vert_[0]->residue;
        float f_rot = std::numeric_limits<float>::infinity();
        for (uint32_t i = 1; i < num_coeffs_ + 1; i++) {

          // rotated_simplex_[i].coeff = best_node->coeff + (best_node->coeff - ordered_vert_[i]->coeff);
          interpolateCoeff(rotated_simplex_[i].coeff, best_node->coeff, 1.0, 
             best_node->coeff, ordered_vert_[i]->coeff);
          rotated_simplex_[i].residue = obj_func_(rotated_simplex_[i].coeff);
          if (rotated_simplex_[i].residue < f_rot) {
            f_rot = rotated_simplex_[i].residue;
          }
        }

        if (f_rot <= best_node->residue - forcingFunction(Delta)) {
          // Take the new rotated simplex and terminate the iteration
          for (uint32_t i = 0; i < num_coeffs_ + 1; i++) {
            simplex_vert_[i] = rotated_simplex_[i];
          }
#ifdef NM_VERBOSE_SOLVER
          cout << "   --> Using safeguard rotated simplex..." << endl;
#endif
          continue;
        } else {
          attempt_contraction = true;
        }

      }

      float r_expansion;
      if (attempt_expansion) {
        // c_expansion_ = c_center_ + delta_e_ * (c_center_ - worst_node->coeff);
        interpolateCoeff(c_expansion_, c_center_, delta_e_, c_center_, worst_node->coeff);
        calcVonAndDiamWithReplacement(c_expansion_, von, diam);

        if (von >= eta && diam <= gamma_e_e_ * Delta) {
          // Geometry is OK, calculate the expanded vertex objective function
          r_expansion = obj_func_(c_expansion_);

          if (r_expansion <= r_reflect) {
            // Replace worse coeff with the expansion point and terminate iteration
            worst_node->coeff = c_expansion_;
            worst_node->residue = r_expansion;
#ifdef NM_VERBOSE_SOLVER
            cout << "   --> Using expanded vertex..." << endl;
#endif
            continue;
          }
        }

        // Otherwise replace worst coeff with the reflection point and terminate iteration
        worst_node->coeff = c_reflect_;
        worst_node->residue = r_reflect;
#ifdef NM_VERBOSE_SOLVER
            cout << "   --> Using reflected vertex..." << endl;
#endif
        continue;
      }

      if (attempt_contraction) {
        // Calculate both the outside and inside contractions and pick the best
        // (if one or both of them have reasonable geometry)
        bool contraction_geometry_ok = false;
        float r_contract_inside = std::numeric_limits<float>::infinity();
        float r_contract_outside = std::numeric_limits<float>::infinity();
        float r_contract;

        // c_contract_outside_ = c_center_ + delta_oc_ * (c_center_ - worst_node->coeff);
        interpolateCoeff(c_contract_outside_, c_center_, delta_oc_, c_center_, worst_node->coeff);
        calcVonAndDiamWithReplacement(c_contract_outside_, von, diam);
        if (von >= eta && diam <= Delta) {
          contraction_geometry_ok = true;
          r_contract_outside = obj_func_(c_contract_outside_);
          r_contract = r_contract_outside;
          c_contract_ = c_contract_outside_;
        }

        // c_contract_inside_ = c_center_ + delta_ic_ * (c_center_ - worst_node->coeff);
        interpolateCoeff(c_contract_inside_, c_center_, delta_ic_, c_center_, worst_node->coeff);
        calcVonAndDiamWithReplacement(c_contract_inside_, von, diam);
        if (von >= eta && diam <= Delta) {
          r_contract_inside = obj_func_(c_contract_inside_);
          if (!contraction_geometry_ok || (contraction_geometry_ok && r_contract_inside <= r_contract_outside)) {
            r_contract = r_contract_inside;
            c_contract_ = c_contract_inside_;
          }
          contraction_geometry_ok = true;
        }

        if (contraction_geometry_ok) {
          if (r_contract <= worst_node->residue - forcingFunction(Delta)) {
            // Replace the worst node with the contracted point and terminate
            // the current iteration
            worst_node->coeff = c_contract_;
            worst_node->residue = r_contract;
#ifdef NM_VERBOSE_SOLVER
            cout << "   --> Using contracted vertex..." << endl;
#endif
            continue;
          }
        }
      }

      // Otherwise, if we get to here then we need to perform a shrink.
#ifdef NM_VERBOSE_SOLVER
      cout << "   --> Performing shrink operation..." << endl;
#endif
      float r_shrink = std::numeric_limits<float>::infinity();
      for (uint32_t i = 1; i < num_coeffs_ + 1; i++) {
        // ordered_vert_[i]->coeff = best_node->coeff + delta_ * (ordered_vert_[i]->coeff - best_node->coeff);
        interpolateCoeff(ordered_vert_[i]->coeff, best_node->coeff, delta_s_, 
          ordered_vert_[i]->coeff, best_node->coeff);
        ordered_vert_[i]->residue = obj_func_(ordered_vert_[i]->coeff);
        if (ordered_vert_[i]->residue < r_shrink) {
          r_shrink = ordered_vert_[i]->residue;
        }
      }

      if (r_shrink <= best_node->residue - forcingFunction(Delta)) {
        // Accept the shrunken simplex and terminate
        continue;
      } else {
        // Repeat the current iteration with the shrunken simplex
        iteration_num--;
        continue;
      }

    } while (iteration_num < max_iterations);
    
    cout << "Finished Nelder-Mead optimization with residue ";
    cout << ordered_vert_[0]->residue << ", " << iteration_num;
    cout << " iterations" << endl;

    return ordered_vert_[0]->coeff;
  }

  // http://en.wikipedia.org/wiki/Insertion_sort
  void NMFitting::InsertionSortSimplexPts() {
    for (uint32_t i = 1; i < num_coeffs_ + 1; i++) {
      OptNode* item = ordered_vert_[i];
      uint32_t i_hole = i;
      while (i_hole > 0 && ordered_vert_[i_hole - 1]->residue > item->residue) {
        // move hole to next smaller index
        ordered_vert_[i_hole] = ordered_vert_[i_hole - 1];
        i_hole--;
      }
      // put item in the hole
      ordered_vert_[i_hole] = item;
    }
  }
  
  // interpolateCoeff performs the following:
  // ret = a + interp_val * (b - c)
  void NMFitting::interpolateCoeff(Eigen::MatrixXf& ret, const Eigen::MatrixXf& a,
    const float interp_val, const Eigen::MatrixXf& b, const Eigen::MatrixXf& c) {
    for (uint32_t i = 0; i < num_coeffs_; i++) {
      if (angle_coeff_ != NULL && angle_coeff_[i]) {
        float real_a = cos(a(i));
        float imag_a = sin(a(i));
        float real_b = cos(b(i));
        float imag_b = sin(b(i));
        float real_c = cos(c(i));
        float imag_c = sin(c(i));
        float real_interp = real_a + interp_val * (real_b - real_c);
        float imag_interp = imag_a + interp_val * (imag_b - imag_c);
        float interp_angle = atan2(imag_interp, real_interp);
        ret(i) = interp_angle;
      } else {
        ret(i) = a(i) + interp_val * (b(i) - c(i));
      }
    }
    if (coeff_update_func_) {
      coeff_update_func_(ret);
    }
  }

  // Page 30 of Derivative Free Optimization - Conn, Scheinburg, Vicente
  double NMFitting::calcSimplexDiameter() {
    double diam = 0;
    for (uint32_t i = 0; i < num_coeffs_+1; i++) {
      for (uint32_t j = i+1; i < num_coeffs_+1; i++) {
        // Calculate the distance between the ith and the jth vertex
        double cur_dist = 0;
        for (uint32_t d = 0; d < num_coeffs_; d++) {
          double dist_d = static_cast<double>(ordered_vert_[i]->coeff(d) - 
                                              ordered_vert_[j]->coeff(d));
          cur_dist += (dist_d * dist_d);
        }
        cur_dist = sqrt(cur_dist);
        if (cur_dist > diam) {
          diam = cur_dist;
        }
      }
    }
    return diam;
  }

  // Page 30 of Derivative Free Optimization - Conn, Scheinburg, Vicente
  double NMFitting::calcApproxSimplexDiameter() {
    double diam = 0;
    for (uint32_t i = 1; i < num_coeffs_+1; i++) {
      // Calculate the distance between the ith and the jth vertex
      double cur_dist = 0;
      for (uint32_t d = 0; d < num_coeffs_; d++) {
        double dist_d = static_cast<double>(ordered_vert_[i]->coeff(d) - 
          ordered_vert_[0]->coeff(d));
        cur_dist += (dist_d * dist_d);
      }
      cur_dist = sqrt(cur_dist);
      if (cur_dist > diam) {
        diam = cur_dist;
      }
    }
    return diam;
  }

  // Page 146 of Derivative Free Optimization - Conn, Scheinburg, Vicente
  double NMFitting::calcNormalizedVolume() {
    double diam = calcSimplexDiameter();
    double vol = calcVolume();
    return vol / pow(diam, num_coeffs_);
  }

  // Page 146 of Derivative Free Optimization - Conn, Scheinburg, Vicente
  double NMFitting::calcVolume() {
    // Construct the Y matrix:
    for (uint32_t vert = 0; vert < num_coeffs_; vert++) {
      for (uint32_t dim = 0; dim < num_coeffs_; dim++) {
        Y_matrix_(dim, vert) = static_cast<double>(ordered_vert_[vert]->coeff(dim) - 
                                                   ordered_vert_[num_coeffs_]->coeff(dim));  // worst coeff
      }
    }
    double det_Y_matrix = Y_matrix_.determinant();
    return abs(det_Y_matrix) / num_coeffs_factorial_;
  }

  // Same as above, but with the simplex where the worst vertex is replaced
  // by the input vertex.
  void NMFitting::calcVonAndDiamWithReplacement(const MatrixXf& v_n, 
      double& volume_normalized, double& diameter) {
    c_tmp_ = ordered_vert_[num_coeffs_]->coeff;
    ordered_vert_[num_coeffs_]->coeff = v_n;
    diameter = calcSimplexDiameter();
    volume_normalized = calcVolume();
    volume_normalized = volume_normalized / pow(diameter, num_coeffs_);
    ordered_vert_[num_coeffs_]->coeff = c_tmp_;
  }

}  // namespace math
}  // namespace jtil
