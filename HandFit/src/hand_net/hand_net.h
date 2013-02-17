//
//  hand_net.h
//
//  Created by Jonathan Tompson on 2/10/13.
//
//  Code to load in a convolutional neural network which will perform
//  full regression on the depth image to calculate the hand model coeffients.
//  The convnets are created in torch (see HandNet/hand_net.lua) and then
//  exported to a easily readable format.
//
//  The architecture is a 2 stage convolution + pooling, then a 2 stage fully
//  connected neural network.

#ifndef HAND_NET_HAND_NET_HEADER
#define HAND_NET_HAND_NET_HEADER

#include "math/math_types.h"
#include "depth_images_io.h"  // for src_dim
#include "eigen"

#define HAND_NET_PIX 192  // U, V size (before downsampling)
#define HAND_NET_DOWN_FACT 2
#define HAND_NET_IM_SIZE (HAND_NET_PIX / HAND_NET_DOWN_FACT)
#define HAND_SIZE 350.0f
#define HPF_SIGMA 6  // 1 finger width in downsampled image!
#define HPF_KERNEL_SIZE 31  // 31
#define NUM_HPF_BANKS 3

#if defined(__APPLE__)
  #define CONVNET_FILE string("./../../../../../../../../../data/" \
          "handmodel_fullcoeffs_tanh_abs_mid_L2Pooling.net.convnet")
#else
  #define CONVNET_FILE string("./../data/handmodel_fullcoeffs_tanh_abs_mid_L2Pooling.net.convnet")
#endif

namespace hand_net {
  // Note 1: All hand positions are in the hand coordinate frame (defined as 
  //         the origin at the XYZ COM of the hand points).
  // Note 2: The convnet will have trouble learning the non-linear angle 
  //         mapping, so we need to have it recognize salient features and then
  //         do inverse kinematics to find the joint angles.
  typedef enum {
    // Base hand position
    HAND_POS_X = 0, HAND_POS_Y = 1, HAND_POS_Z = 2,
    // Base hand orientation --> convnet learns (sin(x), cos(x)), better than x
    HAND_ORIENT_X_COS = 3, HAND_ORIENT_X_SIN = 4,
    HAND_ORIENT_Y_COS = 5, HAND_ORIENT_Y_SIN = 6,
    HAND_ORIENT_Z_COS = 7, HAND_ORIENT_Z_SIN = 8,
    // Wrist angle --> might need to be updated to a position later
    WRIST_THETA_COS = 9, WRIST_THETA_SIN = 10,
    WRIST_PHI_COS = 11, WRIST_PHI_SIN = 12,
    // Thumb
    THUMB_K1_X = 13, THUMB_K1_Y = 14, THUMB_K1_Z = 15,
    THUMB_K2_X = 16, THUMB_K2_Y = 17, THUMB_K2_Z = 18,
    THUMB_TIP_X = 19, THUMB_TIP_Y = 20, THUMB_TIP_Z = 21,
    // F0
    F0_K1_X = 22, F0_K1_Y = 23, F0_K1_Z = 24,
    F0_TIP_X = 25, F0_TIP_Y = 26, F0_TIP_Z = 27,
    // F0
    F0_K1_X = 28, F0_K1_Y = 29, F0_K1_Z = 30,
    F0_TIP_X = 31, F0_TIP_Y = 32, F0_TIP_Z = 33,
    // F0
    F0_K1_X = 34, F0_K1_Y = 35, F0_K1_Z = 36,
    F0_TIP_X = 37, F0_TIP_Y = 38, F0_TIP_Z = 39,
    // F0
    F0_K1_X = 40, F0_K1_Y = 41, F0_K1_Z = 42,
    F0_TIP_X = 43, F0_TIP_Y = 44, F0_TIP_Z = 45,

    HAND_NUM_COEFF_CONVNET = 46, 
  } HandCoeffConvnet;

  class ConvStage;
  class NNStage;
  
  class HandNet {
  public:
    // Constructor / Destructor
    HandNet(const std::string& convnet_filename);
    ~HandNet();

    // Top level functions
    void calcHandCoeff(const int16_t* depth, const uint8_t* label, 
      Eigen::MatrixXf& coeff);  // Slightly slower version --> converts 2 float
    void calcHandCoeff(const float* depth, const uint8_t* label, 
      Eigen::MatrixXf& coeff);  // Faster version

    void createLabelFromSyntheticDepth(const float* depth, uint8_t* label);

    // calcHandImage - creates cropped image, then creates a bank of HPF imgs
    void calcHandImage(const float* depth_in, const uint8_t* label_in);
    void calcCoeffConvnet(const Eigen::MatrixXf& coeff);

    // Some helper functions for debugging
    template <typename T>
    static void print3DTensorToStdCout(T* data, const int32_t n_feats,
      const int32_t height, const int32_t width);
    // This version just prints one feature and within this just a sub image
    template <typename T>
    static void print3DTensorToStdCout(T* data, const int32_t ifeat,
      const int32_t height, const int32_t width, const int32_t ustart, 
      const int32_t vstart, const int32_t printw, const int32_t printh);
    template <typename T>
    static void print3DTensorToStdCout(T** data, const int32_t n_feats,
      const int32_t height, const int32_t width);

    // Getter methods
    float* hpf_hand_image() { return hpf_hand_image_; }
    float* hand_image() { return hand_image_; }
    int32_t size_hpf_hand_image() { return size_hpf_hand_image_; } 
    float* coeff_convnet() { return coeff_convnet_; }

  private:
    int32_t n_conv_stages_;
    ConvStage** conv_stages_;

    int32_t n_nn_stages_;
    NNStage** nn_stages_;

    float* datcur_;  // The current stage's input data
    float* datnext_;  // The current stage's output data

    // Temporary data structures for image processing:
    float hand_image_[HAND_NET_PIX * HAND_NET_PIX];
    int32_t size_hpf_hand_image_;
    float* hpf_hand_image_;
    math::Float2 uv_com_;  // UV COM of the hand image.
    math::Float2 xyz_com_;  // XYZ COM of the hand image.
    float std_;  // STD of the hand image
    float coeff_convnet_[HAND_NUM_COEFF_CONVNET];

    float gauss_filt_unnormalized_[HPF_KERNEL_SIZE];

    // Some temporary data structures
    float float_depth_[src_dim];
    float cropped_depth[HAND_NET_PIX * HAND_NET_PIX];

    void loadFromFile(const std::string& convnet_filename);
    void calcCroppedHand(const float* depth_in, const uint8_t* label_in);
    void calcHPFHandBank(const float* depth_in, const uint8_t* label_in);

    // Non-copyable, non-assignable.
    HandNet(HandNet&);
    HandNet& operator=(const HandNet&);
  };

  // Some debug functions
  template <typename T>
  void HandNet::print3DTensorToStdCout(T* data, 
    const int32_t n_feats, const int32_t height, const int32_t width) {
    const int32_t dim = width * height;
    for (int32_t i = 0; i < n_feats; i++) {
      std::cout.setf(0, std::ios::showpos);
      std::cout << "  3dtensor[" << i << "] =" << std::endl;
      for (int32_t v = 0; v < height; v++) {
        if (v == 0) {
          std::cout << "    (0,0) ";
        } else {
          std::cout << "          ";
        }
        std::cout.setf(std::ios::showpos);
        for (int32_t u = 0; u < width; u++) {
          std::cout << data[i* dim + v * width + u];
          if (u != width - 1) {
            std::cout << ", ";
          } else {
            std::cout << std::endl;
          }
        }
      }
    }
    std::cout << std::resetiosflags(std::ios_base::showpos);
  };

  template <typename T>
  void HandNet::print3DTensorToStdCout(T* data, const int32_t ifeat,
    const int32_t height, const int32_t width, const int32_t ustart, 
    const int32_t vstart, const int32_t printw , const int32_t printh) {
    const int32_t dim = width * height;
    std::cout.setf(0, std::ios::showpos);
    std::cout << "  3dtensor[" << ifeat << "] =" << std::endl;
    std::cout << "    top left is (" << ustart << ", " << vstart << ")";
    std::cout << std::endl;
    for (int32_t v = vstart; v < vstart + printh; v++) {
      std::cout << "          ";
      std::cout.setf(std::ios::showpos);
      for (int32_t u = ustart; u < ustart + printw; u++) {
        std::cout << data[ifeat * dim + v * width + u];
        if (u != ustart + printw - 1) {
          std::cout << ", ";
        } else {
          std::cout << std::endl;
        }
      }
    }
    std::cout << std::resetiosflags(std::ios_base::showpos);
  }

  template <typename T>
  void HandNet::print3DTensorToStdCout(T** data, 
    const int32_t n_feats, const int32_t height, const int32_t width) {
    for (int32_t i = 0; i < n_feats; i++) {
      std::cout.setf(0, std::ios::showpos);
      std::cout << "  3dtensor[" << i << "] =" << std::endl;
      for (int32_t v = 0; v < height; v++) {
        if (v == 0) {
          std::cout << "    (0,0) ";
        } else {
          std::cout << "          ";
        }
        std::cout.setf(std::ios::showpos);
        for (int32_t u = 0; u < width; u++) {
          std::cout << data[i][v * width + u];
          if (u != width - 1) {
            std::cout << ", ";
          } else {
            std::cout << std::endl;
          }
        }
      }
    }
    std::cout << std::resetiosflags(std::ios_base::showpos);
  };
};  // hand_net namespace

#endif  // HAND_NET_HAND_NET_HEADER
