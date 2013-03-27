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
//

#ifndef KINECT_INTERFACE_HAND_NET_HAND_NET_HEADER
#define KINECT_INTERFACE_HAND_NET_HAND_NET_HEADER

#include "jtil/math/math_types.h"
#include "kinect_interface/depth_images_io.h"  // for src_dim
#include "jtil/threading/callback.h"

#define HN_NUM_WORKER_THREADS 6

#if defined(__APPLE__)
  #define CONVNET_FILE string("./../../../../../../../../../data/" \
          "handmodel_fullcoeffs_tanh_abs_mid_L2Pooling.net.convnet")
#else
  #define CONVNET_FILE string("./../data/handmodel.net.convnet")
#endif

namespace jtil { namespace threading { class ThreadPool; } }

namespace kinect_interface {

namespace hand_net {
  // Note 1: All hand positions are in the hand coordinate frame (defined as 
  //         the origin at the UV COM of the hand points).
  // Note 2: The convnet will have trouble learning the non-linear angle 
  //         mapping, so we need to have it recognize salient features in image
  //         space and then do inverse kinematics to find the joint angles.
  typedef enum {
    // Base hand position
    HAND_POS_U = 0, HAND_POS_V = 1, HAND_POS_D = 2,
    // Base hand orientation --> convnet learns (sin(x), cos(x)), better than x
    HAND_ORIENT_X_COS = 3, HAND_ORIENT_X_SIN = 4,
    HAND_ORIENT_Y_COS = 5, HAND_ORIENT_Y_SIN = 6,
    HAND_ORIENT_Z_COS = 7, HAND_ORIENT_Z_SIN = 8,
    // Wrist angle --> might need to be updated to a position later
    WRIST_THETA_COS = 9, WRIST_THETA_SIN = 10,
    WRIST_PHI_COS = 11, WRIST_PHI_SIN = 12,
    // Thumb
    THUMB_K1_U = 13, THUMB_K1_V = 14, THUMB_K1_D = 15,
    THUMB_K2_U = 16, THUMB_K2_V = 17, THUMB_K2_D = 18,
    THUMB_TIP_U = 19, THUMB_TIP_V = 20, THUMB_TIP_D = 21,
    // F0
    F0_K1_U = 22, F0_K1_V = 23, F0_K1_D = 24,
    F0_K2_U = 25, F0_K2_V = 26, F0_K2_D = 27,
    F0_TIP_U = 28, F0_TIP_V = 29, F0_TIP_D = 30,
    // F1
    F1_K1_U = 31, F1_K1_V = 32,  F1_K1_D = 33, 
    F1_K2_U = 34, F1_K2_V = 35, F1_K2_D = 36,
    F1_TIP_U = 37, F1_TIP_V = 38, F1_TIP_D = 39,
    // F2
    F2_K1_U = 40, F2_K1_V = 41, F2_K1_D = 42,
    F2_K2_U = 43, F2_K2_V = 44, F2_K2_D = 45,
    F2_TIP_U = 46, F2_TIP_V = 47, F2_TIP_D = 48,
    // F3
    F3_K1_U = 49, F3_K1_V = 50, F3_K1_D = 51,
    F3_K2_U = 52, F3_K2_V = 53, F3_K2_D = 54,
    F3_TIP_U = 55, F3_TIP_V = 56, F3_TIP_D = 57,

    HAND_NUM_COEFF_CONVNET = 58, 
  } HandCoeffConvnet;

  typedef enum {
    DEPTH_DATA = 0,
    HPF_DEPTH_DATA = 1,
  } HandNetDataType;

  class ConvStage;
  class NNStage;
  class HandImageGenerator;
  
  class HandNet {
  public:
    // Constructor / Destructor
    HandNet();
    ~HandNet();

    void loadFromFile(const std::string& convnet_filename);

    // Top level functions
    // calcHandCoeffConvnet - Calculates the convnet coeffs (not necessary
    // the same as the coeffs the renderer uses - see above)
    // Result is placed in coeff_convnet
    void calcHandCoeffConvnet(const int16_t* depth, const uint8_t* label);

    // If you don't want the full convnet computation but you want the hand 
    // image -> Useful when we know the correct coeff but we want to debug
    void calcHandImage(const int16_t* depth, const uint8_t* label);

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
    const float* hpf_hand_images() const;
    const float* coeff_convnet() const { return coeff_convnet_; }
    float* coeff_convnet() { return coeff_convnet_; }
    const HandImageGenerator* image_generator() const { return image_generator_; }
    const float* hand_image() const;
    const int32_t size_images() const;
    const jtil::math::Float3& uvd_com() const;

  private:
    HandImageGenerator* image_generator_;

    HandNetDataType data_type_;

    int32_t num_conv_banks_;
    int32_t n_conv_stages_;  // n_conv_stages_ = Number PER BANK!  
    ConvStage** conv_stages_;

    int32_t n_nn_stages_;
    NNStage** nn_stages_;

    float coeff_convnet_[HAND_NUM_COEFF_CONVNET];  // output data

    // The data is split up to make multithreading easier.
    float** conv_datcur_;  // The current stage's input data (on each bank)
    float** conv_datnext_;  // The current stage's output data
    float* nn_datcur_;
    float* nn_datnext_;

    // Multithreading
    jtil::threading::ThreadPool* tp_;

    void calcCroppedHand(const int16_t* depth_in, const uint8_t* label_in);
    void calcHPFHandBanks();
    void releaseData();  // Call destructor on all dynamic data

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
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_HAND_NET_HEADER
