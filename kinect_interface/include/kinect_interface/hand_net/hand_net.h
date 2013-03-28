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
    // HAND_POS1: Base hand position --> (0,0,0) in the palm coordinate system
    HAND_POS1_U = 0, HAND_POS1_V = 1, HAND_POS1_D = 2,  // PALM_6 (bounding sph)
    HAND_POS2_U = 3, HAND_POS2_V = 4, HAND_POS2_D = 5,  // PALM_1
    HAND_POS3_U = 6, HAND_POS3_V = 7, HAND_POS3_D = 8,  // PALM_2
    HAND_POS4_U = 9, HAND_POS4_V = 10, HAND_POS4_D = 11,  // PALM_3
    // Thumb
    THUMB_BASE_U = 12, THUMB_BASE_V = 13, THUMB_BASE_D = 14,  // TH_KNU1_B
    THUMB_K2_U = 15, THUMB_K2_V = 16, THUMB_K2_D = 17,  // TH_KNU2_B
    THUMB_K3_U = 18, THUMB_K3_V = 19, THUMB_K3_D = 20,  // TH_KNU3_B
    THUMB_TIP_U = 21, THUMB_TIP_V = 22, THUMB_TIP_D = 23,  // TH_KNU3_A
    // F0
    F0_BASE_U = 24, F0_BASE_V = 25, F0_BASE_D = 26,  // F1_KNU1_B
    F0_K2_U = 27, F0_K2_V = 28, F0_K2_D = 29,  // F1_KNU2_B
    F0_TIP_U = 30, F0_TIP_V = 31, F0_TIP_D = 32,  // F1_KNU3_A
    // F1
    F1_BASE_U = 33, F1_BASE_V = 34,  F1_BASE_D = 35,  // F2_KNU1_B
    F1_K2_U = 36, F1_K2_V = 37, F1_K2_D = 38,  // F2_KNU2_B
    F1_TIP_U = 39, F1_TIP_V = 40, F1_TIP_D = 41,  // F2_KNU3_A
    // F2
    F2_BASE_U = 42, F2_BASE_V = 43, F2_BASE_D = 44,  // F3_KNU1_B
    F2_K2_U = 45, F2_K2_V = 46, F2_K2_D = 47,  // F3_KNU2_B
    F2_TIP_U = 48, F2_TIP_V = 49, F2_TIP_D = 50,  // F3_KNU3_A
    // F3
    F3_BASE_U = 51, F3_BASE_V = 52, F3_BASE_D = 53,  // F4_KNU1_B
    F3_K2_U = 54, F3_K2_V = 55, F3_K2_D = 56,  // F4_KNU2_B
    F3_TIP_U = 57, F3_TIP_V = 58, F3_TIP_D = 59,  // F4_KNU3_A

    HAND_NUM_COEFF_CONVNET = 60, 
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
