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
#define HAND_SIZE 300.0f
#define HPF_SIGMA 1.5f  // in pixels
#define HPF_KERNEL_SIZE 11  // Hopefully >= 2*(3*sigma) + 1 (MUST BE ODD!)
#define NUM_HPF_BANKS 3
#define HPF_GAIN 2.0f

#if defined(__APPLE__)
  #define CONVNET_FILE string("./../../../../../../../../../data/" \
          "handmodel_fullcoeffs_tanh_abs_mid_L2Pooling.net.convnet")
#else
  #define CONVNET_FILE string("./../data/handmodel.net.convnet")
#endif

namespace hand_model { class HandModelRenderer; }
namespace hand_model { class HandModel; }

namespace hand_net {
  // Note 1: All hand positions are in the hand coordinate frame (defined as 
  //         the origin at the UV COM of the hand points).
  // Note 2: The convnet will have trouble learning the non-linear angle 
  //         mapping, so we need to have it recognize salient features in image
  //         space and then do inverse kinematics to find the joint angles.
  typedef enum {
    // Base hand position
    HAND_POS_U = 0, HAND_POS_V = 1,
    // Base hand orientation --> convnet learns (sin(x), cos(x)), better than x
    HAND_ORIENT_X_COS = 2, HAND_ORIENT_X_SIN = 3,
    HAND_ORIENT_Y_COS = 4, HAND_ORIENT_Y_SIN = 5,
    HAND_ORIENT_Z_COS = 6, HAND_ORIENT_Z_SIN = 7,
    // Wrist angle --> might need to be updated to a position later
    WRIST_THETA_COS = 8, WRIST_THETA_SIN = 9,
    WRIST_PHI_COS = 10, WRIST_PHI_SIN = 11,
    // Thumb
    THUMB_K1_U = 12, THUMB_K1_V = 13,
    THUMB_K2_U = 14, THUMB_K2_V = 15,
    THUMB_TIP_U = 16, THUMB_TIP_V = 17,
    // F0
    F0_K1_U = 18, F0_K1_V = 19,
    F0_K2_U = 20, F0_K2_V = 21,
    F0_TIP_U = 22, F0_TIP_V = 23,
    // F1
    F1_K1_U = 24, F1_K1_V = 25, 
    F1_K2_U = 26, F1_K2_V = 27,
    F1_TIP_U = 28, F1_TIP_V = 29,
    // F2
    F2_K1_U = 30, F2_K1_V = 31,
    F2_K2_U = 32, F2_K2_V = 33,
    F2_TIP_U = 34, F2_TIP_V = 35,
    // F3
    F3_K1_U = 36, F3_K1_V = 37,
    F3_K2_U = 38, F3_K2_V = 39,
    F3_TIP_U = 40, F3_TIP_V = 41,

    HAND_NUM_COEFF_CONVNET = 42, 
  } HandCoeffConvnet;

  class ConvStage;
  class NNStage;
  
  class HandNet {
  public:
    // Constructor / Destructor
    HandNet();
    ~HandNet();

    void loadFromFile(const std::string& convnet_filename);

    // Top level functions
    // calcHandCoeffConvnet - Calculates the convnet coeffs (not necessary
    // the same as the coeffs the renderer uses - see above)
    void calcHandCoeffConvnet(const int16_t* depth, const uint8_t* label, 
      float coeff_convnet[HAND_NUM_COEFF_CONVNET]);

    void createLabelFromSyntheticDepth(const float* depth, uint8_t* label);

    // calcHandImage - creates cropped image, then creates a bank of HPF imgs
    void calcHandImage(const int16_t* depth_in, const uint8_t* label_in);
    void calcCoeffConvnet(hand_model::HandModel* hand,
      hand_model::HandModelRenderer* renderer, 
      float coeff_convnet[HAND_NUM_COEFF_CONVNET]);

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
    float* hpf_hand_images() { return hpf_hand_images_; }
    float* hand_image() { return hand_image_; }
    int32_t size_images() { return size_images_; } 
    inline const math::Float3& uvd_com() const { return uvd_com_; }

  private:
    int32_t n_conv_stages_;  // Number PER BANK!  Total = n_conv_stages_ * NUM_HPF_BANKS
    ConvStage** conv_stages_;

    int32_t n_nn_stages_;
    NNStage** nn_stages_;

    // The data is split up to make multithreading easier.
    float** conv_datcur_;  // The current stage's input data (on each bank)
    float** conv_datnext_;  // The current stage's output data
    float* nn_datcur_;
    float* nn_datnext_;

    // Temporary data structures for image processing:
    float* hand_image_;
    int32_t size_images_;  // Default: Size of (96x96 + 48x48 + 24x24)
    float* hpf_hand_images_;
    float* hpf_hand_images_coeff_;  // integral of a ones image with guass filt
    math::Float3 uvd_com_;  // UV COM of the hand image.
    float gauss_filt_[HPF_KERNEL_SIZE];  // This is unnormalized!
    float* im_temp1_;
    float* im_temp2_;

    void calcCroppedHand(const int16_t* depth_in, const uint8_t* label_in);
    void calcHPFHandBanks();
    void initHPFKernels();
    void calcHandImageUVFromXYZ(hand_model::HandModelRenderer* renderer, 
      math::Float3& xyz_pos, math::Float2& uv_pos);

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
