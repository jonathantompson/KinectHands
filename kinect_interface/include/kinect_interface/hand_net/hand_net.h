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
#define FEATURE_SIZE 2  // UV = 2, UVD = 3
#define NUM_FEATS_PER_FINGER 3
#define NUM_FEATS_PER_THUMB 4
#define NUM_FEATS_PER_PALM 4

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
  /*
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
  */

  /*
  typedef enum {
    // HAND_POS1: Base hand position --> (0,0,0) in the palm coordinate system
    HAND_POS1_U = 0,  HAND_POS1_V = 1,   // PALM_6 (bounding sph)
    HAND_POS2_U = 2,  HAND_POS2_V = 3,   // PALM_1
    HAND_POS3_U = 4,  HAND_POS3_V = 5,   // PALM_2
    HAND_POS4_U = 6,  HAND_POS4_V = 7,   // PALM_3
    // Thumb
    THUMB_BASE_U = 8, THUMB_BASE_V = 9,  // TH_KNU1_B
    THUMB_K2_U = 10,  THUMB_K2_V = 11,   // TH_KNU2_B
    THUMB_K3_U = 12,  THUMB_K3_V = 13,   // TH_KNU3_B
    THUMB_TIP_U = 14, THUMB_TIP_V = 15,  // TH_KNU3_A
    // F0
    F0_BASE_U = 16,   F0_BASE_V = 17,    // F1_KNU1_B
    F0_K2_U = 18,     F0_K2_V = 19,      // F1_KNU2_B
    F0_TIP_U = 20,    F0_TIP_V = 21,     // F1_KNU3_A
    // F1
    F1_BASE_U = 22,   F1_BASE_V = 23,    // F2_KNU1_B
    F1_K2_U = 24,     F1_K2_V = 25,      // F2_KNU2_B
    F1_TIP_U = 26,    F1_TIP_V = 27,     // F2_KNU3_A
    // F2
    F2_BASE_U = 28,   F2_BASE_V = 29,    // F3_KNU1_B
    F2_K2_U = 30,     F2_K2_V = 31,      // F3_KNU2_B
    F2_TIP_U = 32,    F2_TIP_V = 33,     // F3_KNU3_A
    // F3
    F3_BASE_U = 34,   F3_BASE_V = 35,    // F4_KNU1_B
    F3_K2_U = 36,     F3_K2_V = 37,      // F4_KNU2_B
    F3_TIP_U = 38,    F3_TIP_V = 39,     // F4_KNU3_A

    HAND_NUM_COEFF_CONVNET = 40, 
  } HandCoeffConvnet;
  */

  typedef enum {
    // HAND_POS1: Base hand position --> (0,0,0) in the palm coordinate system
    HAND_POS1_U = 0,  HAND_POS1_V = 1,   // PALM_6 (bounding sph)
    HAND_POS2_U = 2,  HAND_POS2_V = 3,   // PALM_1
    HAND_POS3_U = 4,  HAND_POS3_V = 5,   // PALM_2
    HAND_POS4_U = 6,  HAND_POS4_V = 7,   // PALM_3
    // Thumb
    THUMB_BASE_U = 8, THUMB_BASE_V = 9,  // TH_KNU1_B
    THUMB_K2_U = 10,  THUMB_K2_V = 11,   // TH_KNU2_B
    THUMB_K3_U = 12,  THUMB_K3_V = 13,   // TH_KNU3_B
    THUMB_TIP_U = 14, THUMB_TIP_V = 15,  // TH_KNU3_A
    // F0
    F0_BASE_U = 16,   F0_BASE_V = 17,    // F1_KNU1_B
    F0_K2_U = 18,     F0_K2_V = 19,      // F1_KNU2_B
    F0_TIP_U = 20,    F0_TIP_V = 21,     // F1_KNU3_A
    // F1
    F1_BASE_U = 22,   F1_BASE_V = 23,    // F2_KNU1_B
    F1_K2_U = 24,     F1_K2_V = 25,      // F2_KNU2_B
    F1_TIP_U = 26,    F1_TIP_V = 27,     // F2_KNU3_A
    // F2
    F2_BASE_U = 28,   F2_BASE_V = 29,    // F3_KNU1_B
    F2_K2_U = 30,     F2_K2_V = 31,      // F3_KNU2_B
    F2_TIP_U = 32,    F2_TIP_V = 33,     // F3_KNU3_A
    // F3
    F3_BASE_U = 34,   F3_BASE_V = 35,    // F4_KNU1_B
    F3_K2_U = 36,     F3_K2_V = 37,      // F4_KNU2_B
    F3_TIP_U = 38,    F3_TIP_V = 39,     // F4_KNU3_A

    HAND_NUM_COEFF_CONVNET = 40, 
  } HandCoeffConvnet;

  typedef enum {
    DEPTH_DATA = 0,
    HPF_DEPTH_DATA = 1,
  } HandNetDataType;

  class HandImageGenerator;
  class TorchStage;
  class Table;
  
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

    // Getter methods
    const float* hpf_hand_image() const;
    const float* coeff_convnet() const { return coeff_convnet_; }
    float* coeff_convnet() { return coeff_convnet_; }
    const HandImageGenerator* image_generator() const { return image_generator_; }
    const float* hand_image() const;
    const int32_t size_images() const;
    const jtil::math::Float3& uvd_com() const;

  private:
    HandImageGenerator* image_generator_;
    HandNetDataType data_type_;
    Table* conv_network_input_;
    int32_t num_conv_banks_;  // Set after Torch model is read from file
    TorchStage* conv_network_;
    float coeff_convnet_[HAND_NUM_COEFF_CONVNET];  // output data

    // Multithreading
    jtil::threading::ThreadPool* tp_;

    void calcCroppedHand(const int16_t* depth_in, const uint8_t* label_in);
    void calcHPFHandBanks();
    void releaseData();  // Call destructor on all dynamic data

    // Non-copyable, non-assignable.
    HandNet(HandNet&);
    HandNet& operator=(const HandNet&);
  };

  
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_HAND_NET_HEADER
