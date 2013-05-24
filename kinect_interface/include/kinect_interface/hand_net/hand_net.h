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
#define FEATURE_SIZE 3  // UV = 2, UVD = 3
#define NUM_FEATS_PER_FINGER 1
#define NUM_FEATS_PER_THUMB 1
#define NUM_FEATS_PER_PALM 3

#if defined(__APPLE__)
  #define CONVNET_FILE string("./../../../../../../../../../data/" \
          "handmodel_fullcoeffs_tanh_abs_mid_L2Pooling.net.convnet")
#else
  #define CONVNET_FILE string("./../data/handmodel.net.convnet")
#endif

namespace jtorch {
  class TorchStage;
  class Table;
}

namespace kinect_interface {
namespace hand_net {
  // Note 1: All hand positions are in the hand coordinate frame (defined as 
  //         the origin at the UV COM of the hand points).
  // Note 2: The convnet will have trouble learning the non-linear angle 
  //         mapping, so we need to have it recognize salient features in image
  //         space and then do inverse kinematics to find the joint angles.

  typedef enum {
    // HAND_POS1: Base hand position --> (0,0,0) in the palm coordinate system
    HAND_POS1_U = 0,  HAND_POS1_V = 1,  HAND_POS1_D = 2,   // PALM_3 (bounding sph)
    HAND_POS2_U = 3,  HAND_POS2_V = 4,  HAND_POS2_D = 5,   // PALM_1
    HAND_POS3_U = 6,  HAND_POS3_V = 7,  HAND_POS3_D = 8,   // PALM_2
    // Thumb
    THUMB_TIP_U = 9,  THUMB_TIP_V = 10, THUMB_TIP_D = 11,  // TH_KNU3_A
    // F0
    F0_TIP_U = 12,     F0_TIP_V = 13,   F0_TIP_D = 14,     // F1_KNU3_A
    // F1
    F1_TIP_U = 15,    F1_TIP_V = 16,    F1_TIP_D = 17,     // F2_KNU3_A
    // F2
    F2_TIP_U = 18,    F2_TIP_V = 19,    F2_TIP_D = 20,     // F3_KNU3_A
    // F3
    F3_TIP_U = 21,    F3_TIP_V = 22,    F3_TIP_D = 23,     // F4_KNU3_A
    HAND_NUM_COEFF_CONVNET = 24, 
  } HandCoeffConvnet;

  typedef enum {
    DEPTH_DATA = 0,
    HPF_DEPTH_DATA = 1,
  } HandNetDataType;

  class HandImageGenerator;
  class HandModelCoeff;
  class HandModel;
  
  class HandNet {
  public:
    // Constructor / Destructor
    HandNet();
    ~HandNet();

    void loadFromFile(const std::string& convnet_filename);

    // Top level functions
    // calcConvnetHeatMap - Calculates the convnet coeffs (not necessary
    // the same as the coeffs the renderer uses - see above)
    // Result is placed in coeff_convnet
    void calcConvnetHeatMap(const int16_t* depth, const uint8_t* label);
    void calcConvnetPose();

    // If you don't want the full convnet computation but you want the hand 
    // image -> Useful when we know the correct coeff but we want to debug
    void calcHandImage(const int16_t* depth, const uint8_t* label);

    // loadHandModels - Needs to be called whenever the renderer gets
    // destroyed and a new model needs to be loaded
    void loadHandModels();

    // Getter methods
    const float* hpf_hand_image() const;
    const float* heat_map_convnet() const { return heat_map_convnet_; }
    float* heat_map_convnet() { return heat_map_convnet_; }
    uint32_t heat_map_size() { return heat_map_size_; }
    uint32_t num_output_features() { return num_output_features_; }
    HandImageGenerator* image_generator() const { return image_generator_; }
    const float* hand_image() const;
    const int32_t size_images() const;
    const jtil::math::Float3& uvd_com() const;

  private:
    HandImageGenerator* image_generator_;
    HandNetDataType data_type_;
    int32_t num_conv_banks_;  // Set after Torch model is read from file
    jtorch::TorchStage* conv_network_;
    float* heat_map_convnet_;  // output data
    uint32_t heat_map_size_;
    uint32_t num_output_features_;
    HandModel* lhand_;  // Not owned here
    HandModel* rhand_;  // Not owned here
    HandModelCoeff* rest_pose_;
    HandModelCoeff* rhand_cur_pose_;
    HandModelCoeff* lhand_cur_pose_;

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
