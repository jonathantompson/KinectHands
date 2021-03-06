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

#pragma once

#include "jtil/math/math_types.h"
#include "kinect_interface_primesense/hand_net/hand_model_coeff.h"  // For HandCoeff
#include "kinect_interface_primesense/depth_images_io.h"  // for src_dim
#include "kinect_interface_primesense/open_ni_funcs.h"
#include "jtil/threading/callback.h"

#define HN_NUM_WORKER_THREADS 6
#define FEATURE_SIZE 3  // UV = 2, UVD = 3
#define NUM_FEATS_PER_FINGER 2
#define NUM_FEATS_PER_THUMB 3
#define NUM_FEATS_PER_PALM 3

#define NUM_COEFFS_PER_GAUSSIAN 5  // (mean_u, mean_v, std_u, std_v)
#define X_DIM_LM_FIT 2
#define BFGS_FINGER_NUM_COEFF 3
#define RAD_UVD_SEARCH 2

#define HN_PSO_RAD_FINGERS 0.30f  // Search radius in frac of min - max coeff
#define HN_PSO_RAD_THUMB 0.20f
#define HN_PSO_RAD_EULER 0.20f
#define HN_PSO_RAD_POSITION 2.0f * (float)M_PI * (5.0f / 100.0f)
#define HN_PSO_SWARM_SIZE 32
// #define HN_PSO_NUM_ITERATIONS 50 --> This now an input parameter 

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

namespace jtil { namespace math { template <class T> class LMFit; } }
namespace jtil { namespace math { template <class T> class BFGS; } }
namespace jtil { namespace math { class PSOParallel; } }
namespace jtil { namespace renderer { class Camera; } }
namespace jtil { namespace data_str { template <class T> class Vector; } }

namespace kinect_interface_primesense {
namespace hand_net {
  // Note 1: All hand positions are in the hand coordinate frame (defined as 
  //         the origin at the UV COM of the hand points).
  // Note 2: The convnet will have trouble learning the non-linear angle 
  //         mapping, so we need to have it recognize salient features in image
  //         space and then do inverse kinematics to find the joint angles.
  // Note 3: These feature points are just the locations of some of the 
  //         bounding spheres that I attach to the model.

  const uint32_t num_convnet_feats = 14;
  const uint32_t convnet_sphere_indices[num_convnet_feats] =  {PALM_3, 
    PALM_1, PALM_2, TH_KNU3_A, TH_KNU3_B, TH_KNU2_B, F1_KNU3_A, F1_KNU2_B, 
    F2_KNU3_A, F2_KNU2_B, F3_KNU3_A, F3_KNU2_B, F4_KNU3_A, F4_KNU2_B};

  typedef enum {
    // HAND_POS1: Base hand position --> (0,0,0) in the palm coordinate system
    HAND_POS1_U = 0,   HAND_POS1_V = 1,  HAND_POS1_D = 2,   // PALM_3 (bounding sph)
    HAND_POS2_U = 3,   HAND_POS2_V = 4,  HAND_POS2_D = 5,   // PALM_1
    HAND_POS3_U = 6,   HAND_POS3_V = 7,  HAND_POS3_D = 8,   // PALM_2
    // Thumb
    THUMB_TIP_U = 9,   THUMB_TIP_V = 10, THUMB_TIP_D = 11,  // TH_KNU3_A
    THUMB_MID_U = 12,  THUMB_MID_V = 13, THUMB_MID_D = 14,  // TH_KNU3_B
    THUMB_KNU_U = 15,  THUMB_KNU_V = 16, THUMB_KNU_D = 17,  // TH_KNU2_B
    // F0
    F0_TIP_U = 18,    F0_TIP_V = 19,   F0_TIP_D = 20,     // F1_KNU3_A
    F0_KNU_U = 21,    F0_KNU_V = 22,   F0_KNU_D = 23,     // F1_KNU2_B
    // F1
    F1_TIP_U = 24,    F1_TIP_V = 25,   F1_TIP_D = 26,     // F2_KNU3_A
    F1_KNU_U = 27,    F1_KNU_V = 28,   F1_KNU_D = 29,     // F2_KNU2_B
    // F2
    F2_TIP_U = 30,    F2_TIP_V = 31,   F2_TIP_D = 32,     // F3_KNU3_A
    F2_KNU_U = 33,    F2_KNU_V = 34,   F2_KNU_D = 35,     // F3_KNU2_B
    // F3
    F3_TIP_U = 36,    F3_TIP_V = 37,   F3_TIP_D = 38,     // F4_KNU3_A
    F3_KNU_U = 39,    F3_KNU_V = 40,   F3_KNU_D = 41,     // F4_KNU2_B
    HAND_NUM_COEFF_CONVNET = 42, 
  } HandCoeffConvnet;

  const uint32_t num_uvd_feats = NUM_BOUNDING_SPHERES;
  const uint32_t HAND_NUM_COEFF_UVD = NUM_BOUNDING_SPHERES * 3;

  typedef enum { 
    BFGS_HAND_POS_X        = 0, 
    BFGS_HAND_POS_Y        = 1,
    BFGS_HAND_POS_Z        = 2,
    BFGS_HAND_ORIENT_X     = 3,
    BFGS_HAND_ORIENT_Y     = 4,
    BFGS_HAND_ORIENT_Z     = 5,
    BFGS_THUMB_THETA       = 6,
    BFGS_THUMB_PHI         = 7,
    BFGS_THUMB_K1_THETA    = 8,
    BFGS_THUMB_K1_PHI      = 9,
    BFGS_THUMB_K2_PHI      = 10,
    BFGS_F0_THETA          = 11,
    BFGS_F0_PHI            = 12,
    BFGS_F0_CURL           = 13,
    BFGS_F1_THETA          = 14,
    BFGS_F1_PHI            = 15,
    BFGS_F1_CURL           = 16,
    BFGS_F2_THETA          = 17,
    BFGS_F2_PHI            = 18,
    BFGS_F2_CURL           = 19,
    BFGS_F3_THETA          = 20,
    BFGS_F3_PHI            = 21,
    BFGS_F3_CURL           = 22,
    BFGS_NUM_PARAMETERS    = 23
  } BFGSHandCoeff;

  typedef enum {
    GaussAmp = 0,
    GaussMeanU = 1,
    GaussMeanV = 2,
    GaussVarU = 3,
    GaussVarV = 4,
  } Gauss2DCoeff;

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
    void calcConvnetHeatMap(const int16_t* depth, const uint8_t* label, 
      bool flip_convnet_input);
    void calcConvnetPose(const int16_t* depth, const uint8_t* label,
      const float smoothing_factor, const uint64_t max_pso_iterations);
    void resetTracking();

    // If you don't want the full convnet computation but you want the hand 
    // image -> Useful when we know the correct coeff but we want to debug
    void calcHandImage(const int16_t* depth, const uint8_t* label, 
      const bool flip = false);

    // loadHandModels - Needs to be called whenever the renderer gets
    // destroyed and a new model needs to be loaded
    void loadHandModels();
    void setModelVisibility(const bool visible);

    void setHandSize(const float size);

    // Getter methods
    const float* hpf_hand_image() const;
    const float* heat_map_convnet() const { return heat_map_convnet_; }
    float* heat_map_convnet() { return heat_map_convnet_; }
    uint32_t heat_map_size() { return heat_map_size_; }  // Number of pixels total
    uint32_t num_output_features() { return num_output_features_; }
    HandImageGenerator* image_generator() const { return image_generator_; }
    const float* hand_image() const;
    const int32_t size_images() const;
    const jtil::math::Float3& uvd_com() const;
    const float* gauss_coeff() const { return gauss_coeff_; }  // 640x480 space
    const float* gauss_coeff_hm() const { return gauss_coeff_hm_; }  // 0 to 1 hm space
    const HandModelCoeff* rhand_cur_pose() const { return rhand_cur_pose_; }

  private:
    HandImageGenerator* image_generator_;
    OpenNIFuncs open_ni_funcs_;
    int32_t num_conv_banks_;  // Set after Torch model is read from file
    jtorch::TorchStage* conv_network_;
    float* heat_map_convnet_;  // output data
    float* hm_temp_;  // output data
    uint32_t heat_map_size_;
    uint32_t num_output_features_;
    HandModel* rhand_;  // Not owned here
    HandModelCoeff* rest_pose_;
    HandModelCoeff* rhand_cur_pose_;
    HandModelCoeff* rhand_prev_pose_;
    float* gauss_coeff_;  // For each feature in 640x480 space
    float* gauss_coeff_hm_;  // Gaussian coeff in heat map space (0 to 1)
    float uvd_pos_[num_convnet_feats * 3];  // For each feature
    float xyz_pos_[num_convnet_feats * 3];  // For each feature
    jtil::math::LMFit<float>* heat_map_lm_;
    float* lm_fit_x_vals_;  // An image for (u, v) at each grid point
    jtil::math::BFGS<double>* bfgs_; 
    jtil::math::PSOParallel* pso_; 
    jtil::renderer::Camera* camera_;
    jtil::math::Double4x4 pv_mat_;  // Camera contains float, but we need 64bit
    float hand_size_;

    void calcCroppedHand(const int16_t* depth_in, const uint8_t* label_in);
    void calcHPFHandBanks();
    // calcGaussDistCoeff - Fit a gaussian using non-linear least squares to
    // im_data.
    void calcGaussDistCoeff(float* gauss_coeff, const float* im_data);
    void releaseData();  // Call destructor on all dynamic data

    // gauss2D --> x = [u,v], c=[amp, mu_u, mu_v, var_u, var_v]
    static float gauss2D(const float* x, const float* c);  
    static double gauss2D(const double* x, const double* c);  
    static double quad2D(const double* x, const double* c);  
    static double linear2D(const double* x, const double* c);  
    static void jacobGauss2D(float* jacob, const float* x, const float* c);

    static double objFunc(const double* bfgs_hand_coeff);
    static float objFunc(const float* bfgs_hand_coeff);
    static void objFuncParallel(jtil::data_str::Vector<float>& residues, 
      jtil::data_str::Vector<float*>& coeffs);
    static float objFuncInternal();
    static void jacobFunc(double* jacob, const double* bfgs_hand_coeff);
    double cur_double_coeff[HandCoeff::NUM_PARAMETERS];
    double bfgs_coeff_start_[BFGSHandCoeff::BFGS_NUM_PARAMETERS];
    double bfgs_coeff_end_[BFGSHandCoeff::BFGS_NUM_PARAMETERS];
    float pso_coeff_start_[BFGSHandCoeff::BFGS_NUM_PARAMETERS];
    float pso_coeff_end_[BFGSHandCoeff::BFGS_NUM_PARAMETERS];
    float pso_radius_[BFGSHandCoeff::BFGS_NUM_PARAMETERS];

    static HandNet* g_hand_net_;  // BFGS is NOT multithreaded
    template <class T>
    static void BFGSHandCoeffToHandCoeff(T* hand_coeff, 
      const T* bfgs_hand_coeff);
    template <class T>
    static void HandCoeffToBFGSHandCoeff(T* bfgs_hand_coeff, 
      const T* hand_coeff);
    static void renormalizeBFGSCoeffs(double* coeff);
    static void renormalizePSOCoeffs(float* coeff);
    void setPSORadius();
    static float calcPenalty(const float* coeff);
    static double calcPenalty(const double* coeff);
    void calc3DPos(const int16_t* depth, const uint8_t* label);

    // Non-copyable, non-assignable.
    HandNet(HandNet&);
    HandNet& operator=(const HandNet&);
  };

  template <class T>
  void HandNet::BFGSHandCoeffToHandCoeff(T* hand_coeff, 
    const T* bfgs_hand_coeff) {
    hand_coeff[HAND_POS_X] = ((T)100.0 * (bfgs_hand_coeff[BFGS_HAND_POS_X]) / (T)(2.0 * M_PI));
    hand_coeff[HAND_POS_Y] = ((T)100.0 * (bfgs_hand_coeff[BFGS_HAND_POS_Y]) / (T)(2.0 * M_PI));
    hand_coeff[HAND_POS_Z] = ((T)100.0 * (bfgs_hand_coeff[BFGS_HAND_POS_Z]) / (T)(2.0 * M_PI));
    hand_coeff[HAND_ORIENT_X] = bfgs_hand_coeff[BFGS_HAND_ORIENT_X];
    hand_coeff[HAND_ORIENT_Y] = bfgs_hand_coeff[BFGS_HAND_ORIENT_Y];
    hand_coeff[HAND_ORIENT_Z] = bfgs_hand_coeff[BFGS_HAND_ORIENT_Z];
    hand_coeff[THUMB_THETA] = bfgs_hand_coeff[BFGS_THUMB_THETA];
    hand_coeff[THUMB_PHI] = bfgs_hand_coeff[BFGS_THUMB_PHI];
    hand_coeff[THUMB_K1_THETA] = bfgs_hand_coeff[BFGS_THUMB_K1_THETA];
    hand_coeff[THUMB_K1_PHI] = bfgs_hand_coeff[BFGS_THUMB_K1_PHI];
    hand_coeff[THUMB_K2_PHI] = bfgs_hand_coeff[BFGS_THUMB_K2_PHI];
    hand_coeff[F0_THETA] = bfgs_hand_coeff[BFGS_F0_THETA];
    hand_coeff[F0_PHI] = bfgs_hand_coeff[BFGS_F0_PHI];
    hand_coeff[F0_KNUCKLE_MID] = bfgs_hand_coeff[BFGS_F0_CURL];
    hand_coeff[F0_KNUCKLE_END] = bfgs_hand_coeff[BFGS_F0_CURL];
    hand_coeff[F1_THETA] = bfgs_hand_coeff[BFGS_F1_THETA];
    hand_coeff[F1_PHI] = bfgs_hand_coeff[BFGS_F1_PHI];
    hand_coeff[F1_KNUCKLE_MID] = bfgs_hand_coeff[BFGS_F1_CURL];
    hand_coeff[F1_KNUCKLE_END] = bfgs_hand_coeff[BFGS_F1_CURL];
    hand_coeff[F2_THETA] = bfgs_hand_coeff[BFGS_F2_THETA];
    hand_coeff[F2_PHI] = bfgs_hand_coeff[BFGS_F2_PHI];
    hand_coeff[F2_KNUCKLE_MID] = bfgs_hand_coeff[BFGS_F2_CURL];
    hand_coeff[F2_KNUCKLE_END] = bfgs_hand_coeff[BFGS_F2_CURL];
    hand_coeff[F3_THETA] = bfgs_hand_coeff[BFGS_F3_THETA];
    hand_coeff[F3_PHI] = bfgs_hand_coeff[BFGS_F3_PHI];
    hand_coeff[F3_KNUCKLE_MID] = bfgs_hand_coeff[BFGS_F3_CURL];
    hand_coeff[F3_KNUCKLE_END] = bfgs_hand_coeff[BFGS_F3_CURL];
  }

  template <class T>
  void HandNet::HandCoeffToBFGSHandCoeff(T* bfgs_hand_coeff, 
    const T* hand_coeff) {
    T curl;
    bfgs_hand_coeff[BFGS_HAND_POS_X] = (T)(2.0 * M_PI) * (hand_coeff[HAND_POS_X] / (T)100.0);
    bfgs_hand_coeff[BFGS_HAND_POS_Y] = (T)(2.0 * M_PI) * (hand_coeff[HAND_POS_Y] / (T)100.0);
    bfgs_hand_coeff[BFGS_HAND_POS_Z] = (T)(2.0 * M_PI) * (hand_coeff[HAND_POS_Z] / (T)100.0);
    bfgs_hand_coeff[BFGS_HAND_ORIENT_X] = hand_coeff[HAND_ORIENT_X];
    bfgs_hand_coeff[BFGS_HAND_ORIENT_Y] = hand_coeff[HAND_ORIENT_Y];
    bfgs_hand_coeff[BFGS_HAND_ORIENT_Z] = hand_coeff[HAND_ORIENT_Z];
    bfgs_hand_coeff[BFGS_THUMB_THETA] = hand_coeff[THUMB_THETA];
    bfgs_hand_coeff[BFGS_THUMB_PHI] = hand_coeff[THUMB_PHI];
    bfgs_hand_coeff[BFGS_THUMB_K1_THETA] = hand_coeff[THUMB_K1_THETA];
    bfgs_hand_coeff[BFGS_THUMB_K1_PHI] = hand_coeff[THUMB_K1_PHI];
    bfgs_hand_coeff[BFGS_THUMB_K2_PHI] = hand_coeff[THUMB_K2_PHI];
    bfgs_hand_coeff[BFGS_F0_THETA] = hand_coeff[F0_THETA];
    bfgs_hand_coeff[BFGS_F0_PHI] = hand_coeff[F0_PHI];
    curl = (T)0.5 * (hand_coeff[F0_KNUCKLE_MID] + hand_coeff[F0_KNUCKLE_END]);
    bfgs_hand_coeff[BFGS_F0_CURL] = curl;
    bfgs_hand_coeff[BFGS_F1_THETA] = hand_coeff[F1_THETA];
    bfgs_hand_coeff[BFGS_F1_PHI] = hand_coeff[F1_PHI];
    curl = (T)0.5 * (hand_coeff[F1_KNUCKLE_MID] + hand_coeff[F1_KNUCKLE_END]);
    bfgs_hand_coeff[BFGS_F1_CURL] = curl;
    bfgs_hand_coeff[BFGS_F2_THETA] = hand_coeff[F2_THETA];
    bfgs_hand_coeff[BFGS_F2_PHI] = hand_coeff[F2_PHI];
    curl = (T)0.5 * (hand_coeff[F2_KNUCKLE_MID] + hand_coeff[F2_KNUCKLE_END]);
    bfgs_hand_coeff[BFGS_F2_CURL] = curl;
    bfgs_hand_coeff[BFGS_F3_THETA] = hand_coeff[F3_THETA];
    bfgs_hand_coeff[BFGS_F3_PHI] = hand_coeff[F3_PHI];
    curl = (T)0.5 * (hand_coeff[F3_KNUCKLE_MID] + hand_coeff[F3_KNUCKLE_END]);
    bfgs_hand_coeff[BFGS_F3_CURL] = curl;
  }

  
};  // namespace hand_net
};  // namespace kinect_interface_primesense
