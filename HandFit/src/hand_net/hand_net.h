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

#define HAND_NET_PIX 192  // U, V size (before downsampling)
#define HAND_NET_DOWN_FACT 2
#define HAND_NET_IM_SIZE (HAND_NET_PIX / HAND_NET_DOWN_FACT)

namespace hand_net {
  class ConvStage;
  class NNStage;
  
  class HandNet {
  public:
    // Constructor / Destructor
    HandNet(const std::string& convnet_filename);
    ~HandNet();

    void calcHandCoeff(const float* depth, float* coeff);
    static void calcHandImageFromSytheticDepth(const float* depth_in, 
      float* hand_im_depth, float* hand_im_xyx = NULL, float* x_com = NULL,
      float* y_com = NULL, float* z_com = NULL, float* std = NULL);

    // Modifiers

  private:
    int32_t n_conv_stages_;
    ConvStage** conv_stages_;

    int32_t n_nn_stages_;
    NNStage** nn_stages_;

    float* datcur_;  // The current stage's input data
    float* datnext_;  // The current stage's output data

    // Some temporary data structures (might be able to clean some of these
    // up later when we're no longer using synthetic data)
    static int16_t depth[src_dim];
    static float xyz[src_dim * 3];
    static float cropped_depth[HAND_NET_PIX * HAND_NET_PIX];
    static float cropped_xyz[3 * HAND_NET_PIX * HAND_NET_PIX];

    void loadFromFile(const std::string& convnet_filename);

    // Non-copyable, non-assignable.
    HandNet(HandNet&);
    HandNet& operator=(const HandNet&);
  };
};  // hand_net namespace

#endif  // HAND_NET_HAND_NET_HEADER
