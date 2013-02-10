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

namespace hand_net {
  class ConvStage;
  class NNStage;
  
  class HandNet {
  public:
    // Constructor / Destructor
    HandNet(const std::string& convnet_filename);
    ~HandNet();

    // Accessors

    // Modifiers

  private:
    int32_t n_conv_stages_;
    ConvStage** conv_stages_;

    int32_t n_nn_stages_;
    NNStage** nn_stages_;

    void loadFromFile(const std::string& convnet_filename);

    // Non-copyable, non-assignable.
    HandNet(HandNet&);
    HandNet& operator=(const HandNet&);
  };
};  // hand_net namespace

#endif  // HAND_NET_HAND_NET_HEADER
