//
//  conv_stage.h
//
//  Created by Jonathan Tompson on 2/10/13.
//
//  Container to hold the convolution stage data (weights, sizing, etc)
//

#ifndef HAND_NET_CONV_STAGE_HEADER
#define HAND_NET_CONV_STAGE_HEADER

#include <fstream>
#include "math/math_types.h"

#define MAX_PRINT_LENGTH 2

namespace hand_net {
  
  typedef enum {
    SubtractiveNorm = 0, 
    UndefinedNorm = 1,
  } NormType;

  typedef enum {
    L2Pool = 2, 
    L4Pool = 4,
    LInfPool = 0,
    UndefinedPool = 3,
  } PoolType;

  typedef enum {
    TanhNonlin = 0, 
    SoftShrinkNonlin = 1,
    UndefinedNonlin = 2,
  } NonlinType;
  
  class ConvStage {
  public:
    // Constructor / Destructor
    ConvStage();
    ~ConvStage();

    void loadFromFile(std::ifstream& file);
    void printToStdOut() const;
    // Calculate the temp data size requirement: 
    const int32_t dataSizeReq(int32_t inw, int32_t inh) const;  

    const int32_t filt_width() const { return filt_width_; }
    const int32_t filt_height() const { return filt_height_; }
    const int32_t pool_size() const { return pool_size_; }
    const int32_t n_output_features() const { return n_output_features_; }

    const int32_t calc_out_im_width(int32_t inw) const;
    const int32_t calc_out_im_height(int32_t inh) const;

  private:
    int32_t filt_width_;
    int32_t filt_height_;
    int32_t n_input_features_;
    int32_t n_output_features_;
    int32_t filt_fan_in_;
    NormType norm_type_;
    PoolType pool_type_;
    int32_t pool_size_;
    NonlinType nonlin_type_;

    float** weights_;
    int16_t** conn_table_;

    // Non-copyable, non-assignable.
    ConvStage(ConvStage&);
    ConvStage& operator=(const ConvStage&);
  };
};  // hand_net namespace

#endif  // HAND_NET_CONV_STAGE_HEADER
