//
//  conv_stage.h
//
//  Created by Jonathan Tompson on 2/10/13.
//
//  Container to hold the convolution stage data (weights, sizing, etc)
//

#ifndef KINECT_INTERFACE_HAND_NET_CONV_STAGE_HEADER
#define KINECT_INTERFACE_HAND_NET_CONV_STAGE_HEADER

#include <fstream>
#include "jtil/math/math_types.h"

#define MAX_PRINT_LENGTH 10

namespace kinect_interface {
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

    void forwardProp(float*&in, const int32_t inw, const int32_t inh, 
      float*& out);
    void loadFromFile(std::ifstream& file);
    void printToStdOut() const;
    // Calculate the temp data size requirement: 
    const int32_t dataSizeReq(const int32_t inw, const int32_t inh) const;  

    const int32_t filt_width() const { return filt_width_; }
    const int32_t filt_height() const { return filt_height_; }
    const int32_t pool_size() const { return pool_size_; }
    const int32_t n_output_features() const { return n_output_features_; }

    const int32_t calcOutWidth(const int32_t inw) const;
    const int32_t calcOutHeight(const int32_t inh) const;

    const int32_t calcIntermWidth(const int32_t inw) const;
    const int32_t calcIntermHeight(const int32_t inh) const;

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
    float* biases_;
    int16_t** conn_table_;

    float* norm_coef_;  // adjustment coeff (result of filtering image of 1s)
    int32_t norm_coef_w;
    int32_t norm_coef_h;
    float* norm_accum_;
    float* norm_filt_tmp_;

    const static float norm_1dkernel_[7];
    const static int32_t norm_kernel_size_;

    void performSpacialConvolution(float*&in, const int32_t inw, 
      const int32_t inh, float*& out) const;
    void performNonlinearity(float*&data, const int32_t w, 
      const int32_t h) const;
    void performPooling(float*&in, const int32_t inw, 
      const int32_t inh, float*& out) const;
    void performNormalization(float*&in, const int32_t inw, 
      const int32_t inh, float*& out) const;
    void initNormCoef(const int32_t inw, const int32_t inh);

    // Non-copyable, non-assignable.
    ConvStage(ConvStage&);
    ConvStage& operator=(const ConvStage&);
  };

};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_CONV_STAGE_HEADER
