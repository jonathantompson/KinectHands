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

#define MAX_PRINT_LENGTH 10

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

    // Some helper functions for debugging
    template <typename T>
    static void print3DTensorToStdOut(const T* data, const int32_t n_feats,
      const int32_t height, const int32_t width);
    // This version just prints one feature and within this just a sub image
    template <typename T>
    static void print3DTensorToStdOut(const T* data, const int32_t ifeat,
      const int32_t height, const int32_t width, const int32_t ustart, 
      const int32_t vstart, const int32_t printw, const int32_t printh);
    template <typename T>
    static void print3DTensorToStdOut(const T** data, const int32_t n_feats,
      const int32_t height, const int32_t width);
    template <typename T>
    static void print2DTensorToStdOut(const T* data, const int32_t height, 
      const int32_t width);
    template <typename T>

    // Non-copyable, non-assignable.
    ConvStage(ConvStage&);
    ConvStage& operator=(const ConvStage&);
  };

  // Some debug functions
  template <typename T>
  void ConvStage::print3DTensorToStdOut(const T* data, 
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
  void ConvStage::print3DTensorToStdOut(const T* data, const int32_t ifeat,
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
  void ConvStage::print3DTensorToStdOut(const T** data, 
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

  template <typename T>
  void ConvStage::print2DTensorToStdOut(const T* data, 
    const int32_t height, const int32_t width) {
    std::cout.setf(0, std::ios::showpos);
    std::cout << "  2dtensor[0] =" << std::endl;
    for (int32_t v = 0; v < height; v++) {
      if (v == 0) {
        std::cout << "    (0,0) ";
      } else {
        std::cout << "          ";
      }
      std::cout.setf(std::ios::showpos);
      for (int32_t u = 0; u < width; u++) {
        std::cout << data[v * width + u];
        if (u != width - 1) {
          std::cout << ", ";
        } else {
          std::cout << std::endl;
        }
      }
    }
    std::cout << std::resetiosflags(std::ios_base::showpos);
  };

};  // hand_net namespace

#endif  // HAND_NET_CONV_STAGE_HEADER
