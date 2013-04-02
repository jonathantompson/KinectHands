//
//  torch_stage.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  C++ replica of various torch stages.  This is the base class that others
//  derive from.
//

#ifndef KINECT_INTERFACE_HAND_NET_TORCH_STAGE_HEADER
#define KINECT_INTERFACE_HAND_NET_TORCH_STAGE_HEADER

#include <iostream>
#include <iomanip>
#include <fstream>
#include "jtil/math/math_types.h"

namespace jtil { namespace threading { class ThreadPool; } }

namespace kinect_interface {
namespace hand_net {

  typedef enum {
    UNDEFINED_STAGE = 0,
    SPATIAL_CONVOLUTION_MAP_STAGE = 1,
    TANH_STAGE = 2,
    THRESHOLD_STAGE = 3,
    LINEAR_STAGE = 4,
    RESHAPE_STAGE = 5,
    SPATIAL_LP_POOLING_STAGE = 6,
    SPATIAL_MAX_POOLING_STAGE = 7,
    SPATIAL_SUBTRACTIVE_NORMALIZATION = 8,
  } TorchStageType;
  
  struct TorchStage {
  public:
    // Constructor / Destructor
    TorchStage();
    virtual ~TorchStage();

    virtual TorchStageType type() const { return UNDEFINED_STAGE; }
    virtual void forwardProp(float* input, jtil::threading::ThreadPool* tp) = 0;  // Pure virtual
    static TorchStage* loadFromFile(std::ifstream& file) { };
    virtual int32_t outWidth() const = 0;
    virtual int32_t outHeight() const = 0;
    virtual int32_t outNFeats() const = 0;

    // Everyone must define an output structure
    float* output;

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

    // gaussian1D In torch: >> normkernel = image.gaussian1D(n)
    // It's a gaussian of x = -2*sigma to 2*sigma, where sigma = n / 2
    template <typename T>
    static T* gaussian1D(const int32_t kernel_size);

  private:
    // Non-copyable, non-assignable.
    TorchStage(TorchStage&);
    TorchStage& operator=(const TorchStage&);
  };

  template <typename T>
  T* TorchStage::gaussian1D(const int32_t kernel_size) {
    T* ret = new T[kernel_size];
    T sigma = (T)0.25;
    T amplitude = (T)1.0;
    T size = (T)kernel_size;
    T center = size/(T)2.0 + (T)0.5;
    for (int32_t i = 0; i < kernel_size; i++) {
      ret[i] = amplitude * (T)exp(-((T)pow(((T)(i+1)-center) / (sigma*size),
        (T)2.0)/(T)2.0));
    }
    return ret;
  }

  // Some debug functions
  template <typename T>
  void TorchStage::print3DTensorToStdCout(T* data, 
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
  void TorchStage::print3DTensorToStdCout(T* data, const int32_t ifeat,
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
  void TorchStage::print3DTensorToStdCout(T** data, 
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

#endif  // KINECT_INTERFACE_HAND_NET_TORCH_STAGE_HEADER
