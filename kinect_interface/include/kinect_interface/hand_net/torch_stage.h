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
    SEQUENTIAL_STAGE = 1,
    PARALLEL_STAGE = 2,
    TANH_STAGE = 3,
    THRESHOLD_STAGE = 4,
    LINEAR_STAGE = 5,
    RESHAPE_STAGE = 6,
    SPATIAL_CONVOLUTION_MAP_STAGE = 7,
    SPATIAL_LP_POOLING_STAGE = 8,
    SPATIAL_MAX_POOLING_STAGE = 9,
    SPATIAL_SUBTRACTIVE_NORMALIZATION_STAGE = 10,
    SPATIAL_DIVISIVE_NORMALIZATION_STAGE = 11,
    SPATIAL_CONTRASTIVE_NORMALIZATION_STAGE = 12,
    JOIN_TABLE_STAGE = 13,
    JOIN_TRANSPOSE_STAGE = 14,  // Not actually implemented.  
                                // Data just passes straight through.
  } TorchStageType;

  class FloatTensor;
  
  class TorchStage {
  public:
    // Constructor / Destructor
    TorchStage();
    virtual ~TorchStage();

    virtual TorchStageType type() const { return UNDEFINED_STAGE; }
    virtual void forwardProp(FloatTensor& input, 
      jtil::threading::ThreadPool& tp) = 0;  // Pure virtual

    // Top level read-write
    static TorchStage* loadFromFile(const std::string& file);

    // Everyone must define an output structure
    FloatTensor* output;

  protected:
    static TorchStage* loadFromFile(std::ifstream& file);

    // Non-copyable, non-assignable.
    TorchStage(TorchStage&);
    TorchStage& operator=(const TorchStage&);
  };

};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_TORCH_STAGE_HEADER
