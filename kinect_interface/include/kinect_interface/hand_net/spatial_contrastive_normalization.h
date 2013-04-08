//
//  spatial_contrastive_normalization.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  This stage is only partially multi-threaded!
//
//  It is just a SpatialSubtractiveNormalization followed by a 
//  SpatialDivisiveNormalization.  In other words, subtracting off the mean and
//  dividing by the standard deviation.
//
//  This stage is the default for local contrast normalization.
//

#ifndef KINECT_INTERFACE_HAND_SPATIAL_CONTRASTIVE_NORMALIZATION_HEADER
#define KINECT_INTERFACE_HAND_SPATIAL_CONTRASTIVE_NORMALIZATION_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "kinect_interface/hand_net/torch_stage.h"

namespace kinect_interface {
namespace hand_net {
  
  struct Sequential;

  struct SpatialContrastiveNormalization : public TorchStage {
  public:
    // Constructor / Destructor
    // Note if kernel1d is NULL, then a rectangular filter kernel is used
    SpatialContrastiveNormalization(const int32_t n_feats, 
      const int32_t height, const int32_t width, const uint32_t kernel1d_size, 
      const float* kernel1d = NULL, const float threshold = 1e-4f);
    virtual ~SpatialContrastiveNormalization();

    virtual TorchStageType type() const { return SPATIAL_CONTRASTIVE_NORMALIZATION_STAGE; }
    virtual void forwardProp(float* input, jtil::threading::ThreadPool* tp);
    virtual int32_t outWidth() const;
    virtual int32_t outHeight() const;
    virtual int32_t outNFeats() const;

    float* output();  // Override output float*

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    Sequential* network_;

    // Non-copyable, non-assignable.
    SpatialContrastiveNormalization(SpatialContrastiveNormalization&);
    SpatialContrastiveNormalization& operator=(const SpatialContrastiveNormalization&);
  };
  
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_SPATIAL_CONTRASTIVE_NORMALIZATION_HEADER
