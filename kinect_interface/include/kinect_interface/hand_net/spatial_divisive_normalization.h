//
//  spatial_divisive_normalization.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  This stage is only partially multi-threaded!
//

#ifndef KINECT_INTERFACE_HAND_SPATIAL_DIVISIVE_NORMALIZATION_HEADER
#define KINECT_INTERFACE_HAND_SPATIAL_DIVISIVE_NORMALIZATION_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "kinect_interface/hand_net/torch_stage.h"

namespace kinect_interface {
namespace hand_net {
  
  struct SpatialDivisiveNormalization : public TorchStage {
  public:
    // Constructor / Destructor
    SpatialDivisiveNormalization(const int32_t n_feats, 
      const uint32_t kernel1d_size, const float* kernel1d, 
      const int32_t height, const int32_t width, const float threshold = 1e-4f);
    virtual ~SpatialDivisiveNormalization();

    virtual TorchStageType type() const { return SPATIAL_DIVISIVE_NORMALIZATION_STAGE; }
    virtual void forwardProp(float* input, jtil::threading::ThreadPool* tp);
    virtual int32_t outWidth() const { return width_; }
    virtual int32_t outHeight() const { return height_; }
    virtual int32_t outNFeats() const { return n_feats_; }

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    float* kernel1d_;
    int32_t kernel1d_size_;
    float* std_coef_;
    float* std_accum_;
    float* filt_tmp_;
    int32_t n_feats_;
    int32_t width_;
    int32_t height_;
    float threshold_;

    // Multithreading primatives and functions
    float* cur_input_;
    int32_t threads_finished_;
    std::mutex thread_update_lock_;
    std::condition_variable not_finished_;
    jtil::threading::Callback<void>** thread_cbs_;  
    void normalizeFeature(const int32_t outf);

    // Non-copyable, non-assignable.
    SpatialDivisiveNormalization(SpatialDivisiveNormalization&);
    SpatialDivisiveNormalization& operator=(const SpatialDivisiveNormalization&);
  };
  
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_SPATIAL_DIVISIVE_NORMALIZATION_HEADER
