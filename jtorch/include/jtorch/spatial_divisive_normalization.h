//
//  spatial_divisive_normalization.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  This stage is only partially multi-threaded!
//
//  Note that just like the torch stage, this divisive stage assumes zero
//  input mean.  That is, it does not subtract off the mean per element when 
//  estimating the standard deviation.
//

#ifndef JTORCH_SPATIAL_DIVISIVE_NORMALIZATION_HEADER
#define JTORCH_SPATIAL_DIVISIVE_NORMALIZATION_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "jtorch/torch_stage.h"

namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace jtorch {

  class FloatTensor;
  
  class SpatialDivisiveNormalization : public TorchStage {
  public:
    // Constructor / Destructor
    SpatialDivisiveNormalization(const FloatTensor& kernel1d, 
      const float threshold = 1e-4f);
    virtual ~SpatialDivisiveNormalization();

    virtual TorchStageType type() const { return SPATIAL_DIVISIVE_NORMALIZATION_STAGE; }
    virtual void forwardProp(TorchData& input, 
      jtil::threading::ThreadPool& tp);

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    FloatTensor* kernel1d_;
    FloatTensor* kernel1d_norm_;  // kernel normalization depends on input size
    float* std_coef_;
    float* std_accum_;
    float* filt_tmp_;
    float threshold_;

    // Multithreading primatives and functions
    float* cur_input_;
    float* cur_output_;
    int32_t threads_finished_;
    std::mutex thread_update_lock_;
    std::condition_variable not_finished_;
    jtil::data_str::VectorManaged<jtil::threading::Callback<void>*>* thread_cbs_; 

    void normalizeFeature(const int32_t outf);

    void init(TorchData& input, jtil::threading::ThreadPool& tp);

    // Non-copyable, non-assignable.
    SpatialDivisiveNormalization(SpatialDivisiveNormalization&);
    SpatialDivisiveNormalization& operator=(const SpatialDivisiveNormalization&);
  };
  
};  // namespace jtorch

#endif  // JTORCH_SPATIAL_DIVISIVE_NORMALIZATION_HEADER
