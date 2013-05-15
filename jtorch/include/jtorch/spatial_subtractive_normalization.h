//
//  spatial_subtractive_normalization.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  This stage is only partially multi-threaded!
//

#ifndef JTORCH_SPATIAL_SUBTRACTIVE_NORMALIZATION_HEADER
#define JTORCH_SPATIAL_SUBTRACTIVE_NORMALIZATION_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "jtorch/torch_stage.h"

namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace jtorch {

  template <typename T> class Tensor;
  
  class SpatialSubtractiveNormalization : public TorchStage {
  public:
    // Constructor / Destructor
    SpatialSubtractiveNormalization(const Tensor<float>& kernel1d);
    virtual ~SpatialSubtractiveNormalization();

    virtual TorchStageType type() const { return SPATIAL_SUBTRACTIVE_NORMALIZATION_STAGE; }
    virtual void forwardProp(TorchData& input);

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    Tensor<float>* kernel1d_;
    Tensor<float>* mean_coef_;

    //float* mean_coef_;
    //float* mean_accum_;
    //float* filt_tmp_;

    void normalizeFeature(const int32_t feat);

    void init(TorchData& input, jtil::threading::ThreadPool& tp);

    // Non-copyable, non-assignable.
    SpatialSubtractiveNormalization(SpatialSubtractiveNormalization&);
    SpatialSubtractiveNormalization& operator=(const SpatialSubtractiveNormalization&);
  };
  
};  // namespace jtorch

#endif  // JTORCH_SPATIAL_SUBTRACTIVE_NORMALIZATION_HEADER
