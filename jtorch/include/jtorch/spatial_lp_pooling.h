//
//  spatial_lp_pooling.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  Multithreading is not all that efficient:  Threads are split up per output 
//  feature.
//

#ifndef JTORCH_SPATIAL_LP_POOLING_HEADER
#define JTORCH_SPATIAL_LP_POOLING_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "jtorch/torch_stage.h"

namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace jtorch {
  
  class SpatialLPPooling : public TorchStage {
  public:
    // Constructor / Destructor
    SpatialLPPooling(const float p_norm, const int32_t poolsize_v, 
      const int32_t poolsize_u);
    virtual ~SpatialLPPooling();

    virtual TorchStageType type() const { return SPATIAL_LP_POOLING_STAGE; }
    virtual void forwardProp(TorchData& input, 
      jtil::threading::ThreadPool& tp);

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    float p_norm_;
    int32_t poolsize_v_;
    int32_t poolsize_u_;

    void forwardPropThread(const int32_t outf, const int32_t outb);

    void init(TorchData& input, jtil::threading::ThreadPool& tp);

    // Non-copyable, non-assignable.
    SpatialLPPooling(SpatialLPPooling&);
    SpatialLPPooling& operator=(const SpatialLPPooling&);
  };
  
};  // namespace jtorch

#endif  // JTORCH_SPATIAL_LP_POOLING_HEADER
