//
//  linear.h
//
//  Created by Jonathan Tompson on 4/1/13.
//

#ifndef JTORCH_LINEAR_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "jtorch/torch_stage.h"

namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace jtorch {

  template <typename T> class Tensor;
  
  class Linear : public TorchStage {
  public:
    // Constructor / Destructor
    Linear(const int32_t n_inputs, const int32_t n_outputs);
    virtual ~Linear();

    virtual TorchStageType type() const { return LINEAR_STAGE; }
    virtual void forwardProp(TorchData& input);

    float* weights;
    float* bias;

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    int32_t n_inputs_;
    int32_t n_outputs_;

    void init(TorchData& input, jtil::threading::ThreadPool& tp);

    // Non-copyable, non-assignable.
    Linear(Linear&);
    Linear& operator=(const Linear&);
  };
  
};  // namespace jtorch

#endif  // JTORCH_LINEAR_HEADER
