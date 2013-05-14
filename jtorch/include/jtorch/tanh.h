//
//  tanh.h
//
//  Created by Jonathan Tompson on 4/1/13.
//

#ifndef JTORCH_TANH_HEADER
#define JTORCH_TANH_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "jtorch/torch_stage.h"

namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace jtorch {
  
  class Tanh : public TorchStage {
  public:
    // Constructor / Destructor
    Tanh();
    virtual ~Tanh();

    virtual TorchStageType type() const { return TANH_STAGE; }
    virtual void forwardProp(TorchData& input, 
      jtil::threading::ThreadPool& tp);

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    // Multithreading primatives and functions
    float* cur_input_;
    float* cur_output_;
    int32_t threads_finished_;
    std::mutex thread_update_lock_;
    std::condition_variable not_finished_;
    jtil::data_str::VectorManaged<jtil::threading::Callback<void>*>* thread_cbs_; 

    void forwardPropThread(const int32_t start, const int32_t end);

    void init(TorchData& input, jtil::threading::ThreadPool& tp);

    // Non-copyable, non-assignable.
    Tanh(Tanh&);
    Tanh& operator=(const Tanh&);
  };
  
};  // namespace jtorch

#endif  // JTORCH_TANH_HEADER
