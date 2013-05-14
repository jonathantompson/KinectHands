//
//  sequential.h
//
//  Created by Jonathan Tompson on 4/2/13.
//

#ifndef JTORCH_SEQUENTIAL_HEADER
#define JTORCH_SEQUENTIAL_HEADER

#include <iostream>
#include <iomanip>
#include <fstream>
#include "jtil/math/math_types.h"
#include "jtorch/torch_stage.h"

namespace jtil { namespace threading { class ThreadPool; } }
namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace jtorch {
  
  class Sequential : public TorchStage {
  public:
    // Constructor / Destructor
    Sequential();
    virtual ~Sequential();

    virtual TorchStageType type() const { return SEQUENTIAL_STAGE; }
    virtual void forwardProp(TorchData& input);

    void add(TorchStage* stage);
    TorchStage* get(const uint32_t i);

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    jtil::data_str::VectorManaged<TorchStage*>* network_;

    // Non-copyable, non-assignable.
    Sequential(Sequential&);
    Sequential& operator=(const Sequential&);
  };

};  // namespace jtorch

#endif  // JTORCH_SEQUENTIAL_HEADER
