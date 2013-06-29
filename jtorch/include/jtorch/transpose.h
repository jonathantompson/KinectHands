//
//  transpose.h
//
//  Created by Jonathan Tompson on 4/9/13.
//
//  NOTE: Transpose is NOT implemented, it just passes the data through
//

#pragma once

#include <iostream>
#include <iomanip>
#include <fstream>
#include "jtil/math/math_types.h"
#include "jtorch/torch_stage.h"

namespace jtil { namespace threading { class ThreadPool; } }

namespace jtorch {
  
  class Transpose : public TorchStage {
  public:
    // Constructor / Destructor
    Transpose();
    virtual ~Transpose();

    virtual TorchStageType type() const { return TRANSPOSE_STAGE; }
    virtual void forwardProp(TorchData& input);

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:

    // Non-copyable, non-assignable.
    Transpose(Transpose&);
    Transpose& operator=(const Transpose&);
  };

};  // namespace jtorch
