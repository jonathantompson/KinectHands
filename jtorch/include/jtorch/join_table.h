//
//  join_table.h
//
//  Created by Jonathan Tompson on 4/9/13.
//
//  NOTE: This version of Join Table ALWAYS joins along dimension 0
//

#ifndef JTORCH_JOIN_TABLE_HEADER
#define JTORCH_JOIN_TABLE_HEADER

#include <iostream>
#include <iomanip>
#include <fstream>
#include "jtil/math/math_types.h"
#include "jtorch/torch_stage.h"

namespace jtil { namespace threading { class ThreadPool; } }

namespace jtorch {
  
  class JoinTable : public TorchStage {
  public:
    // Constructor / Destructor
    JoinTable();
    virtual ~JoinTable();

    virtual TorchStageType type() const { return JOIN_TABLE_STAGE; }
    virtual void forwardProp(TorchData& input, 
      jtil::threading::ThreadPool& tp);

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    void init(TorchData& input, jtil::threading::ThreadPool& tp);

    // Non-copyable, non-assignable.
    JoinTable(JoinTable&);
    JoinTable& operator=(const JoinTable&);
  };

};  // namespace jtorch

#endif  // JTORCH_JOIN_TABLE_HEADER
