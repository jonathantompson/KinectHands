//
//  transpose.h
//
//  Created by Jonathan Tompson on 4/9/13.
//
//  NOTE: Transpose is NOT implemented, it just passes the data through
//

#ifndef KINECT_INTERFACE_HAND_NET_TRANSPOSE_HEADER
#define KINECT_INTERFACE_HAND_NET_TRANSPOSE_HEADER

#include <iostream>
#include <iomanip>
#include <fstream>
#include "jtil/math/math_types.h"
#include "kinect_interface/hand_net/torch_stage.h"

namespace jtil { namespace threading { class ThreadPool; } }

namespace kinect_interface {
namespace hand_net {
  
  class Transpose : public TorchStage {
  public:
    // Constructor / Destructor
    Transpose();
    virtual ~Transpose();

    virtual TorchStageType type() const { return TRANSPOSE_STAGE; }
    virtual void forwardProp(TorchData& input, 
      jtil::threading::ThreadPool& tp);

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:

    // Non-copyable, non-assignable.
    Transpose(Transpose&);
    Transpose& operator=(const Transpose&);
  };

};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_TRANSPOSE_HEADER

