//
//  reshape.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  Works a little differently to the torch version...  It takes a 3D tensor
//  and just makes it into a 1D array, so it's not as general purpose.
//
//  But really this 1D array is just a straight copy of the input data (since
//  we define tensors as float* anyway).
//

#ifndef KINECT_INTERFACE_HAND_NET_RESHAPE_HEADER
#define KINECT_INTERFACE_HAND_NET_RESHAPE_HEADER

#include <mutex>
#include <condition_variable>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"
#include "kinect_interface/hand_net/torch_stage.h"

namespace kinect_interface {
namespace hand_net {
  
  class Reshape : public TorchStage {
  public:
    // Constructor / Destructor
    Reshape();
    virtual ~Reshape();

    virtual TorchStageType type() const { return RESHAPE_STAGE; }
    virtual void forwardProp(FloatTensor& input, 
      jtil::threading::ThreadPool& tp);

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    void init(FloatTensor& input, jtil::threading::ThreadPool& tp);

    // Non-copyable, non-assignable.
    Reshape(Reshape&);
    Reshape& operator=(const Reshape&);
  };
  
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_RESHAPE_HEADER
