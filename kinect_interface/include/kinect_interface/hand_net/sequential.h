//
//  sequential.h
//
//  Created by Jonathan Tompson on 4/2/13.
//

#ifndef KINECT_INTERFACE_HAND_NET_SEQUENTIAL_HEADER
#define KINECT_INTERFACE_HAND_NET_SEQUENTIAL_HEADER

#include <iostream>
#include <iomanip>
#include <fstream>
#include "jtil/math/math_types.h"
#include "kinect_interface/hand_net/torch_stage.h"

namespace jtil { namespace threading { class ThreadPool; } }
namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace kinect_interface {
namespace hand_net {
  
  class Sequential : public TorchStage {
  public:
    // Constructor / Destructor
    Sequential();
    virtual ~Sequential();

    virtual TorchStageType type() const { return SEQUENTIAL_STAGE; }
    virtual void forwardProp(TorchData& input, 
      jtil::threading::ThreadPool& tp);

    void add(TorchStage* stage);
    TorchStage* get(const uint32_t i);

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    jtil::data_str::VectorManaged<TorchStage*>* network_;

    // Non-copyable, non-assignable.
    Sequential(Sequential&);
    Sequential& operator=(const Sequential&);
  };

};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_SEQUENTIAL_HEADER
