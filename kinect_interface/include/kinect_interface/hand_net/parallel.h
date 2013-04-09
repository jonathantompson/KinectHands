//
//  parallel.h
//
//  Created by Jonathan Tompson on 4/8/13.
//
//  Note: For now the output of each network in the parallel stage must be
//  a FloatTensor.
//

#ifndef KINECT_INTERFACE_HAND_NET_PARALLEL_HEADER
#define KINECT_INTERFACE_HAND_NET_PARALLEL_HEADER

#include <iostream>
#include <iomanip>
#include <fstream>
#include "jtil/math/math_types.h"
#include "kinect_interface/hand_net/torch_stage.h"

namespace jtil { namespace threading { class ThreadPool; } }
namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace kinect_interface {
namespace hand_net {
  
  class Parallel : public TorchStage {
  public:
    // Constructor / Destructor
    Parallel();
    virtual ~Parallel();

    virtual TorchStageType type() const { return PARALLEL_STAGE; }
    virtual void forwardProp(TorchData& input, 
      jtil::threading::ThreadPool& tp);

    void add(TorchStage* stage);

    uint32_t numBanks() const;

    static TorchStage* loadFromFile(std::ifstream& file);

  protected:
    jtil::data_str::VectorManaged<TorchStage*>* network_;

    // Non-copyable, non-assignable.
    Parallel(Parallel&);
    Parallel& operator=(const Parallel&);
  };

};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_PARALLEL_HEADER

