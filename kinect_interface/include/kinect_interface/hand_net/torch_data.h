//
//  torch_data.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  This is the base class that other data classes derive from.
//

#ifndef KINECT_INTERFACE_HAND_NET_TORCH_DATA_HEADER
#define KINECT_INTERFACE_HAND_NET_TORCH_DATA_HEADER

#include <iostream>
#include <iomanip>
#include <fstream>
#include "jtil/math/math_types.h"

namespace jtil { namespace threading { class ThreadPool; } }

namespace kinect_interface {
namespace hand_net {

  typedef enum {
    UNDEFINED_DATA = 0,
    TABLE_DATA = 1,
    FLOAT_TENSOR_DATA = 2,
  } TorchDataType;

  class TorchData {
  public:
    // Constructor / Destructor
    TorchData();
    virtual ~TorchData();

    virtual TorchDataType type() const { return UNDEFINED_DATA; }
    virtual uint32_t dataSize() const = 0;  // Pure virtual
    virtual void print() const = 0;

  protected:

    // Non-copyable, non-assignable.
    TorchData(TorchData&);
    TorchData& operator=(const TorchData&);
  };

};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_TORCH_DATA_HEADER
