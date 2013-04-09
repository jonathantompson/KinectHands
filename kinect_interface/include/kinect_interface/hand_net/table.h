//
//  table.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  Simplified C++ replica of a lua table.  It was implemented so that parallel
//  modules work.
//

#ifndef KINECT_INTERFACE_HAND_NET_TABLE_HEADER
#define KINECT_INTERFACE_HAND_NET_TABLE_HEADER

#include <iostream>
#include <iomanip>
#include <fstream>
#include "jtil/math/math_types.h"
#include "kinect_interface/hand_net/torch_data.h"

namespace jtil { namespace threading { class ThreadPool; } }
namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace kinect_interface {
namespace hand_net {
  
  class Table : public TorchData {
  public:
    // Constructor / Destructor
    Table();  // Create an empty table
    virtual ~Table();

    TorchData* operator()(const uint32_t i);
    void add(TorchData* new_data);  // Transfers memory ownership

    virtual TorchDataType type() const { return TABLE_DATA; }
    virtual void print() const;  // print to std::cout
    virtual uint32_t dataSize() const;
    
    uint32_t tableSize() const;

  protected:
    jtil::data_str::VectorManaged<TorchData*>* data_;  // Internal data

    // Non-copyable, non-assignable.
    Table(Table&);
    Table& operator=(const Table&);
  };

};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_TABLE_HEADER
