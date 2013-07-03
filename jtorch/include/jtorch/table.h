//
//  table.h
//
//  Created by Jonathan Tompson on 4/1/13.
//
//  Simplified C++ replica of a lua table.  It was implemented so that parallel
//  modules work.
//

#pragma once

#include <iostream>
#include <iomanip>
#include <fstream>
#include "jtil/math/math_types.h"
#include "jtorch/torch_data.h"

namespace jtil { namespace threading { class ThreadPool; } }
namespace jtil { namespace data_str { template <typename T> class VectorManaged; } }

namespace jtorch {
  
  class Table : public TorchData {
  public:
    // Constructor / Destructor
    Table();  // Create an empty table
    virtual ~Table();

    TorchData* operator()(const uint32_t i);
    void add(TorchData* new_data);  // Transfers memory ownership

    // clearNoDelete - Clear the table but don't delete the memory.  This is a
    // hacky function for use in the Parallel stage.
    void clearNoDelete();  

    virtual TorchDataType type() const { return TABLE_DATA; }
    virtual void print();  // print to std::cout
    virtual uint32_t dataSize() const;
    
    uint32_t tableSize() const;

  protected:
    jtil::data_str::VectorManaged<TorchData*>* data_;  // Internal data

    // Non-copyable, non-assignable.
    Table(Table&);
    Table& operator=(const Table&);
  };

};  // namespace jtorch
