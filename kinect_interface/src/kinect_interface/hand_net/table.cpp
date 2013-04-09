#include <sstream>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include "kinect_interface/hand_net/table.h"
#include "jtil/data_str/vector_managed.h"
#include "jtil/exceptions/wruntime_error.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::math;

namespace kinect_interface {
namespace hand_net {

  Table::Table() {
    data_ = new jtil::data_str::VectorManaged<TorchData*>();
  }

  Table::~Table() {
    delete data_;
  }

  TorchData* Table::operator()(const uint32_t i) {
    return (*data_)[i];
  }

  void Table::print() const {
    for (uint32_t i = 0; i < data_->size(); i++) {
      std::cout << "Table[" << i << "] = " << std::endl;
      (*data_)[i]->print();
    }
  };

  void Table::add(TorchData* new_data) {
    data_->pushBack(new_data);
  }

  uint32_t Table::dataSize() const {
    return data_->size();
  }

  uint32_t Table::tableSize() const {
    return data_->size();
  }

}  // namespace hand_net
}  // namespace kinect_interface
