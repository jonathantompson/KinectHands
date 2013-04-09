#include "kinect_interface/hand_net/parallel.h"
#include "kinect_interface/hand_net/float_tensor.h"
#include "kinect_interface/hand_net/table.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/threading/thread.h"
#include "jtil/threading/callback.h"
#include "jtil/threading/thread_pool.h"
#include "jtil/data_str/vector_managed.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::threading;
using namespace jtil::math;
using namespace jtil::data_str;

namespace kinect_interface {
namespace hand_net {

  Parallel::Parallel() {
    // Create an empty container
    network_ = new VectorManaged<TorchStage*>(1);
  }

  Parallel::~Parallel() {
    SAFE_DELETE(network_);
    SAFE_DELETE(output);
  }

  void Parallel::add(TorchStage* stage) {
    network_->pushBack(stage);
    output = NULL;
  }

  TorchStage* Parallel::loadFromFile(std::ifstream& file) {
    int n_nodes;
    file.read(reinterpret_cast<char*>(&n_nodes), sizeof(n_nodes));
    Parallel* ret = new Parallel();
    ret->network_->capacity(n_nodes);
    for (int32_t i = 0; i < n_nodes; i++) {
      ret->network_->pushBack(TorchStage::loadFromFile(file));
    }
    return ret;
  }

  void Parallel::initOutput() {
    if (output != NULL) {
      // Check output sizes
      Table* out = (Table*)output;
      bool data_ok = out->tableSize() == network_->size();
      for (uint32_t i = 0; i < out->tableSize() && data_ok; i++) {
        if ((*network_)[i]->output->type() != FLOAT_TENSOR_DATA) {
          throw std::wruntime_error("Parallel::forwardProp() - ERROR: "
            "Float Tensor output required!");
        }
        if (!Int4::equal(((FloatTensor*)((*network_)[i])->output)->dim(), 
          ((FloatTensor*)(*out)(i))->dim())) {
          data_ok = false;
        }
      }
      if (!data_ok) {
        SAFE_DELETE(output);
      }
    }
    if (output == NULL) {
      output = new Table();
      Table* out = (Table*)output;
      for (uint32_t i = 0; i < network_->size(); i++) {
        if ((*network_)[i]->output->type() != FLOAT_TENSOR_DATA) {
          throw std::wruntime_error("Parallel::forwardProp() - ERROR: "
            "Float Tensor output required for now!");
        }
        out->add(new FloatTensor(((FloatTensor*)(*network_)[i]->output)->dim()));
      }
    }
  }

  void Parallel::forwardProp(TorchData& input, 
    ThreadPool& tp) {
    if (input.type() != TorchDataType::TABLE_DATA) {
      throw std::wruntime_error("Parallel::forwardProp() - "
        "Table expected!");
    }
    Table& in = (Table&)input;
    if (in.tableSize() != network_->size()) {
      throw std::wruntime_error("Parallel::forwardProp() - ERROR: "
        "Table size does not match number of parallel stages!");
    }
    for (uint32_t i = 0; i < network_->size(); i++) {
      (*network_)[i]->forwardProp(*in(i), tp);
    }
    initOutput();
    // Now copy the data to the output structure
    Table* out = (Table*)output;
    for (uint32_t i = 0; i < network_->size(); i++) {
      FloatTensor* dst_data = (FloatTensor*)(*out)(i);
      FloatTensor* src_data = (FloatTensor*)(*network_)[i]->output;
      memcpy(dst_data->data(), src_data->data(), dst_data->dataSize());
    }
  }

  uint32_t Parallel::numBanks() const {
    if (network_ == NULL) {
      throw std::wruntime_error("Parallel::output() - ERROR: "
        "Network is empty!");
    }
    return (*network_).size();
  }

}  // namespace hand_net
}  // namespace kinect_interface
