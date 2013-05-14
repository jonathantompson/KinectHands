#include "jtorch/join_table.h"
#include "jtorch/float_tensor.h"
#include "jtorch/table.h"
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

namespace jtorch {

  JoinTable::JoinTable() {
    output = NULL;
  }

  JoinTable::~JoinTable() {
    SAFE_DELETE(output);
  }


  TorchStage* JoinTable::loadFromFile(std::ifstream& file) {
    int32_t output_dim;
    file.read((char*)(&output_dim), sizeof(output_dim));
    // But we don't really use it...
    return new JoinTable();
  }

  void JoinTable::init(TorchData& input, jtil::threading::ThreadPool& tp) {
    if (input.type() != TorchDataType::TABLE_DATA) {
      throw std::wruntime_error("Parallel::forwardProp() - "
        "Table expected!");
    }
    Table& in = (Table&)input;

    if (in.tableSize() == 0) {
      throw std::wruntime_error("Parallel::forwardProp() - "
        "Empty input Table!");
    }

    // Check that it is a table of FloatTensors (since tables can be nested)
    for (uint32_t i = 0; i < in.tableSize(); i++) {
      if (in(i)->type() != FLOAT_TENSOR_DATA) {
        throw std::wruntime_error("Parallel::forwardProp() - "
          "Table of float tensors expected!");
      }
    }
    
    const Int4& dim_0 = ((FloatTensor*)in(0))->dim();
    Int4 out_dim(dim_0);
    if (in.tableSize() > 1) {
      // We also need the higher dimensions to all be the same: ie, we're going
      // to concat the lowest dimension
      for (uint32_t i = 1; i < in.tableSize(); i++) {
        const Int4& dim_i = ((FloatTensor*)in(i))->dim();
        if (dim_i[1] != dim_0[1] || dim_i[1] != dim_0[1] ||
          dim_i[1] != dim_0[1]) {
          throw std::wruntime_error("Parallel::forwardProp() - "
            "Tensor dimension mismatch!");
        }
        out_dim[0] += dim_i[0];
      }
    }
    
    if (output != NULL) {
      if (!Int4::equal(((FloatTensor*)output)->dim(), out_dim)) {
        // Input dimension size changed!
        SAFE_DELETE(output);
      }
    }

    if (output == NULL) {
      output = new FloatTensor(out_dim);
    }
  }

  void JoinTable::forwardProp(TorchData& input, ThreadPool& tp) {
    init(input, tp);
    Table& in = (Table&)input;
    FloatTensor& out = (FloatTensor&)*output;
    for (int32_t b = 0; b < out.dim()[3]; b++) {
      for (int32_t f = 0; f < out.dim()[2]; f++) {
        for (int32_t v = 0; v < out.dim()[1]; v++) {
          float* start = &out(0, v, f, b);
          for (uint32_t i = 0; i < in.tableSize(); i++) {
            FloatTensor* cur_data = (FloatTensor*)in(i);
            memcpy(start, &(*cur_data)(0, v, f, b), cur_data->dim()[0] *
              sizeof(start[0]));
            start = &start[cur_data->dim()[0]];  // Move ptr foward
          }
#if defined(DEBUG) || defined(_DEBUG)
          if (start != &(&out(0, v, f, b))[out.dim()[0]]) {
            throw std::wruntime_error("JoinTable::forwardProp() - ERROR: "
              "Internal Pointer arithmetic error!");
          }
#endif
        }
      }
    }
  }

}  // namespace jtorch