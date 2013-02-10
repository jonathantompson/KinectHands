//
//  nn_stage.h
//
//  Created by Jonathan Tompson on 2/10/13.
//
//  Container to hold the neural network stage data (weights, sizing, etc)
//

#ifndef HAND_NET_NN_STAGE_HEADER
#define HAND_NET_CONV_STAGE_HEADER

#include <fstream>
#include "math/math_types.h"

#define NN_MAX_PRINT_LENGTH 10

namespace hand_net {

  typedef enum {
    TanhNNNonlin = 0, 
    NoneNNNonlin = 1,
    UndefinedNNNonlin = 2,
  } NNNonlinType;
  
  class NNStage {
  public:
    // Constructor / Destructor
    NNStage();
    ~NNStage();

    void loadFromFile(std::ifstream& file);
    void printToStdOut() const;
    int32_t dataSizeReq() const;  // Calculate the temp data size requirement

    const int32_t n_inputs() const { return n_inputs_; }
    const int32_t n_outputs() const { return n_outputs_; }

  private:
    int32_t n_inputs_;
    int32_t n_outputs_;
    NNNonlinType nonlin_type_;

    float* weights_;

    // Non-copyable, non-assignable.
    NNStage(NNStage&);
    NNStage& operator=(const NNStage&);
  };
};  // hand_net namespace

#endif  // HAND_NET_CONV_STAGE_HEADER
