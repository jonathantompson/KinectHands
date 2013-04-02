//
//  nn_stage.h
//
//  Created by Jonathan Tompson on 2/10/13.
//
//  Container to hold the neural network stage data (weights, sizing, etc)
//

#ifndef KINECT_INTERFACE_HAND_NET_NN_STAGE_HEADER
#define KINECT_INTERFACE_HAND_NET_NN_STAGE_HEADER

#include <mutex>
#include <condition_variable>
#include <fstream>
#include "jtil/math/math_types.h"
#include "jtil/threading/callback.h"

#define NN_MAX_PRINT_LENGTH 10

namespace jtil { namespace threading { class ThreadPool; } }

namespace kinect_interface {
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

    void forwardProp(float*& in, float*& out, jtil::threading::ThreadPool* tp);
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
    float* bias_;

    // MULTITHREADING
    uint32_t threads_finished_;
    std::mutex thread_update_lock_;
    std::condition_variable not_finished_;
    // One per conv output feat:
    jtil::threading::Callback<void>** linnet_thread_cbs_;  
    jtil::threading::Callback<void>** nonlin_thread_cbs_;
    const float* cur_in_;
    float* cur_out_;

    void performNonlin(float*&data, jtil::threading::ThreadPool* tp);
    void performNonlinRange(const int32_t start, const int32_t end);
    void performLinearNet(float*&out, const float*& in, 
      jtil::threading::ThreadPool* tp);
    void performLinearNetRange(const int32_t start, const int32_t end);

    // Non-copyable, non-assignable.
    NNStage(NNStage&);
    NNStage& operator=(const NNStage&);
  };
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_NN_STAGE_HEADER
