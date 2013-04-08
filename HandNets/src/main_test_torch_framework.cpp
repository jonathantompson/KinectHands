//
//  main.cpp
//
//  This is a c++ version of the non_linear_least_sq.m file...  The section
//  at the bottom (Leven. Marq.).
//  

#include <stdlib.h>
#include <cmath>
#include <thread>
#include <iostream>
#include <limits>
#include "kinect_interface/hand_net/torch_stage.h"
#include "kinect_interface/hand_net/float_tensor.h"
#include "kinect_interface/hand_net/spatial_convolution_map.h"
#include "kinect_interface/hand_net/spatial_lp_pooling.h"
#include "kinect_interface/hand_net/spatial_max_pooling.h"
#include "kinect_interface/hand_net/spatial_subtractive_normalization.h"
#include "kinect_interface/hand_net/spatial_divisive_normalization.h"
#include "kinect_interface/hand_net/spatial_contrastive_normalization.h"
#include "kinect_interface/hand_net/linear.h"
#include "kinect_interface/hand_net/reshape.h"
#include "kinect_interface/hand_net/tanh.h"
#include "kinect_interface/hand_net/threshold.h"
#include "jtil/threading/thread_pool.h"
#include "jtil/data_str/vector_managed.h"
#include "jtil/debug_util/debug_util.h"
#include "jtil/file_io/file_io.h"

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif

#define NUM_WORKER_THREADS 8

using namespace std;
using namespace kinect_interface::hand_net;
using namespace jtil::threading;
using namespace jtil::math;
using namespace jtil::data_str;

const uint32_t num_feats_in = 5;
const uint32_t num_feats_out = 10;
const uint32_t fan_in = 3;  // For SpatialConvolutionMap
const uint32_t width = 10;
const uint32_t height = 10;
const uint32_t filt_height = 5;
const uint32_t filt_width = 5;
float* data_in;
float* data_out;

int main(int argc, char *argv[]) {  
#if defined(_DEBUG) || defined(DEBUG)
  jtil::debug::EnableMemoryLeakChecks();
  // jtil::debug::EnableAggressiveMemoryLeakChecks();
  // jtil::debug::SetBreakPointOnAlocation(8634);
#endif

  FloatTensor data_in(Uint3(width, height, num_feats_in));
  FloatTensor data_out(Uint3(width, height, num_feats_out));
  ThreadPool tp(NUM_WORKER_THREADS);

  for (uint32_t f = 0; f < num_feats_in; f++) {
    float val = (f+1) - (float)(width * height) / 16.0f;
    for (uint32_t v = 0; v < height; v++) {
      for (uint32_t u = 0; u < width; u++) {
        data_in(u, v, f, 0) = val;
        val += 1.0f / 8.0f;
      }
    }
  }
  std::cout << "Data In:" << std::endl;
  data_in.print();

  VectorManaged<TorchStage*> stages;

  // ***********************************************
  // Test Tanh
  stages.pushBack(new Tanh());
  stages[0]->forwardProp(data_in, tp);
  std::cout << endl << endl << "Tanh output:" << std::endl;
  stages[0]->output->print();

  // ***********************************************
  // Test Threshold
  const float threshold = 0.5f;
  const float val = 0.1f;
  stages.pushBack(new Threshold());
  ((Threshold*)stages[1])->threshold = threshold;
  ((Threshold*)stages[1])->val = val;
  stages[1]->forwardProp(*stages[0]->output, tp);
  std::cout << endl << endl << "Threshold output:" << std::endl;
  stages[1]->output->print();

  /*
  // ***********************************************
  // Test SpatialConvolutionMap
  stage[2] = new SpatialConvolutionMap(num_feats_in, num_feats_out, fan_in,
    filt_height, filt_width, height, width);
  for (int32_t i = 0; i < num_feats_out; i++) {
    ((SpatialConvolutionMap*)stage[2])->biases_[i] = (float)(i+1) / 
      (float)num_feats_out - 0.5f;
  }
  std::cout << "SpatialConvolutionMap biases:" << std::endl;
  TorchStage::print3DTensorToStdCout<float>(((SpatialConvolutionMap*)stage[2])->biases_, 
    1, 1, num_feats_out);
  const float sigma_x_sq = 1.0f;
  const float sigma_y_sq = 1.0f;
  for (int32_t i = 0; i < num_feats_out * fan_in; i++) {
    float scale = ((float)(i + 1) / (float)(num_feats_out * fan_in));
    for (int32_t v = 0; v < filt_height; v++) {
      for (int32_t u = 0; u < filt_width; u++) {
        float x = (float)u - (float)(filt_width-1) / 2.0f;
        float y = (float)v - (float)(filt_height-1) / 2.0f;
        ((SpatialConvolutionMap*)stage[2])->weights_[i][v * filt_width + u] = 
          scale * expf(-((x*x)/(2.0f*sigma_x_sq) + (y*y)/(2.0f*sigma_y_sq)));
      }
    }
  }
  std::cout << "SpatialConvolutionMap weights:" << std::endl;
  TorchStage::print3DTensorToStdCout<float>(((SpatialConvolutionMap*)stage[2])->weights_, 
    num_feats_out * fan_in, filt_height, filt_width);
  int32_t cur_filt = 0;
  for (int32_t f_out = 0; f_out < num_feats_out; f_out++) {
    for (int32_t f_in = 0; f_in < fan_in; f_in++) {
      ((SpatialConvolutionMap*)stage[2])->conn_table_[f_out][f_in * 2 + 1] = cur_filt;
      int32_t cur_f_in = (f_out + f_in) % num_feats_in;
      ((SpatialConvolutionMap*)stage[2])->conn_table_[f_out][f_in * 2] = cur_f_in;
      cur_filt++;
    }
  }
  std::cout << "SpatialConvolutionMap connection table:" << std::endl;
  TorchStage::print3DTensorToStdCout<int16_t>(((SpatialConvolutionMap*)stage[2])->conn_table_, 
    num_feats_out, fan_in, 2);
  stage[2]->forwardProp(stage[1]->output, tp);
  std::cout << endl << endl << "SpatialConvolutionMap output:" << std::endl;
  TorchStage::print3DTensorToStdCout<float>(stage[2]->output, num_feats_out,
    ((SpatialConvolutionMap*)stage[2])->outHeight(), 
    ((SpatialConvolutionMap*)stage[2])->outWidth());

  // ***********************************************
  // Test SpatialLPPooling
  const float pnorm = 2;
  const int32_t pool_u = 2;
  const int32_t pool_v = 2;
  stage[3] = new SpatialLPPooling(num_feats_out, pnorm, pool_v, pool_u, 
    stage[2]->outHeight(), stage[2]->outWidth());
  stage[3]->forwardProp(stage[2]->output, tp);
  std::cout << endl << endl << "SpatialLPPooling output:" << std::endl;
  TorchStage::print3DTensorToStdCout<float>(stage[3]->output, num_feats_out,
    ((SpatialConvolutionMap*)stage[3])->outHeight(), 
    ((SpatialConvolutionMap*)stage[3])->outWidth());

  // ***********************************************
  // Test SpatialMaxPooling
  TorchStage* max_pool_stage = new SpatialMaxPooling(num_feats_in, pool_v,
    pool_u, height, width);
  max_pool_stage->forwardProp(data_in, tp);
  std::cout << endl << endl << "SpatialLPPooling output:" << std::endl;
  TorchStage::print3DTensorToStdCout<float>(max_pool_stage->output, 
    num_feats_in, ((SpatialMaxPooling*)max_pool_stage)->outHeight(), 
    ((SpatialMaxPooling*)max_pool_stage)->outWidth());

  // ***********************************************
  // Test SpatialSubtractiveNormalization
  uint32_t gauss_size = 7;
  float* kernel1D = TorchStage::gaussian1D<float>(gauss_size);
  std::cout << "kernel1D:" << std::endl;
  TorchStage::print3DTensorToStdCout<float>(kernel1D, 1, 1, gauss_size);

  TorchStage* sub_norm_stage = new SpatialSubtractiveNormalization(
    num_feats_in, gauss_size, kernel1D, height, width);
  sub_norm_stage->forwardProp(data_in, tp);
  std::cout << endl << endl << "SpatialSubtractiveNormalization output:" << endl;
  TorchStage::print3DTensorToStdCout<float>(sub_norm_stage->output, 
    num_feats_in, height, width);

  // ***********************************************
  // Test SpatialDivisiveNormalization
  TorchStage* div_norm_stage = new SpatialDivisiveNormalization(
    num_feats_in, gauss_size, kernel1D, height, width);
  div_norm_stage->forwardProp(data_in, tp);
  std::cout << endl << endl << "SpatialDivisiveNormalization output:" << endl;
  TorchStage::print3DTensorToStdCout<float>(div_norm_stage->output, 
    num_feats_in, height, width);

  // ***********************************************
  // Test SpatialContrastiveNormalization
  const int32_t lena_width = 512;
  const int32_t lena_height = 512;
  float* lena = new float[512*512];
  jtil::file_io::LoadArrayFromFile<float>(lena, lena_width * lena_height, 
    "lena_image.bin");
  uint32_t kernel_size = 7;
  TorchStage* cont_norm_stage = new SpatialContrastiveNormalization(
    1, lena_height, lena_width, kernel_size, NULL);
  cont_norm_stage->forwardProp(lena, tp);
  std::cout << endl << endl << "SpatialContrastiveNormalization output saved ";
  std::cout << "to lena_image_processed.bin" << endl;
  jtil::file_io::SaveArrayToFile<float>(
    ((SpatialContrastiveNormalization*)cont_norm_stage)->output(), 
    lena_width * lena_height, "lena_image_processed.bin");

  // ***********************************************
  // Test Linear
  TorchStage* lin_stage[2];
  lin_stage[0] = new Reshape(num_feats_in, height, width);
  int32_t lin_size = num_feats_in * width * height;
  int32_t lin_size_out = 20;
  lin_stage[1] = new Linear(num_feats_in * width * height, lin_size_out,
    NUM_WORKER_THREADS);
  for (int32_t i = 0; i < lin_size * lin_size_out; i++) {
    ((Linear*)lin_stage[1])->weights[i] = (float)(i+1) / 
      (float)(lin_size * lin_size_out);
  }
  for (int32_t i = 0; i < lin_size_out; i++) {
    ((Linear*)lin_stage[1])->bias[i] = (float)(i+1) / (float)(lin_size_out);
  }
  lin_stage[0]->forwardProp(data_in, tp);
  lin_stage[1]->forwardProp(lin_stage[0]->output, tp);
  std::cout << endl << endl << "Linear output:" << std::endl;
  TorchStage::print3DTensorToStdCout<float>(lin_stage[1]->output, 1, 1,
    lin_size_out);

  // ***********************************************
  // Test Loading a model
  TorchStage* m = TorchStage::loadFromFile("../data/handmodel.net.convnet");
  */

#if defined(WIN32) || defined(_WIN32)
  cout << endl;
  system("PAUSE");
#endif

  tp.stop();
}
