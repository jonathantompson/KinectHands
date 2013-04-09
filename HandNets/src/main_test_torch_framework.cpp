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
#include "kinect_interface/hand_net/sequential.h"
#include "jtil/threading/thread_pool.h"
#include "jtil/data_str/vector_managed.h"
#include "jtil/debug_util/debug_util.h"
#include "jtil/file_io/file_io.h"
#include "jtil/string_util/string_util.h"

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif

#define NUM_WORKER_THREADS 1

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
  // jtil::debug::SetBreakPointOnAlocation(6965);
#endif

  try {

    FloatTensor data_in(Int3(width, height, num_feats_in));
    FloatTensor data_out(Int3(width, height, num_feats_out));
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

    Sequential stages;

    // ***********************************************
    // Test Tanh
    stages.add(new Tanh());
    stages.forwardProp(data_in, tp);
    std::cout << endl << endl << "Tanh output:" << std::endl;
    stages.output->print();

    // ***********************************************
    // Test Threshold
    const float threshold = 0.5f;
    const float val = 0.1f;
    stages.add(new Threshold());
    ((Threshold*)stages.get(1))->threshold = threshold;
    ((Threshold*)stages.get(1))->val = val;
    stages.forwardProp(data_in, tp);
    std::cout << endl << endl << "Threshold output:" << std::endl;
    stages.output->print();

  
    // ***********************************************
    // Test SpatialConvolutionMap
    stages.add(new SpatialConvolutionMap(num_feats_in, num_feats_out, fan_in,
      filt_height, filt_width));
    for (int32_t i = 0; i < num_feats_out; i++) {
      ((SpatialConvolutionMap*)stages.get(2))->biases[i] = (float)(i+1) / 
        (float)num_feats_out - 0.5f;
    }
    const float sigma_x_sq = 1.0f;
    const float sigma_y_sq = 1.0f;
    for (int32_t i = 0; i < num_feats_out * fan_in; i++) {
      float scale = ((float)(i + 1) / (float)(num_feats_out * fan_in));
      for (int32_t v = 0; v < filt_height; v++) {
        for (int32_t u = 0; u < filt_width; u++) {
          float x = (float)u - (float)(filt_width-1) / 2.0f;
          float y = (float)v - (float)(filt_height-1) / 2.0f;
          ((SpatialConvolutionMap*)stages.get(2))->weights[i][v * filt_width + u] = 
            scale * expf(-((x*x)/(2.0f*sigma_x_sq) + (y*y)/(2.0f*sigma_y_sq)));
        }
      }
    }
    int32_t cur_filt = 0;
    for (int32_t f_out = 0; f_out < num_feats_out; f_out++) {
      for (int32_t f_in = 0; f_in < fan_in; f_in++) {
        ((SpatialConvolutionMap*)stages.get(2))->conn_table[f_out][f_in * 2 + 1] = cur_filt;
        int32_t cur_f_in = (f_out + f_in) % num_feats_in;
        ((SpatialConvolutionMap*)stages.get(2))->conn_table[f_out][f_in * 2] = cur_f_in;
        cur_filt++;
      }
    }
    stages.forwardProp(data_in, tp);
    std::cout << endl << endl << "SpatialConvolutionMap output:" << std::endl;
    stages.output->print();

  
    // ***********************************************
    // Test SpatialLPPooling
    const float pnorm = 2;
    const int32_t pool_u = 2;
    const int32_t pool_v = 2;
    stages.add(new SpatialLPPooling(pnorm, pool_v, pool_u));
    stages.forwardProp(data_in, tp);
    std::cout << endl << endl << "SpatialLPPooling output:" << std::endl;
    stages.output->print();

    // ***********************************************
    // Test SpatialMaxPooling
    SpatialMaxPooling max_pool_stage(pool_v, pool_u);
    max_pool_stage.forwardProp(data_in, tp);
    std::cout << endl << endl << "SpatialLPPooling output:" << std::endl;
    max_pool_stage.output->print();
  
    // ***********************************************
    // Test SpatialSubtractiveNormalization
    uint32_t gauss_size = 7;
    FloatTensor* kernel = FloatTensor::gaussian1D(gauss_size);
    std::cout << "kernel1D:" << std::endl;
    kernel->print();

    SpatialSubtractiveNormalization sub_norm_stage(*kernel);
    sub_norm_stage.forwardProp(data_in, tp);
    std::cout << endl << endl << "SpatialSubtractiveNormalization output:" << endl;
    sub_norm_stage.output->print();

    // ***********************************************
    // Test SpatialDivisiveNormalization
    SpatialDivisiveNormalization div_norm_stage(*kernel);
    div_norm_stage.forwardProp(data_in, tp);
    std::cout << endl << endl << "SpatialDivisiveNormalization output:" << endl;
    div_norm_stage.output->print();

    // ***********************************************
    // Test SpatialContrastiveNormalization
    const int32_t lena_width = 512;
    const int32_t lena_height = 512;
    FloatTensor lena(Int2(lena_width, lena_height));
    jtil::file_io::LoadArrayFromFile<float>(lena.data(), 
      lena_width * lena_height, "lena_image.bin");
    uint32_t kernel_size = 7;
    FloatTensor* kernel2 = FloatTensor::ones1D(kernel_size);
    SpatialContrastiveNormalization cont_norm_stage(kernel2);
    cont_norm_stage.forwardProp(lena, tp);
    std::cout << endl << endl << "SpatialContrastiveNormalization output saved ";
    std::cout << "to lena_image_processed.bin" << endl;
    jtil::file_io::SaveArrayToFile<float>(((FloatTensor*)cont_norm_stage.output)->data(), 
      lena_width * lena_height, "lena_image_processed.bin");

    // ***********************************************
    // Test Linear
    Sequential lin_stage;
    lin_stage.add(new Reshape());
    int32_t lin_size = num_feats_in * width * height;
    int32_t lin_size_out = 20;
    lin_stage.add(new Linear(lin_size, lin_size_out));
    for (int32_t i = 0; i < lin_size * lin_size_out; i++) {
      ((Linear*)lin_stage.get(1))->weights[i] = (float)(i+1) / 
        (float)(lin_size * lin_size_out);
    }
    for (int32_t i = 0; i < lin_size_out; i++) {
      ((Linear*)lin_stage.get(1))->bias[i] = (float)(i+1) / (float)(lin_size_out);
    }
    lin_stage.forwardProp(data_in, tp);
    std::cout << endl << endl << "Linear output:" << std::endl;
    lin_stage.output->print();

    // ***********************************************
    // Test Loading a model
    TorchStage* m = TorchStage::loadFromFile("../data/handmodel.net.convnet");

    delete kernel;
    delete kernel2;

    tp.stop();
  } catch (std::wruntime_error e) {
    std::cout << "Exception caught!" << std::endl;
    std::cout << jtil::string_util::ToNarrowString(e.errorMsg()) << std::endl;
  };


#if defined(WIN32) || defined(_WIN32)
  cout << endl;
  system("PAUSE");
#endif


}
