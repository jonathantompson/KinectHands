// THE CPP FUNCTIONALITY HERE IS TO BE TESTED AGAINST "jtorch_test.lua" SCRIPT

#include <stdlib.h>
#include <cmath>
#include <thread>
#include <iostream>
#include <limits>
#include "jtorch/torch_stage.h"
#include "jtorch/jtorch.h"
#include "jtorch/tensor.h"
#include "jtorch/spatial_convolution.h"
#include "jtorch/spatial_convolution_map.h"
#include "jtorch/spatial_lp_pooling.h"
#include "jtorch/spatial_max_pooling.h"
#include "jtorch/spatial_subtractive_normalization.h"
#include "jtorch/spatial_divisive_normalization.h"
#include "jtorch/spatial_contrastive_normalization.h"
#include "jtorch/linear.h"
#include "jtorch/reshape.h"
#include "jtorch/tanh.h"
#include "jtorch/threshold.h"
#include "jtorch/sequential.h"
#include "jtorch/parallel.h"
#include "jtorch/table.h"
#include "jtil/threading/thread_pool.h"
#include "jtil/data_str/vector_managed.h"
#include "jtil/debug_util/debug_util.h"
#include "jtil/file_io/file_io.h"
#include "jtil/string_util/string_util.h"
#include "jtil/clk/clk.h"

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif

#define NUM_WORKER_THREADS 1
// #define TEST_MODULES
// #define TEST_MODEL
#define TEST_BIG_MODEL  // The actual convnet

using namespace std;
using namespace jtorch;
using namespace jtil::threading;
using namespace jtil::math;
using namespace jtil::data_str;
using namespace jtil::file_io;
using namespace jtil::clk;

const uint32_t num_feats_in = 5;
const uint32_t num_feats_out = 10;
const uint32_t fan_in = 3;  // For SpatialConvolutionMap
const uint32_t width = 10;
const uint32_t height = 10;
const uint32_t filt_height = 5;
const uint32_t filt_width = 5;
float din[width * height * num_feats_in];
float dout[width * height * num_feats_out];

// CPU weights and biases for SpatialConvolution stage
float cweights[num_feats_out * num_feats_in * filt_height * filt_width];
float cbiases[num_feats_out];

// CPU weights and biases for Linear stage
const int32_t lin_size_in = num_feats_in * width * height;
const int32_t lin_size_out = 20;
float lweights[lin_size_in * lin_size_out];
float lbiases[lin_size_out];

int main(int argc, char *argv[]) {  
#if defined(_DEBUG) || defined(DEBUG)
  jtil::debug::EnableMemoryLeakChecks();
  // jtil::debug::EnableAggressiveMemoryLeakChecks();
  // jtil::debug::SetBreakPointOnAlocation(3940);
#endif

  try {
    jtorch::InitJTorch("../jtorch");

    Tensor<float> data_in(Int3(width, height, num_feats_in));
    Tensor<float> data_out(Int3(width, height, num_feats_out));

    for (uint32_t f = 0; f < num_feats_in; f++) {
      float val = (f+1) - (float)(width * height) / 16.0f;
      for (uint32_t v = 0; v < height; v++) {
        for (uint32_t u = 0; u < width; u++) {
          din[f * width * height + v * width + u] = val;
          val += 1.0f / 8.0f;
        }
      }
    }
    data_in.setData(din);

#ifdef TEST_MODULES
    Sequential stages;

    std::cout << "Data In:" << std::endl;
    data_in.print();

    // ***********************************************
    // Test Tanh
    stages.add(new Tanh());
    stages.forwardProp(data_in);
    std::cout << endl << endl << "Tanh output:" << std::endl;
    stages.output->print();

    
    // ***********************************************
    // Test Threshold
    const float threshold = 0.5f;
    const float val = 0.1f;
    stages.add(new jtorch::Threshold());
    ((jtorch::Threshold*)stages.get(1))->threshold = threshold;
    ((jtorch::Threshold*)stages.get(1))->val = val;
    stages.forwardProp(data_in);
    std::cout << endl << endl << "Threshold output:" << std::endl;
    stages.output->print();
    
    // ***********************************************
    // Test SpatialConvolutionMap --> THIS STAGE IS STILL ON THE CPU!!
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
    stages.forwardProp(data_in);
    std::cout << endl << endl << "SpatialConvolutionMap output:" << std::endl;
    stages.output->print();

    // ***********************************************
    // Test SpatialConvolution
    SpatialConvolution* conv = new SpatialConvolution(num_feats_in, 
      num_feats_out, filt_height, filt_width);
    for (int32_t i = 0; i < num_feats_out; i++) {
      cbiases[i] = (float)(i+1) / (float)num_feats_out - 0.5f;
    }
    const uint32_t filt_dim = filt_width * filt_height;
    for (int32_t fout = 0; fout < num_feats_out; fout++) {
      for (int32_t fin = 0; fin < num_feats_in; fin++) {
        int32_t i = fout * num_feats_out + fin;
        float scale = ((float)(i + 1) / (float)(num_feats_out * num_feats_in));
        for (int32_t v = 0; v < filt_height; v++) {
          for (int32_t u = 0; u < filt_width; u++) {
            float x = (float)u - (float)(filt_width-1) / 2.0f;
            float y = (float)v - (float)(filt_height-1) / 2.0f;
            cweights[fout * filt_dim * num_feats_in + fin * filt_dim + v * filt_width + u] =
              scale * expf(-((x*x)/(2.0f*sigma_x_sq) + (y*y)/(2.0f*sigma_y_sq)));
          }
        }
      }
    }
    conv->setWeights(cweights);
    conv->setBiases(cbiases);
    conv->forwardProp(*stages.get(1)->output);
    std::cout << endl << endl << "SpatialConvolution output:" << std::endl;
    conv->output->print();
  
    
    // ***********************************************
    // Test SpatialLPPooling
    const float pnorm = 2;
    const int32_t pool_u = 2;
    const int32_t pool_v = 2;
    stages.add(new SpatialLPPooling(pnorm, pool_v, pool_u));
    stages.forwardProp(data_in);
    std::cout << endl << endl << "SpatialLPPooling output:" << std::endl;
    stages.output->print();

    // ***********************************************
    // Test SpatialMaxPooling
    SpatialMaxPooling max_pool_stage(pool_v, pool_u);
    max_pool_stage.forwardProp(data_in);
    std::cout << endl << endl << "SpatialMaxPooling output:" << std::endl;
    max_pool_stage.output->print();
  
    // ***********************************************
    // Test SpatialSubtractiveNormalization
    uint32_t gauss_size = 7;
    Tensor<float>* kernel = Tensor<float>::gaussian1D(gauss_size);
    std::cout << "kernel1D:" << std::endl;
    kernel->print();

    SpatialSubtractiveNormalization sub_norm_stage(*kernel);
    sub_norm_stage.forwardProp(data_in);
    std::cout << endl << endl << "SpatialSubtractiveNormalization output:" << endl;
    sub_norm_stage.output->print();

    // ***********************************************
    // Test SpatialDivisiveNormalization
    SpatialDivisiveNormalization div_norm_stage(*kernel);
    div_norm_stage.forwardProp(data_in);
    std::cout << endl << endl << "SpatialDivisiveNormalization output:" << endl;
    div_norm_stage.output->print();

    // ***********************************************
    // Test SpatialContrastiveNormalization
    const int32_t lena_width = 512;
    const int32_t lena_height = 512;
    Tensor<float> lena(Int2(lena_width, lena_height));
    float* lena_cpu = new float[lena.dataSize()];
    jtil::file_io::LoadArrayFromFile<float>(lena_cpu, 
      lena_width * lena_height, "lena_image.bin");
    lena.setData(lena_cpu);
    delete[] lena_cpu;
    uint32_t kernel_size = 7;
    Tensor<float>* kernel2 = Tensor<float>::ones1D(kernel_size);
    SpatialContrastiveNormalization cont_norm_stage(kernel2);
    cont_norm_stage.forwardProp(lena);
    std::cout << endl << endl << "SpatialContrastiveNormalization output saved ";
    std::cout << "to lena_image_processed.bin" << endl;
    float* cont_norm_output_cpu = new float[cont_norm_stage.output->dataSize()];
    ((Tensor<float>*)cont_norm_stage.output)->getData(cont_norm_output_cpu);
    jtil::file_io::SaveArrayToFile<float>(cont_norm_output_cpu, 
      lena_width * lena_height, "lena_image_processed.bin");
    delete[] cont_norm_output_cpu;

    // ***********************************************
    // Test Linear
    Sequential lin_stage;
    lin_stage.add(new Reshape());

    Linear* lin = new Linear(lin_size_in, lin_size_out);
    lin_stage.add(lin);
    for (int32_t i = 0; i < lin_size_in * lin_size_out; i++) {
      lweights[i] = (float)(i+1) / (float)(lin_size_in * lin_size_out);
    }
    for (int32_t i = 0; i < lin_size_out; i++) {
      lbiases[i] = (float)(i+1) / (float)(lin_size_out);
    }
    lin->setBiases(lbiases);
    lin->setWeights(lweights);
    lin_stage.forwardProp(data_in);
    std::cout << endl << endl << "Linear output:" << std::endl;
    lin_stage.output->print();

    delete kernel;
    delete kernel2;
#endif

#ifdef TEST_MODEL
    // ***********************************************
    // Test Loading a model
    TorchStage* model = TorchStage::loadFromFile("./testmodel.bin");

    model->forwardProp(data_in);
    std::cout << "Model Output = " << std::endl;
    model->output->print();

    // Some debugging if things go wrong:
    if (model->type() != SEQUENTIAL_STAGE) {
      throw std::wruntime_error("main() - ERROR: Expecting sequential!");
    }

    delete model;
#endif

#ifdef TEST_BIG_MODEL
    // ***********************************************
    // Test Loading the big convnet model
    TorchStage* convnet_model = TorchStage::loadFromFile("../data/handmodel.net.convnet");

    if (convnet_model->type() != SEQUENTIAL_STAGE) {
      throw std::wruntime_error("main() - ERROR: Expecting Sequential!");
    }

    uint32_t w = 96;
    uint32_t h = 96;
    const uint32_t num_banks = 3;
    uint32_t data_size = 0;
    for (uint32_t i = 0; i < num_banks; i++) {
      data_size += w * h; 
      w = w / 2;
      h = h / 2;
    }
    
    float* convnet_input_cpu = new float[data_size];
    LoadArrayFromFile<float>(convnet_input_cpu, data_size, 
      "../data/hand_depth_data_processed_for_CN/"
      "hpf_processed_1294371228_hands0_263917398000.bin");

    // Create some dummy data (all zeros for now)
    Table* convnet_input = new Table();
    w = 96;
    h = 96;
    float* cur_hand_image = convnet_input_cpu;
    for (uint32_t i = 0; i < num_banks; i++) {
      Tensor<float>* im = new Tensor<float>(Int3(w, h, 1));
      convnet_input->add(im);
      im->setData(cur_hand_image);
      cur_hand_image = &cur_hand_image[w*h];
      w = w / 2;
      h = h / 2;
    }
    
    std::cout << "Performing forward prop...";
    convnet_model->forwardProp(*convnet_input);
    Tensor<float>* convnet_output = (Tensor<float>*)convnet_model->output;
    std::cout << "Model Output (just the first 30 numbers) = " << std::endl;
    convnet_output->print(Int2(0, 29), Int2(0, 0), Int2(0, 0));

    // Save the result to file
    float* convnet_output_cpu = new float[convnet_output->dataSize()];
    convnet_output->getData(convnet_output_cpu);
    jtil::file_io::SaveArrayToFile<float>(convnet_output_cpu, 
      convnet_output->dataSize(), "convnet_output.bin");
    delete[] convnet_output_cpu;

    Tensor<float>* accum_buffer = new Tensor<float>(convnet_output->dim());
    Int3 local_worgroup_size;
    for (uint32_t i = 0; i < 3; i++) {
      local_worgroup_size[i] = std::min<int>(jtorch::max_local_workgroup_size,
       convnet_output->dim()[i]);
      while (local_worgroup_size[i] > 1 &&
        convnet_output->dim()[i] % local_worgroup_size[i] != 0) {
          local_worgroup_size[i]--;
      }
    }

    // Now profile
    jtorch::cl_context->useKernel("./accum.cl", "Accum");  // To build the kernel before execution
    jtorch::cl_context->sync(jtorch::deviceid);
    std::cout << "Profiling..." << std::endl;
    Clk clk;
    double t0 = clk.getTime();
    for (uint32_t i = 0; i < 100; i++) {
      convnet_model->forwardProp(*convnet_input);

      // Accumulate the output (so the optimizer doesn't blow the loop away)
      jtorch::cl_context->useKernel("./accum.cl", "Accum");
      jtorch::cl_context->setArg(0, convnet_output->data());
      jtorch::cl_context->setArg(1, accum_buffer->data());
      jtorch::cl_context->runKernel3D(jtorch::deviceid, accum_buffer->dim(),
        local_worgroup_size, false);

    }
    jtorch::cl_context->sync(jtorch::deviceid);
    double t1 = clk.getTime();
    std::cout << "Time for 100 evaluations = " << (t1 - t0) << std::endl;
    std::cout << "Accum Output (just the first 30 numbers) = " << std::endl;
    accum_buffer->print(Int2(0, 29), Int2(0, 0), Int2(0, 0));

    delete[] convnet_input_cpu;
    delete convnet_input;
    delete convnet_model;
#endif

  } catch (std::wruntime_error e) {
    std::cout << "Exception caught!" << std::endl;
    std::cout << jtil::string_util::ToNarrowString(e.errorMsg()) << std::endl;
  };

  jtorch::ShutdownJTorch();

#if defined(WIN32) || defined(_WIN32)
  cout << endl;
  system("PAUSE");
#endif


}
