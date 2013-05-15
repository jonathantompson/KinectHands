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
#include "jtorch/table.h"
#include "jtil/threading/thread_pool.h"
#include "jtil/data_str/vector_managed.h"
#include "jtil/debug_util/debug_util.h"
#include "jtil/file_io/file_io.h"
#include "jtil/string_util/string_util.h"

#if defined(WIN32) || defined(_WIN32)
  #define snprintf _snprintf_s
#endif

#define NUM_WORKER_THREADS 1
#define TEST_MODULES
// #define TEST_MODEL

using namespace std;
using namespace jtorch;
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
float din[width * height * num_feats_in];
float dout[width * height * num_feats_out];

// CPU weights and biases for SpatialConvolution stage
float weights[num_feats_out * num_feats_in * filt_height * filt_width];
float biases[num_feats_out];

int main(int argc, char *argv[]) {  
#if defined(_DEBUG) || defined(DEBUG)
  jtil::debug::EnableMemoryLeakChecks();
  // jtil::debug::EnableAggressiveMemoryLeakChecks();
  // jtil::debug::SetBreakPointOnAlocation(3940);
#endif

  try {
    jtorch::InitJTorch("../jtorch");

#ifdef TEST_MODULES
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
    std::cout << "Data In:" << std::endl;
    data_in.print();

    Sequential stages;

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
      biases[i] = (float)(i+1) / (float)num_feats_out - 0.5f;
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
            weights[fout * filt_dim * num_feats_in + fin * filt_dim + v * filt_width + u] =
              scale * expf(-((x*x)/(2.0f*sigma_x_sq) + (y*y)/(2.0f*sigma_y_sq)));
          }
        }
      }
    }
    conv->setWeights(weights);
    conv->setBiases(biases);
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

    /*
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

    delete kernel;
    delete kernel2;
    */
#endif

#ifdef TEST_MODEL
    // ***********************************************
    // Test Loading a model
    TorchStage* m = TorchStage::loadFromFile("../data/handmodel.net.convnet");



    // Lets create a fake input image (of zeros) and make sure it passes through
    uint32_t w = HN_IM_SIZE;
    uint32_t h = HN_IM_SIZE;
    uint32_t data_file_size = 0;
    Table* hand_image = new Table();
    for (uint32_t i = 0; i < HN_DEFAULT_NUM_CONV_BANKS; i++) {
      data_file_size += w * h;
      FloatTensor* cur_image = new FloatTensor(Int2(h, w));
      hand_image->add(cur_image);
      w = w / 2;
      h = h / 2;
    }

    float* im = new float[data_file_size];
    jtil::file_io::LoadArrayFromFile<float>(im, data_file_size, 
      "kinect_hpf_depth_image_uncompressed.bin");
    
    w = HN_IM_SIZE;
    h = HN_IM_SIZE;
    float* ptr = im;
    for (uint32_t i = 0; i < HN_DEFAULT_NUM_CONV_BANKS; i++) {
      float* internal_data = ((FloatTensor*)(*hand_image)(i))->data();
      memcpy(internal_data, ptr, sizeof(internal_data[0]) * w * h);
      ptr = &ptr[w*h];
      w = w / 2;
      h = h / 2;
    }

    m->forwardProp(*hand_image, tp);
    std::cout << "Model Output = " << std::endl;
    m->output->print();

    // Some debugging if things go wrong:
    if (m->type() != SEQUENTIAL_STAGE) {
      throw std::wruntime_error("main() - ERROR: Expecting sequential!");
    }
    TorchStage* join_table = ((Sequential*)m)->get(1);
    if (join_table->type() != JOIN_TABLE_STAGE) {
      throw std::wruntime_error("main() - ERROR: Expecting JoinTable!");
    }
    FloatTensor* join_table_out = (FloatTensor*)join_table->output;

    delete m;
    delete im;
    delete hand_image;
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
