#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include "hand_net/hand_net.h"
#include "hand_net/conv_stage.h"
#include "hand_net/nn_stage.h"
#include "exceptions/wruntime_error.h"
#include "depth_images_io.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }

using math::Float4x4;
using math::FloatQuat;
using math::Float3;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using math::Float3;
using depth_images_io::DepthImagesIO;

namespace hand_net {

  int16_t HandNet::depth[src_dim];
  float HandNet::xyz[src_dim * 3];
  float HandNet::cropped_depth[HAND_NET_PIX * HAND_NET_PIX];
  float HandNet::cropped_xyz[3 * HAND_NET_PIX * HAND_NET_PIX];
 
  HandNet::HandNet(const std::string& convnet_filename) {
    n_conv_stages_ = 0;
    conv_stages_ = NULL;
    n_nn_stages_ = 0;
    nn_stages_ = NULL;
    datcur_ = NULL;
    datnext_ = NULL;

    loadFromFile(convnet_filename);
  }

  HandNet::~HandNet() {
    for (int32_t i = 0; i < n_conv_stages_; i++) {
      SAFE_DELETE(conv_stages_[i]);
    }
    SAFE_DELETE(conv_stages_);
    for (int32_t i = 0; i < n_nn_stages_; i++) {
      SAFE_DELETE(nn_stages_[i]);
    }
    SAFE_DELETE(nn_stages_);
    SAFE_DELETE(datcur_);
    SAFE_DELETE(datnext_);
  }

  void HandNet::loadFromFile(const std::string& convnet_filename) {
    if (conv_stages_ != NULL) {
      throw std::wruntime_error("ConvStage::loadFromFile() - ERROR: "
        "Convnet data already exists. If you want to reload the convnet, then"
        " call the destructor and reload.");
    }

    std::cout << "loading HandNet from " << convnet_filename << std::endl;

    std::ifstream file(convnet_filename.c_str(), std::ios::in | std::ios::binary);
    if (!file.is_open()) {
      throw std::wruntime_error("HandNet::loadFromFile() - ERROR: Could not "
        "open convnet file!");
    }
    file.seekg(0, std::ios::beg);

    // Get the meta data
    file.read(reinterpret_cast<char*>(&n_conv_stages_), sizeof(n_conv_stages_));
    file.read(reinterpret_cast<char*>(&n_nn_stages_), sizeof(n_nn_stages_));

    // Load in the convolution stages
    conv_stages_ = new ConvStage*[n_conv_stages_];
    for (int32_t i = 0; i < n_conv_stages_; i++) {
      conv_stages_[i] = new ConvStage();
      conv_stages_[i]->loadFromFile(file);
#if defined(DEBUG) || defined(_DEBUG)
      std::cout << "ConvStage[" << i << "]" << std::endl;
      conv_stages_[i]->printToStdOut();
#endif
    }

    // Load in the neural network stages
    nn_stages_ = new NNStage*[n_nn_stages_];
    for (int32_t i = 0; i < n_nn_stages_; i++) {
      nn_stages_[i] = new NNStage();
      nn_stages_[i]->loadFromFile(file);
#if defined(DEBUG) || defined(_DEBUG)
      std::cout << "NNStage[" << i << "]" << std::endl;
      nn_stages_[i]->printToStdOut();
#endif
    }

    // clean up file io
    file.close();

    // Now create sufficient temporary data so that we can propogate the
    // forward model
    int32_t im_sizeu = HAND_NET_IM_SIZE;
    int32_t im_sizev = HAND_NET_IM_SIZE;
    int32_t max_size = im_sizeu * im_sizev;
    for (int32_t i = 0; i < n_conv_stages_; i++) {
      max_size = std::max<int32_t>(max_size, 
        conv_stages_[i]->dataSizeReq(im_sizeu, im_sizev));
      im_sizeu = conv_stages_[i]->calc_out_im_width(im_sizeu);
      im_sizev = conv_stages_[i]->calc_out_im_height(im_sizev);
    }

    // Quick check to make sure the sizes match up!
    if ((im_sizeu * im_sizev * 
      conv_stages_[n_conv_stages_-1]->n_output_features()) != 
      nn_stages_[0]->n_inputs()) {
      throw std::wruntime_error("HandNet::loadFromFile() - INTERNAL ERROR: "
        "convolution output size doesn't match neural net intput size");
    }

    for (int32_t i = 0; i < n_nn_stages_; i++) {
      max_size = std::max<uint32_t>(max_size, nn_stages_[i]->dataSizeReq());
      if (i < n_nn_stages_ - 1) {
        if (nn_stages_[i]->n_outputs() != nn_stages_[i+1]->n_inputs()) {
          throw std::wruntime_error("HandNet::loadFromFile() - INTERNAL ERROR:"
            " neural net output size doesn't match neural net intput size");
        }
      }
    }
    
    datcur_ = new float[max_size];
    datnext_ = new float[max_size];
  }

  void HandNet::calcHandImageFromSytheticDepth(const float* depth_in, 
    float* hand_im_depth, float* hand_im_xyx, float* x_com,
    float* y_com, float* z_com, float* std) {
    // Convert float to int16
    for (uint32_t i = 0; i < src_dim; i++) {
      depth[i] = (int16_t)depth_in[i];
    }

    if (hand_im_xyx != NULL) {
      DepthImagesIO::convertSingleImageToXYZ(xyz, depth);
    }

    // Find the COM in pixel space so we can crop the image.
    uint32_t cnt = 0;
    uint32_t u_com = 0;
    uint32_t v_com = 0;
    for (uint32_t v = 0; v < src_height; v++) {
      for (uint32_t u = 0; u < src_width; u++) {
        uint32_t src_index = v * src_width + u;
        if (depth_in[src_index] > EPSILON) {
          u_com += u;
          v_com += v;
          cnt++;
        }
      }
    }
    u_com /= cnt;
    v_com /= cnt;
    uint32_t u_start = u_com - (HAND_NET_PIX / 2);
    uint32_t v_start = v_com - (HAND_NET_PIX / 2);

    // Crop the image
    for (uint32_t v = v_start; v < v_start + HAND_NET_PIX; v++) {
      for (uint32_t u = u_start; u < u_start + HAND_NET_PIX; u++) {
        uint32_t dst_index = (v-v_start)* HAND_NET_PIX + (u-u_start);
        uint32_t src_index = v * src_width + u;
        if (depth_in[src_index] > EPSILON && depth_in[src_index] < GDT_MAX_DIST) {
            cropped_depth[dst_index] = depth_in[src_index];
            if (hand_im_xyx != NULL) {
              cropped_xyz[3*dst_index] = xyz[src_index*3];
              cropped_xyz[3*dst_index + 1] = xyz[src_index*3 + 1];
              cropped_xyz[3*dst_index + 2] = xyz[src_index*3 + 2];
            }
        } else {
          cropped_depth[dst_index] = 0;
          if (hand_im_xyx != NULL) {
            cropped_xyz[3*dst_index] = 0;
            cropped_xyz[3*dst_index + 1] = 0;
            cropped_xyz[3*dst_index + 2] = 0;
          }
        }
      }
    }

    // Now downsample if needed:
    if (HAND_NET_DOWN_FACT > 1) {
      for (uint32_t v = 0; v < HAND_NET_IM_SIZE; v++) {
        for (uint32_t u = 0; u < HAND_NET_IM_SIZE; u++) {
          uint32_t dst = v*HAND_NET_IM_SIZE + u;
          hand_im_depth[dst] = 0;
          if (hand_im_xyx != NULL) {
            hand_im_xyx[dst*3] = 0;
            hand_im_xyx[dst*3+1] = 0;
            hand_im_xyx[dst*3+2] = 0;
          }
          float n_pts = 0;
          for (uint32_t v_off = v; v_off < v + HAND_NET_DOWN_FACT; v_off++) {
            for (uint32_t u_off = u; u_off < u + HAND_NET_DOWN_FACT; u_off++) {
              uint32_t src = HAND_NET_DOWN_FACT * v * HAND_NET_PIX + HAND_NET_DOWN_FACT * u;
              if (cropped_depth[src] > 0) {
                hand_im_depth[dst] += cropped_depth[src];
                if (hand_im_xyx != NULL) {
                  hand_im_xyx[dst*3] += cropped_xyz[src*3];
                  hand_im_xyx[dst*3+1] += cropped_xyz[src*3+1];
                  hand_im_xyx[dst*3+2] += cropped_xyz[src*3+2];
                }
                n_pts += 1;
              }
            }
          }

          if (n_pts > 0) {
            hand_im_depth[dst] /= n_pts;
            if (hand_im_xyx != NULL) {
              hand_im_xyx[dst*3] /= n_pts;
              hand_im_xyx[dst*3+1] /= n_pts;
              hand_im_xyx[dst*3+2] /= n_pts;
            }

          } else {
            hand_im_depth[dst] = 0;
            if (hand_im_xyx != NULL) {
              hand_im_xyx[dst*3] /= n_pts;
              hand_im_xyx[dst*3+1] /= n_pts;
              hand_im_xyx[dst*3+2] /= n_pts;
            }
          }
        }
      }
    } else {
      memcpy(hand_im_depth, cropped_depth, 
        sizeof(hand_im_depth[0]) * HAND_NET_IM_SIZE * HAND_NET_IM_SIZE);
      if (hand_im_xyx != NULL) {
        memcpy(hand_im_xyx, cropped_xyz, 
          sizeof(hand_im_xyx[0]) * HAND_NET_IM_SIZE*HAND_NET_IM_SIZE*3);
      }
    }

    // Now calculate the mean and STD of the cropped downsampled image
    cnt = 0;
    if (x_com) { x_com = 0; }
    if (y_com) { y_com = 0; }
    if (z_com) { z_com = 0; }
    float running_sum = 0;
    float running_sum_sq = 0;
    for (uint32_t v = 0; v < HAND_NET_IM_SIZE; v++) {
      for (uint32_t u = 0; u < HAND_NET_IM_SIZE; u++) {
        uint32_t src_index = v * HAND_NET_IM_SIZE + u;
        if (hand_im_depth[src_index] > EPSILON) {
          cnt++;
          running_sum += hand_im_depth[src_index];
          running_sum_sq += hand_im_depth[src_index] * hand_im_depth[src_index];
          if (x_com && hand_im_xyx) { *x_com += hand_im_xyx[src_index* 3]; }
          if (y_com && hand_im_xyx) { *y_com += hand_im_xyx[src_index*3 + 1]; }
          if (z_com && hand_im_xyx) { *z_com += hand_im_xyx[src_index*3 + 2]; }
        }
      }
    }
    if (x_com) { *x_com /= (float)cnt; }
    if (y_com) { *y_com /= (float)cnt; }
    if (z_com) { *z_com /= (float)cnt; }
    
    float mean = running_sum / (float)cnt;
    float var = (running_sum_sq / (float)cnt) - (mean * mean);
    float std_ = sqrtf(var);
    if (std) { *std = std_; }

    // Now subtract away the mean and scale depth by the std
    for (uint32_t v = 0; v < HAND_NET_IM_SIZE; v++) {
      for (uint32_t u = 0; u < HAND_NET_IM_SIZE; u++) {
        uint32_t src_index = v * HAND_NET_IM_SIZE + u;
        if (hand_im_depth[src_index] > EPSILON) {
          hand_im_depth[src_index] = (hand_im_depth[src_index] - mean) / std_;
        } else {
          hand_im_depth[src_index] = 0;
        }
      }
    }
  }

}  // namespace hand_model
