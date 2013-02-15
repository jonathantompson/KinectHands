#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <cmath>
#include "hand_net/hand_net.h"
#include "hand_net/conv_stage.h"
#include "hand_net/nn_stage.h"
#include "exceptions/wruntime_error.h"
#include "hand_model/hand_model.h"  // for HandCoeff
#include "open_ni_funcs.h"

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
using hand_model::HandCoeff;

namespace hand_net {

  float HandNet::float_depth_[src_dim];
  float HandNet::cropped_depth[HAND_NET_PIX * HAND_NET_PIX];
 
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
//#if defined(DEBUG) || defined(_DEBUG)
//      std::cout << "ConvStage[" << i << "]" << std::endl;
//      conv_stages_[i]->printToStdOut();
//#endif
    }

    // Load in the neural network stages
    nn_stages_ = new NNStage*[n_nn_stages_];
    for (int32_t i = 0; i < n_nn_stages_; i++) {
      nn_stages_[i] = new NNStage();
      nn_stages_[i]->loadFromFile(file);
//#if defined(DEBUG) || defined(_DEBUG)
//      std::cout << "NNStage[" << i << "]" << std::endl;
//      nn_stages_[i]->printToStdOut();
//#endif
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
      im_sizeu = conv_stages_[i]->calcOutWidth(im_sizeu);
      im_sizev = conv_stages_[i]->calcOutHeight(im_sizev);
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

    std::cout << "Finished Loading HandNet from " << convnet_filename << std::endl;
  }

  void HandNet::calcHandCoeff(const int16_t* depth, const uint8_t* label, 
    Eigen::MatrixXf& coeff) {
    for (uint32_t i = 0; i < src_dim; i++) {
      float_depth_[i] = (float)depth[i];
    }
    calcHandCoeff(float_depth_, label, coeff);
  }

  void HandNet::calcHandCoeff(const float* depth, const uint8_t* label, 
    Eigen::MatrixXf& coeff) {
    // First create a hand image:
    Float3 xyz_com;
    float std;
    HandNet::calcHandImage(depth, label, datcur_, xyz_com, std);

    int32_t width = HAND_NET_IM_SIZE;
    int32_t height = HAND_NET_IM_SIZE;

    for (int32_t i = 0; i < n_conv_stages_; i++) {
      conv_stages_[i]->forwardProp(datcur_, width, height, datnext_);
      // Ping-pong the buffers
      float* tmp = datnext_;
      datnext_ = datcur_;
      datcur_ = tmp;
      // Calculate the next stage size
      width = conv_stages_[i]->calcOutWidth(width);
      height = conv_stages_[i]->calcOutHeight(height);
    }

    // print3DTensorToStdCout<float>(datcur_, 0, width, height, 2, 2, 6, 6);

    for (int32_t i = 0; i < n_nn_stages_; i++) {
      nn_stages_[i]->forwardProp(datcur_, datnext_);
      // Ping-pong the buffers
      float* tmp = datnext_;
      datnext_ = datcur_;
      datcur_ = tmp;
    }

    // print3DTensorToStdCout<float>(datcur_, 1, 25, 1);

    // Now the coeffs are in a scaled format which also has euler angles
    // instead of quaternions, convert the format back
    for (uint32_t i = HandCoeff::HAND_POS_X; i <= HandCoeff::HAND_POS_Z; i++) {
      coeff(i) = datcur_[i] * std + xyz_com[i];
    }

    Float3 euler(datcur_[HandCoeff::HAND_ORIENT_X],
      datcur_[HandCoeff::HAND_ORIENT_Y], datcur_[HandCoeff::HAND_ORIENT_Z]);
    FloatQuat quat;
    math::FloatQuat::eulerAngles2Quat(&quat, euler[0], euler[1], euler[2]);
    for (uint32_t i = 0; i < 4; i++) {
      coeff(HandCoeff::HAND_ORIENT_X + i) = quat[i];
    }

    // The rest are just shifted back by one
    for (uint32_t i = HandCoeff::HAND_ORIENT_W + 1; i < HAND_NUM_COEFF; i++) {
      coeff(i) = datcur_[i-1];
    }

    // print3DTensorToStdCout<float>(&coeff(0), 1, 26, 1);
  }

  void HandNet::createLabelFromSyntheticDepth(const float* depth, 
    uint8_t* label) {
    for (uint32_t i = 0; i < src_dim; i++) {
      label[i] = (depth[i] > EPSILON && depth[i] < GDT_MAX_DIST) ? 1 : 0;
    }
  }

  // Create the downsampled hand image, background is at 1 and hand is
  // in front of it.
  void HandNet::calcHandImage(const float* depth_in, const uint8_t* label_in,
    float* hand_image, math::Float2& uv_com, math::Float3& xyz_com, 
    float& std) {
    // Find the COM in pixel space so we can crop the image.
    uint32_t cnt = 0;
    uv_com[0] = 0;
    uv_com[1] = 0;
    for (uint32_t v = 0; v < src_height; v++) {
      for (uint32_t u = 0; u < src_width; u++) {
        uint32_t src_index = v * src_width + u;
        if (label_in[src_index] == 1) {
          uv_com[0] += u;
          uv_com[1] += v;
          cnt++;
        }
      }
    }
    uv_com[0] /= cnt;
    uv_com[1] /= cnt;
    uint32_t u_start = uv_com[0] - (HAND_NET_PIX / 2);
    uint32_t v_start = uv_com[1] - (HAND_NET_PIX / 2);

    // Crop the image
    for (uint32_t v = v_start; v < v_start + HAND_NET_PIX; v++) {
      for (uint32_t u = u_start; u < u_start + HAND_NET_PIX; u++) {
        uint32_t dst_index = (v-v_start)* HAND_NET_PIX + (u-u_start);
        uint32_t src_index = v * src_width + u;
        if (label_in[src_index] == 1) {
            cropped_depth[dst_index] = depth_in[src_index];
        } else {
          cropped_depth[dst_index] = 0;
        }
      }
    }

    // Now downsample if needed:
    if (HAND_NET_DOWN_FACT > 1) {
      for (uint32_t v = 0; v < HAND_NET_IM_SIZE; v++) {
        for (uint32_t u = 0; u < HAND_NET_IM_SIZE; u++) {
          uint32_t dst = v*HAND_NET_IM_SIZE + u;
          hand_image[dst] = 0;
          float n_pts = 0;
          for (uint32_t v_off = v; v_off < v + HAND_NET_DOWN_FACT; v_off++) {
            for (uint32_t u_off = u; u_off < u + HAND_NET_DOWN_FACT; u_off++) {
              uint32_t src = HAND_NET_DOWN_FACT * v * HAND_NET_PIX + HAND_NET_DOWN_FACT * u;
              if (cropped_depth[src] > 0) {
                hand_image[dst] += cropped_depth[src];
                n_pts += 1;
              }
            }
          }

          if (n_pts > 0) {
            hand_image[dst] /= n_pts;
          } else {
            hand_image[dst] = 0;
          }
        }
      }
    } else {
      memcpy(hand_image, cropped_depth, 
        sizeof(hand_image[0]) * HAND_NET_IM_SIZE * HAND_NET_IM_SIZE);
    }

    // Now calculate the mean and STD of the cropped downsampled image
    cnt = 0;
    float running_sum = 0;
    float running_sum_sq = 0;
    for (uint32_t v = 0; v < HAND_NET_IM_SIZE; v++) {
      for (uint32_t u = 0; u < HAND_NET_IM_SIZE; u++) {
        uint32_t src_index = v * HAND_NET_IM_SIZE + u;
        float val = hand_image[src_index];
        if (val > EPSILON) {
          cnt++;
          running_sum += val;
          running_sum_sq += (val * val);
        }
      }
    }

    float mean;
    if (cnt == 0) {
      throw std::wruntime_error("ERROR: No non-background pixels found!");
      mean = 0;
      std = 1.0f;  // Something arbitrary
    } else {
      xyz_com.scale(1.0f / (float)(cnt));
      mean = running_sum / (float)cnt;
      float var = (running_sum_sq / (float)cnt) - (mean * mean);
      std = sqrtf(var);
    }
    
    float uvd[3] = {(float)uv_com[0], (float)uv_com[1], mean};
    kinect::OpenNIFuncs::xnConvertProjectiveToRealWorld(1, 
      (kinect::Vector3D*)uvd, (kinect::Vector3D*)xyz_com.m);

    float background = xyz_com[2] + 0.5f * HAND_SIZE;
    float foreground = xyz_com[2] - 0.5f * HAND_SIZE;

    // Now subtract away the forground and set the background to 1
    for (uint32_t v = 0; v < HAND_NET_IM_SIZE; v++) {
      for (uint32_t u = 0; u < HAND_NET_IM_SIZE; u++) {
        uint32_t src_index = v * HAND_NET_IM_SIZE + u;
        float val = hand_image[src_index];
        if (val > EPSILON) {
          hand_image[src_index] = (val - foreground) / HAND_SIZE;
        } else {
          hand_image[src_index] = 1;
        }
      }
    }
  }

}  // namespace hand_model
