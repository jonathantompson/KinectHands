#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <cmath>
#include <sstream>
#include "kinect_interface/hand_net/hand_image_generator.h"
#include "kinect_interface/hand_net/hand_net.h"  // 
#include "kinect_interface/open_ni_funcs.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"  // for GDT_MAX_DIST
#include "jtil/image_util/image_util.h"
#include "jtil/renderer/colors/colors.h"
#include "jtil/exceptions/wruntime_error.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using jtil::math::Float4x4;
using jtil::math::FloatQuat;
using jtil::math::Float3;
using jtil::math::Float4;
using jtil::math::Float2;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using jtil::math::Float3;
using namespace jtil::image_util;

namespace kinect_interface {
namespace hand_net {
 
  HandImageGenerator::HandImageGenerator(const int32_t num_banks) {
    hpf_hand_images_ = NULL;
    size_images_ = 0;
    hand_image_ = NULL;
    im_temp1_ = NULL;
    im_temp2_ = NULL;
    hpf_hand_images_gauss_norm_coeff_ = NULL;
    hpf_hand_images_rect_norm_coeff_ = NULL;
    num_banks_ = num_banks;
    initHandImageData();
    initHPFKernels();
  }

  HandImageGenerator::~HandImageGenerator() {
    releaseData();
  }

  void HandImageGenerator::releaseData() {
    SAFE_DELETE_ARR(hpf_hand_images_);
    SAFE_DELETE_ARR(hand_image_);
    SAFE_DELETE_ARR(im_temp1_);
    SAFE_DELETE_ARR(im_temp2_);
    SAFE_DELETE_ARR(hpf_hand_images_gauss_norm_coeff_);
    SAFE_DELETE_ARR(hpf_hand_images_rect_norm_coeff_);
  }

  void HandImageGenerator::initHandImageData() {
    // Figure out the size of the HPF bank array
    int32_t im_sizeu = HN_IM_SIZE;
    int32_t im_sizev = HN_IM_SIZE;
    size_images_ = 0;
    for (int32_t i = 0; i < num_banks_; i++) {
      size_images_ += (im_sizeu * im_sizev);
      im_sizeu /= 2;
      im_sizev /= 2;
    }

    // Some temporary data structures
    int32_t datasize = std::max<int32_t>(size_images_, 
      HN_SRC_IM_SIZE * HN_SRC_IM_SIZE);
    hand_image_ = new float [datasize];
    im_temp1_ = new float[datasize];
    im_temp2_ = new float[datasize];
    hpf_hand_images_ = new float[datasize];
    hpf_hand_images_gauss_norm_coeff_ = new float[datasize];
    hpf_hand_images_rect_norm_coeff_ = new float[datasize];
  }

  void HandImageGenerator::initHPFKernels() {
#if defined(DEBUG) || defined(_DEBUG)
    if (HN_HPF_KERNEL_SIZE % 2 == 0) {
      throw std::runtime_error("HandImageGenerator::initHPFKernels() - ERROR:"
        " HPF kernel size must be odd!");
    }
#endif

    // Calculate the gaussian coeff
    int32_t center = (HN_HPF_KERNEL_SIZE - 1)/2;
    for (int32_t i = 0; i < HN_HPF_KERNEL_SIZE; i++) {
      gauss_filt_[i] = exp(-pow((float)(i - center) / HN_HPF_SIGMA, 2) * 0.5f);
    }

    // Calculate the rectangular coeff
    for (uint32_t i = 0; i < HN_RECT_KERNEL_SIZE; i++) {
      rect_filt_[i] = 1;
    }

    // Now at each resolution that we care about, filter an image of ones 
    // (where we crop to zero for pixels outside image) to create normalization
    // coefficients per pixel.
    int32_t w = HN_IM_SIZE;
    int32_t h = HN_IM_SIZE;
    float* gauss_coeff = hpf_hand_images_gauss_norm_coeff_;
    float* rect_coeff = hpf_hand_images_rect_norm_coeff_;
    for (int32_t i = 0; i < num_banks_; i++) {
      // Create ones image
      for (int32_t j = 0; j < w * h; j++) {
        im_temp1_[j] = 1.0f;
      }
      // Filter ones image
      ConvolveImageZeroCrop<float>(gauss_coeff, im_temp1_, im_temp2_, w, h, 
        gauss_filt_, HN_HPF_KERNEL_SIZE);

      // Create ones image
      for (int32_t j = 0; j < w * h; j++) {
        im_temp1_[j] = 1.0f;
      }
      // Filter ones image
      ConvolveImageZeroCrop<float>(rect_coeff, im_temp1_, im_temp2_, w, h, 
        rect_filt_, HN_RECT_KERNEL_SIZE);

      // Step to the next image size
      gauss_coeff = &gauss_coeff[w * h];
      rect_coeff = &rect_coeff[w * h];
      w = w / 2;
      h = h / 2;
    }
  }

  void HandImageGenerator::createLabelFromSyntheticDepth(const float* depth, 
    uint8_t* label) {
    for (uint32_t i = 0; i < src_dim; i++) {
      label[i] = (depth[i] > EPSILON && depth[i] < GDT_MAX_DIST) ? 1 : 0;
    }
  }

  // Create the downsampled hand image, background is at 1 and hand is
  // in front of it.
  void HandImageGenerator::calcHandImage(const int16_t* depth_in, 
    const uint8_t* label_in, const bool create_hpf_image, 
    const float* synthetic_depth) {
    calcCroppedHand(depth_in, label_in, synthetic_depth);
    if (create_hpf_image) {
      calcHPFHandBanks();
    }
  }

  void HandImageGenerator::calcCroppedHand(const int16_t* depth_in, 
    const uint8_t* label_in, const float* synthetic_depth) {
    // Find the COM in pixel space so we can crop the image around it
    uint32_t cnt = 0; 
    uvd_com_.zeros();
    for (uint32_t v = 0; v < src_height; v++) {
      for (uint32_t u = 0; u < src_width; u++) {
        uint32_t src_index = v * src_width + u;
        if (label_in[src_index] == 1) {
          uvd_com_[0] += (float)u;
          uvd_com_[1] += (float)v;
          uvd_com_[2] += (float)depth_in[src_index];
          cnt++;
        }
      }
    }
    Float3::scale(uvd_com_, 1.0f / (float)cnt);
    uvd_com_[0] = floor(uvd_com_[0]);
    uvd_com_[1] = floor(uvd_com_[1]);

    int32_t u_start = (int32_t)uvd_com_[0] - (HN_SRC_IM_SIZE / 2);
    int32_t v_start = (int32_t)uvd_com_[1] - (HN_SRC_IM_SIZE / 2);

    // Crop the image and scale between 0.0 and 1.0
    float dmin = uvd_com_[2] - (HN_HAND_SIZE * 0.5f);
    float dmax = uvd_com_[2] + (HN_HAND_SIZE * 0.5f);
    const float background = 1.0f;
    const bool use_synthetic = synthetic_depth != NULL;
    for (int32_t v = v_start; v < v_start + HN_SRC_IM_SIZE; v++) {
      for (int32_t u = u_start; u < u_start + HN_SRC_IM_SIZE; u++) {
        int32_t dst_index = (v-v_start)* HN_SRC_IM_SIZE + (u-u_start);
        if (v >= 0 && v < src_height && u >= 0 && u < src_width) {
          int32_t src_index = v * src_width + u;
          if (!use_synthetic) {
            if (label_in[src_index] == 1) {
              hand_image_[dst_index] = ((float)depth_in[src_index] - dmin) / 
                HN_HAND_SIZE;
            } else {
              hand_image_[dst_index] = background;
            }
          } else {
            if (synthetic_depth[src_index] > EPSILON) {
              hand_image_[dst_index] = (synthetic_depth[src_index] - dmin) / 
                HN_HAND_SIZE;
            } else {
              hand_image_[dst_index] = background;
            }
          }
        } else {
          // Going off the screen
          hand_image_[dst_index] = background;
        }
      }
    }

    // Now in a texture of 256x256 we have a hand image.  This needs to be
    // scaled based on the average depth value
    // The further away the less it is downsampled
    cur_downsample_scale_ = ((float)HN_NOM_DIST * 
      ((float)HN_SRC_IM_SIZE / (float)HN_IM_SIZE)) / uvd_com_[2];
    cur_downsample_scale_ = std::max<float>(cur_downsample_scale_, 1.0f);
    // Find the rectangle in the highres image that will get scaled to the
    // final downsampled image
    int32_t srcw = std::min<int32_t>(HN_SRC_IM_SIZE,
      (int32_t)floor((float)HN_IM_SIZE * cur_downsample_scale_));
    int32_t srch = srcw;
    int32_t srcx = (HN_SRC_IM_SIZE - srcw) / 2;
    int32_t srcy = (HN_SRC_IM_SIZE - srch) / 2;
    cur_downsample_scale_ = (float)srcw /  (float)HN_IM_SIZE;
    // Note FracDownsampleImageSAT destroys the origional source image
    FracDownsampleImageSAT<float>(im_temp1_, 0, 0, HN_IM_SIZE, HN_IM_SIZE,
      HN_IM_SIZE, hand_image_, srcx, srcy, srcw, srch, HN_SRC_IM_SIZE, 
      HN_SRC_IM_SIZE);
    // Now ping-pong buffers
    float* tmp = im_temp1_;
    im_temp1_ = hand_image_;
    hand_image_ = tmp;

    // Save the lower left corner and the width / height
    hand_pos_wh_[0] = ((int32_t)uvd_com_[0] - (HN_SRC_IM_SIZE / 2)) + srcx;
    hand_pos_wh_[1] = ((int32_t)uvd_com_[1] - (HN_SRC_IM_SIZE / 2)) + srcy;
    hand_pos_wh_[2] = srcw;
    hand_pos_wh_[3] = srch;

    // Now downsample as many times as there are banks
    int32_t w = HN_IM_SIZE;
    int32_t h = HN_IM_SIZE;
    float* src = hand_image_;
    for (int32_t i = 1; i < num_banks_; i++) {
      DownsampleImage<float>(&src[w*h], src, w, h, 2);
      src = &src[w*h];
      w /= 2;
      h /= 2;
    }
  }

  void HandImageGenerator::calcHPFHandBanks() {
    int32_t w = HN_IM_SIZE;
    int32_t h = HN_IM_SIZE;
    float* coeff = hpf_hand_images_coeff_;
    float* dst = hpf_hand_images_;
    float* src = hand_image_;
    for (int32_t i = 0; i < num_banks_; i++) {
      // Apply LPF to the source image
      ConvolveImageZeroCrop<float>(dst, src, im_temp1_, w, h, 
        gauss_filt_, HN_HPF_KERNEL_SIZE);
      // Normalize based on the pre-calculated normalization coefficients and
      // create the HPF image by subtracting the LPF image from the inputs src
      for (int32_t j = 0; j < w * h; j++) {
        dst[j] /= coeff[j];
        dst[j] = HN_HPF_GAIN * (src[j] - dst[j]);  // HPF = src - LPF
      }
      // Downsample for the next iteration
      if (i < (num_banks_ - 1)) {
        // Iterate the dst, coeff and src pointers
        src = &src[w * h];
        coeff = &coeff[w * h];
        dst = &dst[w * h];
        // Update the new width and height
#if defined(DEBUG) || defined(_DEBUG)
        if (2*(w / 2) != w || 2*(h / 2) != h) {
          throw std::runtime_error("HandImageGenerator::calcHPFHandBanks(): "
            "ERROR - image size is not divisible by 2!");
        }
#endif
        w = w / 2;
        h = h / 2;
      }
    }
  }

  void HandImageGenerator::annotateFeatsToKinectImage(uint8_t* im, 
    const float* coeff_convnet) const {
    for (uint32_t i = 0; i < HAND_NUM_COEFF_CONVNET; i += FEATURE_SIZE) {
        renderCrossToImageArr(&coeff_convnet[i], im, src_width, 
          src_height, 4, i, hand_pos_wh_);
    }
  }

  void HandImageGenerator::annotateFeatsToHandImage(uint8_t* im, 
    const float* coeff_convnet) const {
    jtil::math::Int4 hand_pos_wh(0, 0, HN_IM_SIZE, HN_IM_SIZE);
    for (uint32_t i = 0; i < HAND_NUM_COEFF_CONVNET; i += FEATURE_SIZE) {
        renderCrossToImageArr(&coeff_convnet[i], im, HN_IM_SIZE, 
          HN_IM_SIZE, 2, i, hand_pos_wh);
    }
  }

  // renderCrossToImageArr - UV is 0 to 1 in U and V
  // Render's directly to the texture array data (not using OpenGL)
  void HandImageGenerator::renderCrossToImageArr(const float* uv, uint8_t* im, 
    const int32_t w, const int32_t h, const int32_t rad, 
    const int32_t color_ind, const jtil::math::Int4& hand_pos_wh) const {
    int32_t u = (int32_t)floor(uv[0] * hand_pos_wh[2]) + hand_pos_wh[0];
    int32_t v = (int32_t)floor(uv[1] * hand_pos_wh[3]) + hand_pos_wh[1];
    v = h - v - 1;

    const Float3* color = 
      &jtil::renderer::colors[(color_ind/2) % jtil::renderer::n_colors];
    const uint8_t r = (uint8_t)(color->m[0] * 255.0f);
    const uint8_t g = (uint8_t)(color->m[1] * 255.0f);
    const uint8_t b = (uint8_t)(color->m[2] * 255.0f);

    // Note: We need to render upside down
    // Render the horizontal cross
    int32_t vcross = v;
    for (int32_t ucross = u - rad; ucross <= u + rad; ucross++) {
      if (ucross >= 0 && ucross < w && vcross >= 0 && vcross < h) {
        int32_t dst_index = vcross * w + ucross;
        im[dst_index * 3] = r;
        im[dst_index * 3+1] = g;
        im[dst_index * 3+2] = b;
      }
    }
    // Render the vertical cross
    int32_t ucross = u;
    for (int32_t vcross = v - rad; vcross <= v + rad; vcross++) {
      if (ucross >= 0 && ucross < w && vcross >= 0 && vcross < h) {
        int32_t dst_index = vcross * w + ucross;
        im[dst_index * 3] = r;
        im[dst_index * 3+1] = g;
        im[dst_index * 3+2] = b;
      }
    }
  }

}  // namespace hand_net
}  // namespace kinect_interface
