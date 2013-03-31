//
//  hand_image_generator.h
//
//  Created by Jonathan Tompson on 2/10/13.
//
//  Code to generate a bank of multi-resolution hand images, from a label +
//  depth image.
//

#ifndef KINECT_INTERFACE_HAND_NET_HAND_IMAGE_GENERATOR_HEADER
#define KINECT_INTERFACE_HAND_NET_HAND_IMAGE_GENERATOR_HEADER

#include "jtil/math/math_types.h"
#include "kinect_interface/depth_images_io.h"  // for src_dim
#include "kinect_interface/hand_net/hand_net.h"  // for src_dim
#include "jtil/threading/callback.h"

#define HN_SRC_IM_SIZE 256  // U, V size (before downsampling)
#define HN_IM_SIZE 96  // Size after downsampling
#define HN_NOM_DIST 700  // Downsample is exactly 1:4 at this depth
#define HN_HAND_SIZE 300.0f
#define HN_DEFAULT_NUM_CONV_BANKS 3
#define HN_HPF_GAIN 2.0f
#define HN_HPF_SIGMA 1.5f  // in pixels
#define HN_HPF_KERNEL_SIZE 11  // Hopefully >= 2*(3*sigma) + 1 (MUST BE ODD!)
#define HN_RECT_KERNEL_SIZE 5  // Clemont recommends 5x5 (aggressive)

#define HN_USE_RECT_LPF_KERNEL  // Otherwise use gaussian --> Clemont recommends rect.

namespace kinect_interface {

namespace hand_net {
  
  class HandImageGenerator {
  public:
    // Constructor / Destructor
    HandImageGenerator(const int32_t num_banks);
    ~HandImageGenerator();

    void createLabelFromSyntheticDepth(const float* depth, uint8_t* label);

    // calcHandImage - creates cropped image, then creates a bank of HPF imgs
    // if synthetic_depth != NULL, then the crop window will be chosen using
    // the real depth, but the synthetic depth will be stored
    void calcHandImage(const int16_t* depth_in, const uint8_t* label_in,
      const bool create_hpf_image, const float* synthetic_depth = NULL);

    void HandImageGenerator::annotateFeatsToKinectImage(uint8_t* im, 
      const float* coeff_convnet) const;  // 640 x 480
    void HandImageGenerator::annotateFeatsToHandImage(uint8_t* im, 
      const float* coeff_convnet) const;  // 96 x 96

    // Getter methods
    const float* hpf_hand_images() { return hpf_hand_images_; }
    const float* hand_image() { return hand_image_; }
    const int32_t size_images() { return size_images_; } 
    inline const jtil::math::Float3& uvd_com() const { return uvd_com_; }
    inline const jtil::math::Int4& hand_pos_wh() const { return hand_pos_wh_; }

  private:
    int32_t num_banks_;
    float* hand_image_;
    float cur_downsample_scale_;
    int32_t size_images_;  // Default: HAND_NET_IM_SIZE^2 *(1 + 1/(2*2) + 1/(4*4))
    float* hpf_hand_images_;
    float* hpf_hand_images_gauss_norm_coeff_;  // integral of a ones image with guass filt
    float* hpf_hand_images_rect_norm_coeff_;  // integral of a ones image with norm filt
    jtil::math::Float3 uvd_com_;  // UV COM of the hand image.
    jtil::math::Int4 hand_pos_wh_;  // Lower left pos and width/height of the hand image
    float gauss_filt_[HN_HPF_KERNEL_SIZE];  // This is unnormalized!
    float rect_filt_[HN_RECT_KERNEL_SIZE];  // This is unnormalized!
    float* im_temp1_;
    float* im_temp2_;

    void calcCroppedHand(const int16_t* depth_in, const uint8_t* label_in, 
      const float* synthetic_depth = NULL);
    void calcHPFHandBanks();
    void initHPFKernels();
    void releaseData();  // Call destructor on all dynamic data
    void initHandImageData();
    void renderCrossToImageArr(const float* uv, uint8_t* im, const int32_t w, 
      const int32_t h, const int32_t rad, const int32_t color_ind,
      const jtil::math::Int4& hand_pos_wh) const;

    // Non-copyable, non-assignable.
    HandImageGenerator(HandImageGenerator&);
    HandImageGenerator& operator=(const HandImageGenerator&);
  };
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_HAND_IMAGE_GENERATOR_HEADER
