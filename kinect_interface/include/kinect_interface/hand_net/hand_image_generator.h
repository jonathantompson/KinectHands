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
#include "jtil/threading/callback.h"

#define HN_SRC_IM_SIZE 256  // U, V size (before downsampling)
#define HN_IM_SIZE 96  // Size after downsampling
#define HN_NOM_DIST 700  // Downsample is exactly 1:4 at this depth
#define HN_HAND_SIZE 300.0f
#define HN_DEFAULT_NUM_CONV_BANKS 3
#define HN_HPF_GAIN 2.0f
#define HN_NUM_WORKER_THREADS 8
#define HN_HPF_SIGMA 1.5f  // in pixels
#define HN_HPF_KERNEL_SIZE 11  // Hopefully >= 2*(3*sigma) + 1 (MUST BE ODD!)

namespace kinect_interface {

namespace hand_net {
  
  class HandImageGenerator {
  public:
    // Constructor / Destructor
    HandImageGenerator(const int32_t num_banks);
    ~HandImageGenerator();

    void createLabelFromSyntheticDepth(const float* depth, uint8_t* label);

    // calcHandImage - creates cropped image, then creates a bank of HPF imgs
    void calcHandImage(const int16_t* depth_in, const uint8_t* label_in,
      const bool create_hpf_image);

    // Getter methods
    const float* hpf_hand_images() { return hpf_hand_images_; }
    const float* hand_image() { return hand_image_; }
    const int32_t size_images() { return size_images_; } 
    inline const jtil::math::Float3& uvd_com() const { return uvd_com_; }

  private:
    int32_t num_banks_;
    float* hand_image_;
    float cur_downsample_scale_;
    int32_t size_images_;  // Default: HAND_NET_IM_SIZE^2 *(1 + 1/(2*2) + 1/(4*4))
    float* hpf_hand_images_;
    float* hpf_hand_images_coeff_;  // integral of a ones image with guass filt
    jtil::math::Float3 uvd_com_;  // UV COM of the hand image.
    float gauss_filt_[HN_HPF_KERNEL_SIZE];  // This is unnormalized!
    float* im_temp1_;
    float* im_temp2_;

    void calcCroppedHand(const int16_t* depth_in, const uint8_t* label_in);
    void calcHPFHandBanks();
    void initHPFKernels();
    void releaseData();  // Call destructor on all dynamic data
    void initHandImageData();

    // Non-copyable, non-assignable.
    HandImageGenerator(HandImageGenerator&);
    HandImageGenerator& operator=(const HandImageGenerator&);
  };
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_HAND_IMAGE_GENERATOR_HEADER
