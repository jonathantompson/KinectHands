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
#include "jtil/data_str/vector.h"

#define HN_SRC_IM_SIZE 384  // U, V size (before downsampling)
#define HN_IM_SIZE 96  // Size after downsampling
#define HN_NOM_DIST 500  // Downsample is exactly HN_SRC_IM_SIZE:HN_IM_SIZE at this depth
#define HN_HAND_SIZE 300.0f
#define HN_DEFAULT_NUM_CONV_BANKS 3
#define HN_HPF_GAIN 2.0f
#define HN_RECT_KERNEL_SIZE 11  // Clemont recommends 5x5 (aggressive), must be odd
// #define HN_LOCAL_CONTRAST_NORM  // Otherwise subtractive local, divisive global

#define HN_USE_RECT_LPF_KERNEL  // Otherwise use gaussian --> Clemont recommends rect.

namespace jtil { namespace threading { class ThreadPool; } }
namespace jtil { namespace data_str { template <typename T> class Vector; } }
namespace jtorch {  
  template <typename T> class Tensor;
  class SpatialContrastiveNormalization;
  class SpatialSubtractiveNormalization;
}

namespace kinect_interface {
namespace hand_net {

  typedef enum {
    BasicNormalApproximation,  // average normals (no weighting)
    SimpleNormalApproximation,  // average normals around vert weighted by area
    RobustNormalApproximation,  // average normals weighted by angle at the vert
  } NormalApproximationMethod;
  
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
      const bool create_hpf_image, jtil::threading::ThreadPool* tp, 
      const float* synthetic_depth = NULL);

    void annotateFeatsToKinectImage(uint8_t* im,
      const float* coeff_convnet) const;  // 640 x 480
    void annotateFeatsToHandImage(uint8_t* im, 
      const float* coeff_convnet) const;  // 96 x 96

    void calcNormalImage(float* normals_xyz, const float* xyz, 
      const uint8_t* labels);

    // Getter methods
    const float* hpf_hand_image() { return hpf_hand_image_; }
    const float* hand_image() { return hand_image_; }
    const float* cropped_hand_image() { return cropped_hand_image_; }
    const int32_t size_images() { return size_images_; } 
    inline const jtil::math::Float3& uvd_com() const { return uvd_com_; }
    inline const jtil::math::Int4& hand_pos_wh() const { return hand_pos_wh_; }

  private:
    int32_t num_banks_;
    float* cropped_hand_image_;
    float* hand_image_;
    float cur_downsample_scale_;
    int32_t size_images_;  // Default: HAND_NET_IM_SIZE^2 *(1 + 1/(2*2) + 1/(4*4))
    float* hpf_hand_image_;
    jtil::math::Float3 uvd_com_;  // UV COM of the hand image.
    jtil::math::Int4 hand_pos_wh_;  // Lower left pos and width/height of the hand image
    double* im_temp_double_;
    jtorch::TorchStage** norm_module_;  // One per bank
    jtorch::Tensor<float>** norm_module_input_;
    jtil::data_str::Vector<jtil::math::Int3> hand_mesh_indices_;
    jtil::data_str::Vector<jtil::math::Float3> hand_mesh_vertices_;
    jtil::data_str::Vector<jtil::math::Float3> hand_mesh_normals_;
    const NormalApproximationMethod norm_method_;

    void calcCroppedHand(const int16_t* depth_in, const uint8_t* label_in, 
      const float* synthetic_depth = NULL);
    void calcHPFHandBanks(jtil::threading::ThreadPool* tp);
    void releaseData();  // Call destructor on all dynamic data
    void initHandImageData();
    void renderCrossToImageArr(const float* uv, uint8_t* im, const int32_t w, 
      const int32_t h, const int32_t rad, const int32_t color_ind,
      const jtil::math::Int4& hand_pos_wh) const;
    void calcNormalUnNormalized(jtil::math::Float3& normal, 
      const jtil::math::Float3& pt0, const jtil::math::Float3& pt1, 
      const jtil::math::Float3& pt2);
    void calcNormal(jtil::math::Float3& normal, const jtil::math::Float3& pt0,
      const jtil::math::Float3& pt1, const jtil::math::Float3& pt2);
    float calcAngleSafe(const jtil::math::Float3& pt0, 
      const jtil::math::Float3& pt1, const jtil::math::Float3& pt2);

    // Create hand geometry from xyz + label data
    void createHandMeshVertices(const float* xyz);  // Step 1
    void createHandMeshIndices(const uint8_t* labels);  // Step 2
    void createHandMeshNormals();  // Step 3

    // Non-copyable, non-assignable.
    HandImageGenerator(HandImageGenerator&);
    HandImageGenerator& operator=(const HandImageGenerator&);
  };
};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_NET_HAND_IMAGE_GENERATOR_HEADER
