//
//  hand_image_generator.h
//
//  Created by Jonathan Tompson on 2/10/13.
//
//  Code to generate a bank of multi-resolution hand images, from a label +
//  depth image.
//

#pragma once

#include "jtil/math/math_types.h"
#include "kinect_interface_primesense/depth_images_io.h"  // for src_dim
#include "kinect_interface_primesense/hand_net/hand_net.h"  // for src_dim
#include "jtil/threading/callback.h"
#include "jtil/data_str/vector.h"

#define HN_SRC_IM_SIZE 384  // U, V size (before downsampling)
#define HN_IM_SIZE 96  // Size after downsampling
#define HN_NOM_DIST 500  // Downsample is exactly HN_SRC_IM_SIZE:HN_IM_SIZE at this depth
#define HN_HAND_SIZE 300.0f
#define HN_DEFAULT_NUM_CONV_BANKS 3
#define HN_HPF_GAIN 2.0f 
#define HN_RECT_KERNEL_SIZE 9  // Clemont recommends 5x5 (aggressive), must be odd --> Was 11
#define HN_CONTRAST_NORM_THRESHOLD 5e-2f  // Was 2e-2f
#define HN_LOCAL_CONTRAST_NORM  // Otherwise subtractive local, divisive global --> Was undefined
// #define DOWNSAMPLE_POINT  // Low quality downsample (but fast!)

// #define HN_USE_RECT_LPF_KERNEL  // Otherwise use gaussian --> Clemont recommends rect.

namespace jtil { namespace data_str { template <typename T> class Vector; } }
namespace jtorch {  
  template <typename T> class Tensor;
  class SpatialContrastiveNormalization;
  class SpatialSubtractiveNormalization;
  class TorchStage;
  class Table;
  class Parallel;
}

namespace kinect_interface_primesense {
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
    // hand_size_modifier is a hack to make the convnet hand look bigger than
    // it actuall is.  It allows us to set a constant hand scale for different
    // users.
    void calcHandImage(const int16_t* depth_in, const uint8_t* label_in,
      const float hand_size_modifier = 1.0f,
      const float* synthetic_depth = NULL, const bool flip = false);

    void annotateFeatsToKinectImage(uint8_t* im,
      const float* coeff_convnet) const;  // 640 x 480
    void annotateFeatsToHandImage(uint8_t* im, 
      const float* coeff_convnet) const;  // 96 x 96

    void createHeatMap(float* hm, const uint32_t size, 
      const float* coeff_convnet, const float std);

    void calcNormalImage(float* normals_xyz, const float* xyz, 
      const uint8_t* labels);

    // Getter methods
    jtorch::Table* hpf_hand_image() { return hpf_hand_image_; }
    jtorch::Table* hand_image() { return hand_image_; }
    const float* hpf_hand_image_cpu() { return hpf_hand_image_cpu_; }
    const float* hand_image_cpu() { return hand_image_cpu_; }
    const float* cropped_hand_image() { return cropped_hand_image_; }
    uint32_t size_images() const { return size_images_; }
    
    inline const jtil::math::Float3& uvd_com() const { return uvd_com_; }
    inline const jtil::math::Int4& hand_pos_wh() const { return hand_pos_wh_; }

  private:
    int32_t num_banks_;
    uint32_t size_images_;
    float* cropped_hand_image_;
    jtorch::Table* hand_image_;
    jtorch::Table* hpf_hand_image_;  // NOT OWNED HERE!
    float* hpf_hand_image_cpu_;
    float* hand_image_cpu_;
    float cur_downsample_scale_;
    jtil::math::Float3 uvd_com_;  // UV COM of the hand image.
    jtil::math::Int4 hand_pos_wh_;  // Lower left pos and width/height of the hand image
    double* im_temp_double_;
    jtorch::Parallel* norm_module_;  // One per bank
    jtil::data_str::Vector<jtil::math::Int3> hand_mesh_indices_;
    jtil::data_str::Vector<jtil::math::Float3> hand_mesh_vertices_;
    jtil::data_str::Vector<jtil::math::Float3> hand_mesh_normals_;
    const NormalApproximationMethod norm_method_;

    void calcCroppedHand(const int16_t* depth_in, const uint8_t* label_in, 
      float hand_size_modifier, const float* synthetic_depth = NULL,
      const bool flip = false);
    void calcHPFHandBanks();
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
};  // namespace kinect_interface_primesense
