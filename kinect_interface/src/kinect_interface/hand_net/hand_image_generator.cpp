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
#include "kinect_interface/hand_net/spatial_contrastive_normalization.h"
#include "kinect_interface/hand_net/spatial_subtractive_normalization.h"
#include "kinect_interface/hand_net/float_tensor.h"
#include "jtil/image_util/image_util.h"
#include "jtil/renderer/colors/colors.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/file_io/file_io.h"

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
using namespace jtil::threading;
using namespace jtil::math;
using namespace jtil::data_str;

namespace kinect_interface {
namespace hand_net {
 
  HandImageGenerator::HandImageGenerator(const int32_t num_banks) : 
    norm_method_(BasicNormalApproximation){
    hpf_hand_image_ = NULL;
    size_images_ = 0;
    hand_image_ = NULL;
    cropped_hand_image_ = NULL;
    im_temp_double_ = NULL;
    num_banks_ = num_banks;
    norm_module_ = NULL;
    norm_module_input_ = NULL;
    hand_mesh_normals_.capacity(src_dim);
    hand_mesh_vertices_.capacity(src_dim);
    initHandImageData();
  }

  HandImageGenerator::~HandImageGenerator() {
    releaseData();
  }

  void HandImageGenerator::releaseData() {
    SAFE_DELETE_ARR(hpf_hand_image_);
    SAFE_DELETE_ARR(hand_image_);
    SAFE_DELETE_ARR(cropped_hand_image_);
    SAFE_DELETE_ARR(im_temp_double_);
    if (norm_module_) {
      for (int32_t i = 0; i < num_banks_; i++) {
        SAFE_DELETE(norm_module_[i]);
      }
    }
    SAFE_DELETE_ARR(norm_module_);
    if (norm_module_input_) {
      for (int32_t i = 0; i < num_banks_; i++) {
        SAFE_DELETE(norm_module_input_[i]);
      }
    }
    SAFE_DELETE_ARR(norm_module_input_);
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
    cropped_hand_image_ = new float[HN_SRC_IM_SIZE * HN_SRC_IM_SIZE];
    im_temp_double_ = new double[HN_SRC_IM_SIZE * HN_SRC_IM_SIZE];

    hpf_hand_image_ = new float[datasize];
#if defined(DEBUG) || defined(_DEBUG)
    if (HN_RECT_KERNEL_SIZE % 2 == 0) {
      throw std::runtime_error("HandImageGenerator::initHPFKernels() - ERROR:"
        " HN_RECT_KERNEL_SIZE must be odd!");
    }
#endif

    norm_module_ = new TorchStage*[num_banks_];
    norm_module_input_ = new FloatTensor*[num_banks_];
    im_sizeu = HN_IM_SIZE;
    im_sizev = HN_IM_SIZE;
    for (int32_t i = 0; i < num_banks_; i++) {
      FloatTensor* kernel = FloatTensor::ones1D(HN_RECT_KERNEL_SIZE);
      norm_module_input_[i] = 
        new FloatTensor(Int4(im_sizeu, im_sizev, 1, 1));
      norm_module_[i] = new SpatialSubtractiveNormalization(*kernel);
      delete kernel;
      im_sizeu /= 2;
      im_sizev /= 2;
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
    const uint8_t* label_in, const bool create_hpf_image, ThreadPool* tp, 
    const float* synthetic_depth) {
    calcCroppedHand(depth_in, label_in, synthetic_depth);
    if (create_hpf_image) {
      calcHPFHandBanks(tp);
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
              cropped_hand_image_[dst_index] = 
                ((float)depth_in[src_index] - dmin) / HN_HAND_SIZE;
            } else {
              cropped_hand_image_[dst_index] = background;
            }
          } else {
            if (synthetic_depth[src_index] > EPSILON) {
              cropped_hand_image_[dst_index] = 
                (synthetic_depth[src_index] - dmin) / HN_HAND_SIZE;
            } else {
              cropped_hand_image_[dst_index] = background;
            }
          }
        } else {
          // Going off the screen
          cropped_hand_image_[dst_index] = background;
        }
      }
    }

    //// Remove any outliers
    //for (int32_t v = 0; v < HN_SRC_IM_SIZE; v++) {
    //  for (int32_t u = 0; u < HN_SRC_IM_SIZE; u++) {

    //  }
    //}

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
    // Note FracDownsampleImageSAT destroys the origional source image
    FracDownsampleImageSAT<float>(hand_image_, 0, 0, HN_IM_SIZE, HN_IM_SIZE,
      HN_IM_SIZE, cropped_hand_image_, srcx, srcy, srcw, srch, HN_SRC_IM_SIZE, 
      HN_SRC_IM_SIZE, im_temp_double_);

    // Save the lower left corner and the width / height
    hand_pos_wh_[0] = u_start + srcx;
    hand_pos_wh_[1] = v_start + srcy;
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

  void HandImageGenerator::calcHPFHandBanks(ThreadPool* tp) {
    int32_t w = HN_IM_SIZE;
    int32_t h = HN_IM_SIZE;
    float* dst = hpf_hand_image_;
    float* src = hand_image_;
    for (int32_t i = 0; i < num_banks_; i++) {
      // Apply local contrast normalization
      memcpy(norm_module_input_[i]->data(), src, 
        w * h * sizeof(norm_module_input_[i]->data()[0]));
      norm_module_[i]->forwardProp(*norm_module_input_[i], 
        *tp);
      memcpy(dst, ((FloatTensor*)norm_module_[i]->output)->data(), 
        w * h * sizeof(dst[0]));

      // Now subtract by the GLOBAL std
      float sum = 0;
      float sum_sqs = 0;
      for (int32_t j = 0; j < w * h; j++) {
        sum += dst[j];
        sum_sqs += dst[j] * dst[j];
      }
      // Normalize
      sum *= (1.0f / (float)(w * h));
      sum_sqs *= (1.0f / (float)(w * h));
      float var = sqrtf(sum_sqs - sum*sum);
      float scale_fact = 1.0f / var;
      for (int32_t j = 0; j < w * h; j++) {
        dst[j] *= scale_fact;
      }

      if (i < (num_banks_ - 1)) {
        // Iterate the dst, coeff and src pointers
        src = &src[w * h];
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
          HN_IM_SIZE, 1, i, hand_pos_wh);
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

  void HandImageGenerator::createHandMeshVertices(const float* xyz) {
    hand_mesh_vertices_.resize(src_dim);
    for (uint32_t i = 0; i < src_dim; i++) {
      hand_mesh_vertices_[i].set(xyz[i*3], xyz[i*3 + 1], xyz[i*3 + 2]);
    }
  }

  void HandImageGenerator::createHandMeshIndices(const uint8_t* labels) {
    hand_mesh_indices_.resize(0);  // Set the size to zero without deallocation
    uint32_t verts_case;
    uint32_t p0, p1, p2, p3;
  
    // Populate the mesh data with faces
    // Step through the UV map examining each quad of 4 neighbouring vertices.
    // If 3 or more of those vertices are part of the hand, then add the
    // corresponding face.
    for (uint32_t v = 0; v < (src_height-1); v++) {
      for (uint32_t u = 0; u < (src_width-1); u++) {
        p0 = v*src_width + u;          // top left
        p1 = v*src_width + (u+1);      // top right
        p2 = (v+1)*src_width + u;      // bottom left
        p3 = (v+1)*src_width + (u+1);  // bottom right
      
        // Early out for a large number of quads
        if (labels[p0] == 0 && labels[p1] == 0) {
          continue;
        }
      
        // For each box of 4 vertices, if 3 of them have valid points, add
        // a triangle, if 4 of them have vertices add 4 triangles
        verts_case = 0;
        if (labels[p0] != 0) {
          verts_case = verts_case | 1;
        }
        if (labels[p1] != 0) {
          verts_case = verts_case | 2;
        }
        if (labels[p2] != 0) {
          verts_case = verts_case | 4;
        }
        if (labels[p3] != 0) {
          verts_case = verts_case | 8;
        } 
      
        // Recall: Front face is counter-clockwise
        switch(verts_case) { 
          case 7:  // 0111 = p2 & p1 & p0
            hand_mesh_indices_.pushBack(Int3(p0, p2, p1));
            break;
          case 11:  // 1011 = p3 & p1 & p0
            hand_mesh_indices_.pushBack(Int3(p0, p3, p1));           
            break;
          case 13:  // 1101 = p3 & p2 & p0
            hand_mesh_indices_.pushBack(Int3(p0, p2, p3));              
            break;      
          case 14:  // 1110 = p3 & p2 & p1
            hand_mesh_indices_.pushBack(Int3(p1, p2, p3));                   
            break;   
          case 15:  // 1111 = p3 & p2 & p1 & p0
            // THIS METHOD JUST SPLITS EVERY QUAD DOWN THE SAME DIAGONAL
            hand_mesh_indices_.pushBack(Int3(p0, p2, p1));   
            hand_mesh_indices_.pushBack(Int3(p3, p1, p2));        
            break;
        }
      }
    }
  }

  void HandImageGenerator::createHandMeshNormals() {
    hand_mesh_normals_.resize(src_dim);
    
    // Set all the normals as 0
    for (uint32_t i = 0; i < hand_mesh_normals_.size(); i++) {
      hand_mesh_normals_[i].zeros();
    }
    
    // For each face in the index array, calculate its normal and add it to the
    // normal accumulation
    Float3 cur_normal;
    for (uint32_t i = 0; i < hand_mesh_indices_.size(); i++) {
      uint32_t v1 = hand_mesh_indices_[i][0];
      uint32_t v2 = hand_mesh_indices_[i][1];
      uint32_t v3 = hand_mesh_indices_[i][2];
      
      switch (norm_method_) {
      case BasicNormalApproximation:
        // METHOD 1 --> FAST
        // If you sum the normal weighted by the tri area, then you get a better
        // average normal.  More importantly, if you calculate the normal by 
        // cross product then you get a normal whos lenght is 2 x the triangle 
        // area, so summing these gives us the correct weights.
        calcNormal(cur_normal, hand_mesh_vertices_[v1], 
          hand_mesh_vertices_[v2], hand_mesh_vertices_[v3]);
        Float3::add(hand_mesh_normals_[v1], hand_mesh_normals_[v1], cur_normal);
        Float3::add(hand_mesh_normals_[v2], hand_mesh_normals_[v2], cur_normal);
        Float3::add(hand_mesh_normals_[v3], hand_mesh_normals_[v3], cur_normal);
        break;
      case SimpleNormalApproximation:
        // METHOD 1 --> FAST
        // If you sum the normal weighted by the tri area, then you get a better
        // average normal.  More importantly, if you calculate the normal by 
        // cross product then you get a normal whos lenght is 2 x the triangle 
        // area, so summing these gives us the correct weights.
        calcNormalUnNormalized(cur_normal, hand_mesh_vertices_[v1], 
          hand_mesh_vertices_[v2], hand_mesh_vertices_[v3]);
        Float3::add(hand_mesh_normals_[v1], hand_mesh_normals_[v1], cur_normal);
        Float3::add(hand_mesh_normals_[v2], hand_mesh_normals_[v2], cur_normal);
        Float3::add(hand_mesh_normals_[v3], hand_mesh_normals_[v3], cur_normal);
        break;
      case RobustNormalApproximation:
        // METHOD 2 --> EXPENSIVE
        // Weight the normals by the angle they form at the vertex.
        calcNormalUnNormalized(cur_normal, hand_mesh_vertices_[v1], 
          hand_mesh_vertices_[v2], hand_mesh_vertices_[v3]);
        // V2 Angle
        float angle = calcAngleSafe(hand_mesh_vertices_[v1], 
          hand_mesh_vertices_[v2], hand_mesh_vertices_[v3]);
        Float3* norm = hand_mesh_normals_.at(v2);
        norm->m[0] += (cur_normal[0] * angle);
        norm->m[1] += (cur_normal[1] * angle);
        norm->m[2] += (cur_normal[2] * angle);
        // V1 Angle
        angle = calcAngleSafe(hand_mesh_vertices_[v3], hand_mesh_vertices_[v1], 
          hand_mesh_vertices_[v2]);
        norm = hand_mesh_normals_.at(v1);
        norm->m[0] += (cur_normal[0] * angle);
        norm->m[1] += (cur_normal[1] * angle);
        norm->m[2] += (cur_normal[2] * angle);
        // V3 Angle
        angle = calcAngleSafe(hand_mesh_vertices_[v2], hand_mesh_vertices_[v3], 
          hand_mesh_vertices_[v1]);
        norm = hand_mesh_normals_.at(v3);
        norm->m[0] += (cur_normal[0] * angle);
        norm->m[1] += (cur_normal[1] * angle);
        norm->m[2] += (cur_normal[2] * angle);
        break;
      }
    }

    for (uint32_t i = 0; i < hand_mesh_normals_.size(); i++) { 
      // No need to divide by the number of faces...  Just normalize
      hand_mesh_normals_[i].normalize();
    }
  }

  void HandImageGenerator::calcNormalUnNormalized(jtil::math::Float3& normal, 
    const jtil::math::Float3& pt0, const jtil::math::Float3& pt1, 
    const jtil::math::Float3& pt2) {
    Float3 tmp1_, tmp2_;
    Float3::sub(tmp1_, pt0, pt1);
    Float3::sub(tmp2_, pt2, pt1);
    Float3::cross(normal, tmp1_, tmp2_);
  }

  void HandImageGenerator::calcNormal(jtil::math::Float3& normal, 
    const jtil::math::Float3& pt0, const jtil::math::Float3& pt1, 
    const jtil::math::Float3& pt2) {
    Float3 tmp1_, tmp2_;
    Float3::sub(tmp1_, pt0, pt1);
    Float3::sub(tmp2_, pt2, pt1);
    Float3::cross(normal, tmp1_, tmp2_);
    normal.normalize();
  }

  float HandImageGenerator::calcAngleSafe(const Float3& pt0, const Float3& pt1, 
    const Float3& pt2)  {
    Float3 tmp1_, tmp2_;
    Float3::sub(tmp1_, pt0, pt1);
    Float3::sub(tmp2_, pt2, pt1);
    tmp1_.normalize();
    tmp2_.normalize();
    float dot = Float3::dot(tmp1_, tmp2_);
    dot = dot > 1 ? 1 : dot;
    dot = dot < -1 ? -1 : dot;
    return acosf(dot);
  }

  void HandImageGenerator::calcNormalImage(float* normals_xyz,
    const float* xyz, const uint8_t* labels) {
    createHandMeshVertices(xyz);
    createHandMeshIndices(labels);
    createHandMeshNormals();
    for (uint32_t i = 0; i < src_dim; i++) {
      normals_xyz[i*3] = hand_mesh_normals_[i][0];
      normals_xyz[i*3 + 1] = hand_mesh_normals_[i][1];
      normals_xyz[i*3 + 2] = hand_mesh_normals_[i][2];
    }
  }

}  // namespace hand_net
}  // namespace kinect_interface
