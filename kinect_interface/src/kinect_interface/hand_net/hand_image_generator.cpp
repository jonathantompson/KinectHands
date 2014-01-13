#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <cmath>
#include <sstream>
#include "kinect_interface/kinect_interface.h"
#include "kinect_interface/hand_net/hand_image_generator.h"
#include "kinect_interface/hand_net/hand_net.h"
#include "jtorch/spatial_contrastive_normalization.h"
#include "jtorch/spatial_subtractive_normalization.h"
#include "jtorch/spatial_divisive_normalization.h"
#include "jtorch/tensor.h"
#include "jtorch/table.h"
#include "jtorch/parallel.h"
#include "jtorch/sequential.h"
#include "jtil/image_util/image_util.h"
#include "jtil/renderer/colors/colors.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/file_io/file_io.h"
#include "jtil/settings/settings_manager.h"

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
using namespace jtorch;

namespace kinect_interface {
namespace hand_net {
 
  HandImageGenerator::HandImageGenerator(const int32_t num_banks) : 
    norm_method_(BasicNormalApproximation){
    hpf_hand_image_ = NULL;
    hand_image_ = NULL;
    hand_image_cpu_ = NULL;
    cropped_hand_image_ = NULL;
    im_temp_double_ = NULL;
    num_banks_ = num_banks;
    norm_module_ = NULL;
    hpf_hand_image_cpu_ = NULL;
    hand_mesh_normals_.capacity(depth_dim);
    hand_mesh_vertices_.capacity(depth_dim);
    initHandImageData();
  }

  HandImageGenerator::~HandImageGenerator() {
    releaseData();
  }

  void HandImageGenerator::releaseData() {
    SAFE_DELETE(hand_image_);
    SAFE_DELETE_ARR(hand_image_cpu_);
    SAFE_DELETE_ARR(cropped_hand_image_);
    SAFE_DELETE_ARR(im_temp_double_);
    SAFE_DELETE_ARR(hpf_hand_image_cpu_);
    SAFE_DELETE(norm_module_);
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
    hand_image_cpu_ = new float [datasize];
    hpf_hand_image_cpu_ = new float[datasize];
    cropped_hand_image_ = new float[HN_SRC_IM_SIZE * HN_SRC_IM_SIZE];
    im_temp_double_ = new double[HN_SRC_IM_SIZE * HN_SRC_IM_SIZE];

    // OpenCL structures
    if (jtorch::cl_context == NULL) {
      throw std::wruntime_error("HandImageGenerator::initHandImageData() - "
        "ERROR: jtorch has not been initialized!"); 
    }
    hand_image_ = new Table();
#if defined(DEBUG) || defined(_DEBUG)
    if (HN_RECT_KERNEL_SIZE % 2 == 0) {
      throw std::runtime_error("HandImageGenerator::initHPFKernels() - ERROR:"
        " HN_RECT_KERNEL_SIZE must be odd!");
    }
#endif

    norm_module_ = new Parallel();
    im_sizeu = HN_IM_SIZE;
    im_sizev = HN_IM_SIZE;
    for (int32_t i = 0; i < num_banks_; i++) {
      hand_image_->add(new Tensor<float>(Int2(im_sizeu, im_sizev)));
#ifdef HN_USE_RECT_LPF_KERNEL
      Tensor<float>* kernel = Tensor<float>::ones1D(HN_RECT_KERNEL_SIZE);
#else
      Tensor<float>* kernel = Tensor<float>::gaussian1D(HN_RECT_KERNEL_SIZE);
#endif
#ifdef HN_LOCAL_CONTRAST_NORM
      /*
      Sequential* seq = new Sequential();
      norm_module_->add(seq);
      seq->add(new SpatialSubtractiveNormalization(*kernel));
      kernel = Tensor<float>::ones1D(HN_RECT_KERNEL_SIZE+10);
      seq->add(new SpatialDivisiveNormalization(*kernel));
      */
      norm_module_->add(new SpatialContrastiveNormalization(kernel, 
        HN_CONTRAST_NORM_THRESHOLD));
#else
      norm_module_->add(new SpatialSubtractiveNormalization(*kernel));
#endif
      delete kernel;
      im_sizeu /= 2;
      im_sizev /= 2;
    }
  }

  void HandImageGenerator::createLabelFromSyntheticDepth(const float* depth, 
    uint8_t* label) {
    for (uint32_t i = 0; i < depth_dim; i++) {
      label[i] = (depth[i] > EPSILON && depth[i] < GDT_MAX_DIST) ? 1 : 0;
    }
  }

  // Create the downsampled hand image, background is at 1 and hand is
  // in front of it.
  void HandImageGenerator::calcHandImage(const int16_t* depth_in, 
    const uint8_t* label_in, const float hand_size_modifier, 
    const float* synthetic_depth, const bool flip) {
    if (hand_size_modifier > 1.0f) {
      throw std::wruntime_error("HandImageGenerator::calcHandImage() - ERROR: "
        "hand_size_modifier > 1.0f!");
    }
    calcCroppedHand(depth_in, label_in, hand_size_modifier, synthetic_depth,
      flip);

    calcHPFHandBanks();
  }

  void HandImageGenerator::calcCroppedHand(const int16_t* depth_in, 
    const uint8_t* label_in, float hand_size_modifier, 
    const float* synthetic_depth, const bool flip) {
    // Find the COM in pixel space so we can crop the image around it
    // TO DO: Implement accumulate as described here and put this in OpenCL:
    // http://www.icg.tugraz.at/courses/lv710.092/ezg2uebung1
    uint32_t cnt = 0; 
    uvd_com_.zeros();
    for (uint32_t v = 0; v < depth_h; v++) {
      for (uint32_t u = 0; u < depth_w; u++) {
        uint32_t src_index = v * depth_w + u;
        if (label_in[src_index] == 1) {
          uvd_com_[0] += (float)u;
          uvd_com_[1] += (float)v;
          uvd_com_[2] += (float)depth_in[src_index];
          cnt++;
        }
      }
    }
    if (cnt != 0) {
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
//#pragma omp parallel for num_threads(4)
      for (int32_t v = v_start; v < v_start + HN_SRC_IM_SIZE; v++) {
        for (int32_t u = u_start; u < u_start + HN_SRC_IM_SIZE; u++) {
          int32_t dst_index = (v-v_start)* HN_SRC_IM_SIZE + (u-u_start);
          if (v >= 0 && v < depth_h && u >= 0 && u < depth_w) {
            int32_t src_index = v * depth_w + u;
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

      // Now in a texture of 384x384 we have a hand image.  This needs to be
      // scaled based on the average depth value
      // The further away the less it is downsampled
      cur_downsample_scale_ =  ((float)HN_NOM_DIST * 
        ((float)HN_SRC_IM_SIZE / (float)HN_IM_SIZE)) / uvd_com_[2];
      cur_downsample_scale_ = std::max<float>(cur_downsample_scale_, 1.0f);
      if (cur_downsample_scale_ * hand_size_modifier < 1.0f) {
        hand_size_modifier = 1.0f / cur_downsample_scale_;
      }

      cur_downsample_scale_ *= hand_size_modifier;
      
      // Find the rectangle in the highres image that will get scaled to the
      // final downsampled image
      int32_t srcw = std::min<int32_t>(HN_SRC_IM_SIZE,
        (int32_t)floor((float)HN_IM_SIZE * cur_downsample_scale_));
      int32_t srch = srcw;
      int32_t srcx = (HN_SRC_IM_SIZE - srcw) / 2;
      int32_t srcy = (HN_SRC_IM_SIZE - srch) / 2;
      // Note FracDownsampleImageSAT destroys the origional source image
#ifdef DOWNSAMPLE_POINT
      FracDownsampleImagePoint<float>(hand_image_cpu_, 0, 0, HN_IM_SIZE, HN_IM_SIZE,
        HN_IM_SIZE, cropped_hand_image_, srcx, srcy, srcw, srch, HN_SRC_IM_SIZE, 
        HN_SRC_IM_SIZE);
#else
      FracDownsampleImageSAT<float>(hand_image_cpu_, 0, 0, HN_IM_SIZE, HN_IM_SIZE,
        HN_IM_SIZE, cropped_hand_image_, srcx, srcy, srcw, srch, HN_SRC_IM_SIZE, 
        HN_SRC_IM_SIZE, im_temp_double_);
#endif

      // Save the lower left corner and the width / height
      hand_pos_wh_[0] = u_start + srcx;
      hand_pos_wh_[1] = v_start + srcy;
      hand_pos_wh_[2] = srcw;
      hand_pos_wh_[3] = srch;

    } else {
      hand_pos_wh_.set(0, 0, HN_SRC_IM_SIZE, HN_SRC_IM_SIZE);
      for (uint32_t i = 0; i < HN_IM_SIZE * HN_IM_SIZE; i++) {
        hand_image_cpu_[i] = 0;
      }
    }

    if (flip) {
      std::cout << "true" << std::endl;
      jtil::image_util::FlipImageVertInPlace<float>(hand_image_cpu_,
        HN_IM_SIZE, HN_IM_SIZE, 1);
      jtil::image_util::FlipImageHorzInPlace<float>(hand_image_cpu_,
        HN_IM_SIZE, HN_IM_SIZE, 1);
    }

    // Now downsample as many times as there are banks
    int32_t w = HN_IM_SIZE;
    int32_t h = HN_IM_SIZE;
    float* src = hand_image_cpu_;
    for (int32_t i = 1; i < num_banks_; i++) {
      DownsampleImage<float>(&src[w*h], src, w, h, 2);
      src = &src[w*h];
      w /= 2;
      h /= 2;
    }
  }

  void HandImageGenerator::calcHPFHandBanks() {
    // Firstly, upload the src data to jtorch
    int32_t w = HN_IM_SIZE;
    int32_t h = HN_IM_SIZE;
    float* src = hand_image_cpu_;
    for (int32_t i = 0; i < num_banks_; i++) {
      Tensor<float>* cur_image = (Tensor<float>*)(*hand_image_)(i);
      cur_image->setData(src);
      src = &src[w*h];
      w /= 2;
      h /= 2;
    }
    // Now perform local contrast normalization
    norm_module_->forwardProp(*hand_image_);
    if (norm_module_->output->type() != TABLE_DATA) {
      throw std::wruntime_error("HandImageGenerator::calcHPFHandBanks() - "
        "ERROR: norm_module output is of the wrong type!");
    }
    hpf_hand_image_ = (Table*)norm_module_->output;

    w = HN_IM_SIZE;
    h = HN_IM_SIZE;
    float* dst = hpf_hand_image_cpu_;
    for (int32_t i = 0; i < num_banks_; i++) {
      Tensor<float>* cur_image = (Tensor<float>*)(*hpf_hand_image_)(i);
      cur_image->getData(dst);

#ifndef HN_LOCAL_CONTRAST_NORM
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
      cur_image->setData(dst);  // Re-upload the results
#endif
      dst = &dst[w*h];
      w /= 2;
      h /= 2;
    }
  }

  void HandImageGenerator::annotateFeatsToKinectImage(uint8_t* im, 
    const float* coeff_convnet) const {
    for (uint32_t i = 0; i < HAND_NUM_COEFF_CONVNET; i += FEATURE_SIZE) {
        renderCrossToImageArr(&coeff_convnet[i], im, depth_w, 
          depth_h, 4, i, hand_pos_wh_);
    }
  }

  void HandImageGenerator::annotateFeatsToHandImage(uint8_t* im, 
    const float* coeff_convnet) const {
    jtil::math::Int4 hand_pos_wh(0, 0, HN_IM_SIZE, HN_IM_SIZE);
    for (uint32_t i = 0; i < HAND_NUM_COEFF_CONVNET; i += FEATURE_SIZE) {
        renderCrossToImageArr(&coeff_convnet[i], im, HN_IM_SIZE, 
          HN_IM_SIZE, 1, i/FEATURE_SIZE, hand_pos_wh);
    }
  }

  void HandImageGenerator::createHeatMap(float* hm, const uint32_t size, 
    const float* uv, const float std) {
    const float var = std * std;
    float ufeat = uv[0] * size;
    float vfeat = size - uv[1] * size - 1;
    for (uint32_t v = 0; v < size; v++) {
      for (uint32_t u = 0; u < size; u++) {
        float du = (float)u - ufeat;
        float dv = (float)v - vfeat;
        float f_u_v = expf(-((du * du) / (2 * var) + (dv * dv) / (2 * var)));
        hm[v * size + u] = f_u_v;
      }
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
      &jtil::renderer::colors[(color_ind) % jtil::renderer::n_colors];
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
    hand_mesh_vertices_.resize(depth_dim);
    for (uint32_t i = 0; i < depth_dim; i++) {
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
    for (uint32_t v = 0; v < (depth_h-1); v++) {
      for (uint32_t u = 0; u < (depth_w-1); u++) {
        p0 = v*depth_w + u;          // top left
        p1 = v*depth_w + (u+1);      // top right
        p2 = (v+1)*depth_w + u;      // bottom left
        p3 = (v+1)*depth_w + (u+1);  // bottom right
      
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
    hand_mesh_normals_.resize(depth_dim);
    
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

        hand_mesh_normals_[v1][0] += cur_normal[0];
        hand_mesh_normals_[v1][1] += cur_normal[1];
        hand_mesh_normals_[v1][2] += cur_normal[2];

        hand_mesh_normals_[v2][0] += cur_normal[0];
        hand_mesh_normals_[v2][1] += cur_normal[1];
        hand_mesh_normals_[v2][2] += cur_normal[2];

        hand_mesh_normals_[v3][0] += cur_normal[0];
        hand_mesh_normals_[v3][1] += cur_normal[1];
        hand_mesh_normals_[v3][2] += cur_normal[2];

        //Float3::add(hand_mesh_normals_[v1], hand_mesh_normals_[v1], cur_normal);
        //Float3::add(hand_mesh_normals_[v2], hand_mesh_normals_[v2], cur_normal);
        //Float3::add(hand_mesh_normals_[v3], hand_mesh_normals_[v3], cur_normal);
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
    for (uint32_t i = 0; i < depth_dim; i++) {
      normals_xyz[i*3] = hand_mesh_normals_[i][0];
      normals_xyz[i*3 + 1] = hand_mesh_normals_[i][1];
      normals_xyz[i*3 + 2] = hand_mesh_normals_[i][2];
    }
  }

}  // namespace hand_net
}  // namespace kinect_interface
