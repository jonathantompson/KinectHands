#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <cmath>
#include "hand_net/hand_net.h"
#include "hand_net/conv_stage.h"
#include "hand_net/nn_stage.h"
#include "data_str/vector.h"
#include "exceptions/wruntime_error.h"
#include "hand_model/hand_model.h"  // for HandCoeff
#include "hand_model/hand_model_renderer.h"
#include "hand_model/hand_model_geometry.h"
#include "hand_model/hand_model_geometry_mesh.h"
#include "hand_model/bounding_sphere.h"
#include "renderer/camera/camera.h"
#include "open_ni_funcs.h"
#include "image_util.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using math::Float4x4;
using math::FloatQuat;
using math::Float3;
using math::Float4;
using math::Float2;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using math::Float3;
using depth_images_io::DepthImagesIO;
using hand_model::HandCoeff;
using hand_model::HandModelRenderer;
using hand_model::HandModelGeometryMesh;
using hand_model::HandSphereIndices;
using renderer::BoundingSphere;

namespace hand_net {
 
  HandNet::HandNet() {
    n_conv_stages_ = 0;
    conv_stages_ = NULL;
    n_nn_stages_ = 0;
    nn_stages_ = NULL;
    nn_datcur_ = NULL;
    nn_datnext_ = NULL;
    conv_datcur_ = NULL;
    conv_datnext_ = NULL;
    hpf_hand_images_ = NULL;
    size_images_ = 0;
    hand_image_ = NULL;
    im_temp1_ = NULL;
    im_temp2_ = NULL;
    hpf_hand_images_coeff_ = NULL;

    // Figure out the size of the HPF bank array
    int32_t im_sizeu = HAND_NET_IM_SIZE;
    int32_t im_sizev = HAND_NET_IM_SIZE;
    size_images_ = 0;
    for (uint32_t i = 0; i < NUM_CONV_BANKS; i++) {
      size_images_ += (im_sizeu * im_sizev);
#if defined(DEBUG) || defined(_DEBUG)
      if (i < (NUM_CONV_BANKS - 1) && im_sizeu % 2 != 0 && im_sizev % 2 != 0) {
        throw std::wruntime_error("HandNet::loadFromFile() - ERROR: "
          "HPF bank image is not divisible by 2!");
      }
#endif
      im_sizeu /= 2;
      im_sizev /= 2;
    }

    // Some temporary data structures
    int32_t datasize = std::max<int32_t>(size_images_, 
      HAND_NET_PIX * HAND_NET_PIX);
    hand_image_ = new float [datasize];
    im_temp1_ = new float[datasize];
    im_temp2_ = new float[datasize];
    hpf_hand_images_ = new float[datasize];
    hpf_hand_images_coeff_ = new float[datasize];
    initHPFKernels();
  }

  HandNet::~HandNet() {
    if (conv_stages_) {
      for (int32_t i = 0; i < n_conv_stages_ * NUM_CONV_BANKS; i++) {
        SAFE_DELETE(conv_stages_[i]);
      }
    }
    SAFE_DELETE_ARR(conv_stages_);
    if (nn_stages_) {
      for (int32_t i = 0; i < n_nn_stages_; i++) {
        SAFE_DELETE(nn_stages_[i]);
      }
    }
    SAFE_DELETE_ARR(nn_stages_);
    SAFE_DELETE_ARR(nn_datcur_);
    SAFE_DELETE_ARR(nn_datnext_);
    if (conv_datcur_) {
      for (int32_t i = 0; i < NUM_CONV_BANKS; i++) {
        SAFE_DELETE_ARR(conv_datcur_[i]);
      }
    }
    SAFE_DELETE_ARR(conv_datcur_);
    if (conv_datnext_) {
      for (int32_t i = 0; i < NUM_CONV_BANKS; i++) {
        SAFE_DELETE_ARR(conv_datnext_[i]);
      }
    }
    SAFE_DELETE_ARR(conv_datnext_);
    SAFE_DELETE_ARR(hpf_hand_images_);
    SAFE_DELETE_ARR(hand_image_);
    SAFE_DELETE_ARR(im_temp1_);
    SAFE_DELETE_ARR(im_temp2_);
    SAFE_DELETE_ARR(hpf_hand_images_coeff_);
  }

  void HandNet::loadFromFile(const std::string& convnet_filename) {
    if (conv_stages_ != NULL) {
      throw std::wruntime_error("ConvStage::loadFromFile() - ERROR: "
        "Convnet data already exists. If you want to reload the convnet, then"
        " call the destructor and reload.");
    }

    std::cout << "loading HandNet from " << convnet_filename << std::endl;

    std::ifstream file(convnet_filename.c_str(), std::ios::in | std::ios::binary);
    if (file.is_open()) {

      file.seekg(0, std::ios::beg);

      // Get the meta data
      file.read(reinterpret_cast<char*>(&n_conv_stages_), 
        sizeof(n_conv_stages_));
      file.read(reinterpret_cast<char*>(&n_nn_stages_), sizeof(n_nn_stages_));
      int32_t n_hpf_banks;
      file.read(reinterpret_cast<char*>(&n_hpf_banks), sizeof(n_hpf_banks));
      if (n_hpf_banks != NUM_CONV_BANKS) {
        throw std::wruntime_error("HandNet::loadFromFile() - ERROR: "
          "num of hpf banks in convnet file does not equal NUM_CONV_BANKS!");
      }
      int32_t data_type;
      file.read(reinterpret_cast<char*>(&data_type), sizeof(data_type));
      data_type_ = (HandNetDataType)data_type;

      conv_stages_ = new ConvStage*[n_conv_stages_ * NUM_CONV_BANKS];
      for (uint32_t j = 0; j < NUM_CONV_BANKS; j++) {
        // Load in the convolution stages
        for (int32_t i = 0; i < n_conv_stages_; i++) {
          conv_stages_[j * n_conv_stages_ + i] = new ConvStage();
          conv_stages_[j * n_conv_stages_ + i]->loadFromFile(file);
        }
      }

      // Load in the neural network stages
      nn_stages_ = new NNStage*[n_nn_stages_];
      for (int32_t i = 0; i < n_nn_stages_; i++) {
        nn_stages_[i] = new NNStage();
        nn_stages_[i]->loadFromFile(file);
      }

      // clean up file io
      file.close();

      // Now create sufficient temporary data so that we can propogate the
      // forward model
      int32_t total_conv_output_size = 0;
      conv_datcur_ = new float*[NUM_CONV_BANKS];
      conv_datnext_ = new float*[NUM_CONV_BANKS];
      for (uint32_t j = 0; j < NUM_CONV_BANKS; j++) {
        ConvStage* cstage;
        int32_t im_sizeu = HAND_NET_IM_SIZE / (1 << j);
        int32_t im_sizev = HAND_NET_IM_SIZE / (1 << j);
        int32_t max_size = im_sizeu * im_sizev;
        for (int32_t i = 0; i < n_conv_stages_; i++) {
          cstage = conv_stages_[j * n_conv_stages_ + i];
          max_size = std::max<int32_t>(max_size, 
            cstage->dataSizeReq(im_sizeu, im_sizev));
          im_sizeu = cstage->calcOutWidth(im_sizeu);
          im_sizev = cstage->calcOutHeight(im_sizev);
        }
        total_conv_output_size += im_sizeu * im_sizev * 
          cstage->n_output_features();
        conv_datcur_[j] = new float[max_size];
        conv_datnext_[j] = new float[max_size];
      }
      

      // Quick check to make sure the sizes match up!
      if (total_conv_output_size != nn_stages_[0]->n_inputs()) {
          throw std::wruntime_error("HandNet::loadFromFile() - INTERNAL ERROR:"
            " convolution output size doesn't match neural net intput size");
      }

      uint32_t max_size = 0;
      for (int32_t i = 0; i < n_nn_stages_; i++) {
        max_size = std::max<uint32_t>(max_size, nn_stages_[i]->dataSizeReq());
        if (i < n_nn_stages_ - 1) {
          if (nn_stages_[i]->n_outputs() != nn_stages_[i+1]->n_inputs()) {
            throw std::wruntime_error("HandNet::loadFromFile() - INTERNAL "
              "ERROR: neural net out size doesn't match neural net in size");
          }
        }
      }

      // Finally, allocate size for the data that will flow through convnet
      nn_datcur_ = new float[max_size];
      nn_datnext_ = new float[max_size];

    } else {
      std::cout << "HandNet::loadFromFile() - ERROR: Could not open convnet";
      std::cout << " file " << convnet_filename << std::endl;
      std::cout << "Forward prop functionality will be disabled." << std::endl;
    }

    std::cout << "Finished initializing HandNet" << std::endl;
  }

  void HandNet::initHPFKernels() {
#if defined(DEBUG) || defined(_DEBUG)
    if (HPF_KERNEL_SIZE % 2 == 0) {
      throw std::runtime_error("HandNet::initHPFKernels() - ERROR: HPF "
        "kernel size must be odd!");
    }
#endif

    // Calculate the gaussian coeff
    int32_t center = (HPF_KERNEL_SIZE - 1)/2;
    for (int32_t i = 0; i < HPF_KERNEL_SIZE; i++) {
      gauss_filt_[i] = exp(-pow((float)(i - center) / HPF_SIGMA, 2) * 0.5f);
    }

    // Now at each resolution that we care about, filter an image of ones 
    // (where we crop to zero for pixels outside image) to create normalization
    // coefficients per pixel.
    int32_t w = HAND_NET_IM_SIZE;
    int32_t h = HAND_NET_IM_SIZE;
    // Create ones image
    for (int32_t j = 0; j < w * h; j++) {
      im_temp1_[j] = 1.0f;
    }
    float* coeff = hpf_hand_images_coeff_;
    for (uint32_t i = 0; i < NUM_CONV_BANKS; i++) {
      // Filter ones image
      ConvolveImageZeroCrop<float>(coeff, im_temp1_, im_temp2_, w, h, 
        gauss_filt_, HPF_KERNEL_SIZE);
      coeff = &coeff[w * h];
      w = w / 2;
      h = h / 2;
    }
  }

  void HandNet::calcHandCoeffConvnet(const int16_t* depth, 
    const uint8_t* label, float coeff_convnet[HAND_NUM_COEFF_CONVNET]) {
    if (n_conv_stages_ == 0) {
      std::cout << "HandNet::calcHandCoeff() - ERROR: Convnet not loaded";
      std::cout << " from file!" << std::endl;
    }
    HandNet::calcHandImage(depth, label);

    // Copy over the hand images in the input data structures

    float* im;
    switch (data_type_) {
    case DEPTH_DATA:
      im = hand_image_;
      break;
    case HPF_DEPTH_DATA:
      im = hpf_hand_images_;
      break;
    default:
      throw std::wruntime_error("HandNet::calcHandCoeffConvnet() - ERROR: "
        "data_type value is not supported!");
    }
    for (int32_t j = 0; j < NUM_CONV_BANKS; j++) {
      int32_t w = HAND_NET_IM_SIZE / (1 << j);
      int32_t h = HAND_NET_IM_SIZE / (1 << j);
      memcpy(conv_datcur_[j], im, w * h * sizeof(conv_datcur_[j][0]));
      im = &im[w*h];
    }

    // Now propogate the outputs through each of the banks and copy each
    // output into the nn input
    float* nn_input = nn_datcur_;
    ConvStage* stage = NULL;
    for (int32_t j = 0; j < NUM_CONV_BANKS; j++) {
      int32_t w = HAND_NET_IM_SIZE / (1 << j);
      int32_t h = HAND_NET_IM_SIZE / (1 << j);
      for (int32_t i = 0; i < n_conv_stages_; i++) {
        stage = conv_stages_[j * n_conv_stages_ + i];
        stage->forwardProp(conv_datcur_[j], w, h, conv_datnext_[j]);
        // Ping-pong the buffers
        float* tmp = conv_datnext_[j];
        conv_datnext_[j] = conv_datcur_[j];
        conv_datcur_[j] = tmp;
        // Calculate the next stage size
        w = stage->calcOutWidth(w);
        h = stage->calcOutHeight(h);
      }
      int32_t s = w * h * stage->n_output_features();
      memcpy(nn_input, conv_datcur_[j], s * sizeof(nn_input[0]));
      nn_input = &nn_input[s];
      // print3DTensorToStdCout<float>(conv_datcur_[j], 0, w, h, 2, 2, 6, 6);
    }

    // print3DTensorToStdCout<float>(nn_datcur_, 1, nn_stages_[0]->n_inputs(), 1);

    for (int32_t i = 0; i < n_nn_stages_; i++) {
      nn_stages_[i]->forwardProp(nn_datcur_, nn_datnext_);
      // Ping-pong the buffers
      float* tmp = nn_datnext_;
      nn_datnext_ = nn_datcur_;
      nn_datcur_ = tmp; 
      // print3DTensorToStdCout<float>(nn_datcur_, 1, nn_stages_[i]->n_outputs(), 1);
    }

    memcpy(coeff_convnet, nn_datcur_, HAND_NUM_COEFF_CONVNET * 
      sizeof(coeff_convnet[0]));

    // print3DTensorToStdCout<float>(nn_datcur_, 1, HAND_NUM_COEFF_CONVNET, 1);
  }

  void HandNet::createLabelFromSyntheticDepth(const float* depth, 
    uint8_t* label) {
    for (uint32_t i = 0; i < src_dim; i++) {
      label[i] = (depth[i] > EPSILON && depth[i] < GDT_MAX_DIST) ? 1 : 0;
    }
  }

  // Create the downsampled hand image, background is at 1 and hand is
  // in front of it.
  void HandNet::calcHandImage(const int16_t* depth_in, 
    const uint8_t* label_in) {
    calcCroppedHand(depth_in, label_in);
    calcHPFHandBanks();
  }

  void HandNet::calcCroppedHand(const int16_t* depth_in, 
    const uint8_t* label_in) {
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
    uvd_com_.scale(1.0f / (float)cnt);
    uvd_com_[0] = floor(uvd_com_[0]);
    uvd_com_[1] = floor(uvd_com_[1]);
    uint32_t u_start = (uint32_t)uvd_com_[0] - (HAND_NET_PIX / 2);
    uint32_t v_start = (uint32_t)uvd_com_[1] - (HAND_NET_PIX / 2);

    // Crop the image and scale between 0.0 and 1.0
    float dmin = uvd_com_[2] - (HAND_SIZE * 0.5f);
    float dmax = uvd_com_[2] + (HAND_SIZE * 0.5f);
    float background;
    if (HAND_NET_DOWN_FACT > 1) {
      background = std::numeric_limits<float>::infinity();
    } else {
      background = 1.0f;
    }
    for (uint32_t v = v_start; v < v_start + HAND_NET_PIX; v++) {
      for (uint32_t u = u_start; u < u_start + HAND_NET_PIX; u++) {
        uint32_t dst_index = (v-v_start)* HAND_NET_PIX + (u-u_start);
        uint32_t src_index = v * src_width + u;
        if (label_in[src_index] == 1) {
            hand_image_[dst_index] = ((float)depth_in[src_index] - dmin) / 
              HAND_SIZE;
        } else {
          hand_image_[dst_index] = background;
        }
      }
    }

    // Now downsample if needed:
    if (HAND_NET_DOWN_FACT > 1) {
      DownsampleImageWithoutBackground<float>(im_temp1_, hand_image_, 
        HAND_NET_PIX, HAND_NET_PIX, HAND_NET_DOWN_FACT, background);
      // Now ping-pong buffers
      float* tmp = im_temp1_;
      im_temp1_ = hand_image_;
      hand_image_ = tmp;
      // Now set the background back to 1
      for (uint32_t i = 0; i < HAND_NET_IM_SIZE*HAND_NET_IM_SIZE; i++) {
        if (hand_image_[i] == background) {
          hand_image_[i] = 1.0f;
        }
      }
    }

    // Now downsample twice --> We'll need the data for the HPF bank
    int32_t w = HAND_NET_IM_SIZE;
    int32_t h = HAND_NET_IM_SIZE;
    float* src = hand_image_;
    for (uint32_t i = 1; i < NUM_CONV_BANKS; i++) {
      DownsampleImage<float>(&src[w*h], src, w, h, 2);
      src = &src[w*h];
      w /= 2;
      h /= 2;
    }
  }

  void HandNet::calcHPFHandBanks() {
    int32_t w = HAND_NET_IM_SIZE;
    int32_t h = HAND_NET_IM_SIZE;
    float* coeff = hpf_hand_images_coeff_;
    float* dst = hpf_hand_images_;
    float* src = hand_image_;
    for (uint32_t i = 0; i < NUM_CONV_BANKS; i++) {
      // Apply LPF to the source image
      ConvolveImageZeroCrop<float>(dst, src, im_temp1_, w, h, 
        gauss_filt_, HPF_KERNEL_SIZE);
      // Normalize based on the pre-calculated normalization coefficients and
      // create the HPF image by subtracting the LPF image from the inputs src
      for (int32_t j = 0; j < w * h; j++) {
        dst[j] /= coeff[j];
        dst[j] = HPF_GAIN * (src[j] - dst[j]);  // HPF = src - LPF
      }
      // Downsample for the next iteration
      if (i < (NUM_CONV_BANKS - 1)) {
        // Iterate the dst, coeff and src pointers
        src = &src[w * h];
        coeff = &coeff[w * h];
        dst = &dst[w * h];
        // Update the new width and height
#if defined(DEBUG) || defined(_DEBUG)
        if (2*(w / 2) != w || 2*(h / 2) != h) {
          throw std::runtime_error("HandNet::calcHPFHandBanks(): ERROR - "
            "image size is not divisible by 2!");
        }
#endif
        w = w / 2;
        h = h / 2;
      }
    }
  }

  void HandNet::calcHandImageUVFromXYZ(HandModelRenderer* renderer, 
    Float3& xyz_pos, Float2& uv_pos) {
    Float4 pos(xyz_pos[0], xyz_pos[1], xyz_pos[2], 1.0f);
    Float4 homog_pos;
    Float4::mult(&homog_pos, renderer->camera()->proj(), &pos);
    uv_pos[0] = (homog_pos[0] / homog_pos[3]);  // NDC X: -1 --> 1
    uv_pos[1] = (homog_pos[1] / homog_pos[3]);  // NDC Y: -1 --> 1
    // http://www.songho.ca/opengl/gl_transform.html
    // TO DO: figure out why uv[0] needs to be flipped.  It makes no sense!
    uv_pos[0] = (float)src_width * 0.5f * (-uv_pos[0] + 1);  // Window X: 0 --> W
    uv_pos[1] = (float)src_height * 0.5f * (uv_pos[1] + 1);  // Window Y: 0 --> H
    // Now take off the uv COM and scale back to 0 --> 1
    uv_pos[0] = (uv_pos[0] - (uvd_com_[0] - (HAND_NET_PIX/2))) / HAND_NET_PIX;
    uv_pos[1] = (uv_pos[1] - (uvd_com_[1] - (HAND_NET_PIX/2))) / HAND_NET_PIX;
  }

  void HandNet::calcCoeffConvnet(hand_model::HandModel* hand, 
    HandModelRenderer* renderer, float coeff_convnet[HAND_NUM_COEFF_CONVNET]) {
    HandModelGeometryMesh* geom = 
      (HandModelGeometryMesh*)renderer->geom(hand->hand_type());
    const float* coeff = hand->coeff().data();
    Float2 pos_uv;

   // Thumb and finger angles are actually learned as salient points -->
    // Luckily we have a good way to get these.  Use the positions of some of
    // the key bounding sphere positions --> Then project these into UV.
    renderer->updateMatrices(hand->coeff(), hand->hand_type());
    renderer->updateHeirachyMatrices(hand->hand_type());
    renderer->fixBoundingSphereMatrices(hand->hand_type());

    // Project the XYZ position into UV space
    // Use the base of the thumb (which is constant in the hand's coord system)
    // since the model origin is usually off the 192x192 pixels.
    BoundingSphere* sphere = geom->bspheres()[HandSphereIndices::TH_KNU1_B];
    sphere->transform();
    calcHandImageUVFromXYZ(renderer, *sphere->transformed_center(), pos_uv);
    coeff_convnet[HAND_POS_U] = pos_uv[0];
    coeff_convnet[HAND_POS_V] = pos_uv[1];

    // Model origin as hand position
    Float3 hand_pos(coeff[HandCoeff::HAND_POS_X], 
      coeff[HandCoeff::HAND_POS_Y], coeff[HandCoeff::HAND_POS_Z]);
    //calcHandImageUVFromXYZ(renderer, hand_pos, pos_uv);
    //coeff_convnet[HAND_POS_U] = pos_uv[0];
    //coeff_convnet[HAND_POS_V] = pos_uv[1];

    // Convert quaternion to euler angles (might be easier to learn)
    FloatQuat quat(coeff[HandCoeff::HAND_ORIENT_X], 
                   coeff[HandCoeff::HAND_ORIENT_Y], 
                   coeff[HandCoeff::HAND_ORIENT_Z],
                   coeff[HandCoeff::HAND_ORIENT_W]);
    float euler[3];
    quat.quat2EulerAngles(euler[0], euler[1], euler[2]);

#if defined(DEBUG) || defined(_DEBUG)
    // At least make sure the inverse mapping and the conversion to matrix is
    // correct
    FloatQuat quat_tmp;
    FloatQuat::eulerAngles2Quat(&quat_tmp, euler[0], euler[1], euler[2]);
    if (!quat.approxEqual(&quat_tmp)) {
      throw std::runtime_error("ERROR: Quat --> Euler is not correct!");
    }
    Float4x4 mat;
    Float4x4 mat2;
    quat.quat2Mat4x4(&mat);
    Float4x4::euler2RotMat(&mat2, euler[0], euler[1], euler[2]);
    if (!mat.approxEqual(&mat2)) {
      throw std::runtime_error("ERROR: Quat --> Euler is not correct!");
    }
#endif

    // All angle coefficients are stored as (cos(x), sin(x)) to avoid
    // the singularity
    coeff_convnet[HAND_ORIENT_X_COS] = cosf(euler[0]);
    coeff_convnet[HAND_ORIENT_X_SIN] = sinf(euler[0]);
    coeff_convnet[HAND_ORIENT_Y_COS] = cosf(euler[1]);
    coeff_convnet[HAND_ORIENT_Y_SIN] = sinf(euler[1]);
    coeff_convnet[HAND_ORIENT_Z_COS] = cosf(euler[2]);
    coeff_convnet[HAND_ORIENT_Z_SIN] = sinf(euler[2]);

    coeff_convnet[WRIST_THETA_COS] = cosf(coeff[HandCoeff::WRIST_THETA]);
    coeff_convnet[WRIST_THETA_SIN] = sinf(coeff[HandCoeff::WRIST_THETA]);
    coeff_convnet[WRIST_PHI_COS] = cosf(coeff[HandCoeff::WRIST_PHI]);
    coeff_convnet[WRIST_PHI_SIN] = sinf(coeff[HandCoeff::WRIST_PHI]);

    // Thumb
    sphere = geom->bspheres()[HandSphereIndices::TH_KNU3_A];
    sphere->transform();
    calcHandImageUVFromXYZ(renderer, *sphere->transformed_center(), pos_uv);
    coeff_convnet[THUMB_TIP_U] = pos_uv[0];
    coeff_convnet[THUMB_TIP_V] = pos_uv[1];

    sphere = geom->bspheres()[HandSphereIndices::TH_KNU3_B];
    sphere->transform();
    calcHandImageUVFromXYZ(renderer, *sphere->transformed_center(), pos_uv);
    coeff_convnet[THUMB_K2_U] = pos_uv[0];
    coeff_convnet[THUMB_K2_V] = pos_uv[1];

    sphere = geom->bspheres()[HandSphereIndices::TH_KNU2_B];
    sphere->transform();
    calcHandImageUVFromXYZ(renderer, *sphere->transformed_center(), pos_uv);
    coeff_convnet[THUMB_K1_U] = pos_uv[0];
    coeff_convnet[THUMB_K1_V] = pos_uv[1];

    // Fingers
    for (uint32_t i = 0; i < 4; i++) {
      sphere = geom->bspheres()[HandSphereIndices::F1_KNU3_A + 6 * i];
      sphere->transform();
      calcHandImageUVFromXYZ(renderer, *sphere->transformed_center(), pos_uv);
      coeff_convnet[F0_TIP_U + 6 * i] = pos_uv[0];
      coeff_convnet[F0_TIP_V + 6 * i] = pos_uv[1];

      sphere = geom->bspheres()[HandSphereIndices::F1_KNU3_B + 6 * i];
      sphere->transform();
      calcHandImageUVFromXYZ(renderer, *sphere->transformed_center(), pos_uv);
      coeff_convnet[F0_K2_U + 6 * i] = pos_uv[0];
      coeff_convnet[F0_K2_V + 6 * i] = pos_uv[1];

      sphere = geom->bspheres()[HandSphereIndices::F1_KNU2_B + 6 * i];
      sphere->transform();
      calcHandImageUVFromXYZ(renderer, *sphere->transformed_center(), pos_uv);
      coeff_convnet[F0_K1_U + 6 * i] = pos_uv[0];
      coeff_convnet[F0_K1_V + 6 * i] = pos_uv[1];
    }
  }

}  // namespace hand_model
