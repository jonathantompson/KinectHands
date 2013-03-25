//
//  hand_model.h
//
//  Created by Jonathan Tompson on 8/17/12.
//

#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <iomanip>
#include "kinect_interface/hand_net/hand_model.h"
#include "kinect_interface/hand_net/hand_net.h"  // HandCoeffConvnet
#include "kinect_interface/hand_net/hand_image_generator.h"
#include "jtil/jtil.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }

using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using namespace jtil::math;
using namespace jtil::renderer;

namespace kinect_interface {
namespace hand_net {

  float HandModel::scale = HAND_MODEL_DEFAULT_SCALE;
  float HandModel::wrist_length = 80.0f;
  float HandModel::wrist_twist = 0.0;

  jtil::renderer::GeometryInstance* HandModel::lhand = NULL;
  jtil::renderer::GeometryInstance* HandModel::rhand = NULL;
 
  HandModel::HandModel(HandType hand_type) {
    resetPose();
    hand_type_ = hand_type;
  }

  HandModel::~HandModel() {

  }

  void HandModel::resetPose() {
    coeff_[HAND_POS_X] = 0.0f;
    coeff_[HAND_POS_Y] = 0.0f;
    coeff_[HAND_POS_Z] = 750.0f;
    coeff_[HAND_ORIENT_X] = 0;
    coeff_[HAND_ORIENT_Y] = -1.5707963f;  // Pi / 2
    coeff_[HAND_ORIENT_Z] = 0;
    coeff_[WRIST_THETA] = 0;
    coeff_[WRIST_PHI] = 0;
    coeff_[THUMB_THETA] = 0;
    coeff_[THUMB_PHI] = 0;
    coeff_[THUMB_K1_THETA] = 0;
    coeff_[THUMB_K1_PHI] = 0;
    coeff_[THUMB_K2_PHI] = 0;
    coeff_[F0_THETA] = 0;
    coeff_[F0_PHI] = 0;
    coeff_[F0_KNUCKLE_CURL] = 0;
    coeff_[F1_THETA] = 0;
    coeff_[F1_PHI] = 0;
    coeff_[F1_KNUCKLE_CURL] = 0;
    coeff_[F2_THETA] = 0;
    coeff_[F2_PHI] = 0;
    coeff_[F2_KNUCKLE_CURL] = 0;
    coeff_[F3_THETA] = 0;
    coeff_[F3_PHI] = 0;
    coeff_[F3_KNUCKLE_CURL] = 0;
    local_scale_ = HAND_MODEL_DEFAULT_SCALE;
    renormalizeCoeffs(coeff_);  // Just in case
  }
    
  void HandModel::setCoeff(uint32_t index, float coeff_value) {
    if (index < HAND_NUM_COEFF) {
      coeff_[index] = coeff_value;
    } else {
      switch (index) {
      case HAND_NUM_COEFF:
        HandModel::wrist_length = coeff_value;
        break;
      case HAND_NUM_COEFF+1:
        HandModel::local_scale_ = coeff_value;
        HandModel::scale = coeff_value;
        break;
      }
    }
    renormalizeCoeffs(coeff_);
  }
  
  const float HandModel::getCoeff(const uint32_t index) const {
    if (index < HAND_NUM_COEFF) {
      return coeff_[index];
    } else {
      switch (index) {
      case WRIST_LENGTH:
        return HandModel::wrist_length;
      case SCALE:
        return HandModel::local_scale_;
      case WRIST_TWIST:
        return HandModel::wrist_twist;
      default:
        throw std::runtime_error("ERROR: coeff index is out of range");
      }
    }
  }

  void HandModel::setRotation(const Float3& euler) {
    coeff_[HandCoeff::HAND_ORIENT_X] = euler.m[0];
    coeff_[HandCoeff::HAND_ORIENT_Y] = euler.m[1];
    coeff_[HandCoeff::HAND_ORIENT_Z] = euler.m[2];
  }

  void HandModel::getRotation(Float3& euler) const {
    euler.m[0] = coeff_[HandCoeff::HAND_ORIENT_X];
    euler.m[1] = coeff_[HandCoeff::HAND_ORIENT_Y];
    euler.m[2] = coeff_[HandCoeff::HAND_ORIENT_Z];
  }

  // modulu - similar to matlab's mod()
  // result is always possitive. not similar to fmod()
  // Mod(-3,4)= 1   fmod(-3,4)= -3
#if defined(__APPLE__) || defined(_WIN32)
  float inline __fastcall Mod(float x, float y) {
    if (0 == y) {
      return x;
    }
    
    return x - y * floor(x / y);
  }
#else
  float inline Mod(float x, float y) {
    if (0 == y) {
      return x;
    }

    return x - y * floor(x / y);
  }
#endif

  // wrap [rad] angle to [0...2PI)
  inline void WrapTwo2PI(float& angle) {
    angle = Mod(angle, static_cast<float>(2.0 * M_PI));
  }

  // wrap [rad] angle to [-PI...PI)
  inline void WrapTwoPI(float& angle) {
    angle = Mod(angle + static_cast<float>(M_PI), 
      static_cast<float>(2.0 * M_PI)) - static_cast<float>(M_PI);
  }

  void HandModel::copyCoeffFrom(const HandModel* model) {
    copyCoeffFrom(model->coeff_);
  }

  void HandModel::copyCoeffFrom(const float* coeff) {
    memcpy(coeff_, coeff, sizeof(coeff_[0]) * HAND_NUM_COEFF);
  }

  FloatQuat tmp_quat_;
  void HandModel::renormalizeCoeffs(float* coeff) {
    // Set all angles 0 --> 2pi
    WrapTwoPI(coeff[HAND_ORIENT_X]);
    WrapTwoPI(coeff[HAND_ORIENT_Y]);
    WrapTwoPI(coeff[HAND_ORIENT_Z]);
    WrapTwoPI(coeff[WRIST_THETA]);
    WrapTwoPI(coeff[WRIST_PHI]);
    WrapTwoPI(coeff[THUMB_THETA]);
    WrapTwoPI(coeff[THUMB_PHI]);
    WrapTwoPI(coeff[THUMB_K1_THETA]);
    WrapTwoPI(coeff[THUMB_K1_PHI]);
    WrapTwoPI(coeff[THUMB_K2_PHI]);
    WrapTwoPI(coeff[F0_THETA]);
    WrapTwoPI(coeff[F0_PHI]);
    WrapTwoPI(coeff[F0_KNUCKLE_CURL]);
    WrapTwoPI(coeff[F1_THETA]);
    WrapTwoPI(coeff[F1_PHI]);
    WrapTwoPI(coeff[F1_KNUCKLE_CURL]);
    WrapTwoPI(coeff[F2_THETA]);
    WrapTwoPI(coeff[F2_PHI]);
    WrapTwoPI(coeff[F2_KNUCKLE_CURL]);
    WrapTwoPI(coeff[F3_THETA]);
    WrapTwoPI(coeff[F3_PHI]);
    WrapTwoPI(coeff[F3_KNUCKLE_CURL]);
  }
  
  string HandCoeffToString(const uint32_t coeff) {
    switch(coeff) {
      case HAND_POS_X:
        return "HAND_POS_X";
      case HAND_POS_Y:
        return "HAND_POS_Y";
      case HAND_POS_Z:
        return "HAND_POS_Z";
      case HAND_ORIENT_X:
        return "HAND_ORIENT_X";
      case HAND_ORIENT_Y:
        return "HAND_ORIENT_Y";
      case HAND_ORIENT_Z:
        return "HAND_ORIENT_Z";
      case WRIST_THETA:
        return "WRIST_THETA";
      case WRIST_PHI:
        return "WRIST_PHI";
      case THUMB_THETA:
        return "THUMB_THETA";
      case THUMB_PHI:
        return "THUMB_PHI";
      case THUMB_K1_THETA:
        return "THUMB_K1_THETA";
      case THUMB_K1_PHI:
        return "THUMB_K1_PHI";
      case THUMB_K2_PHI:
        return "THUMB_K2_PHI";
      case F0_THETA:
        return "F0_THETA";
      case F0_PHI:
        return "F0_PHI";
      case F0_KNUCKLE_CURL:
        return "F0_KNUCKLE_CURL";
      case F1_THETA:
        return "F1_THETA";
      case F1_PHI:
        return "F1_PHI";
      case F1_KNUCKLE_CURL:
        return "F1_KNUCKLE_CURL";
      case F2_THETA:
        return "F2_THETA";
      case F2_PHI:
        return "F2_PHI";
      case F2_KNUCKLE_CURL:
        return "F2_KNUCKLE_CURL";
      case F3_THETA:
        return "F3_THETA";
      case F3_PHI:
        return "F3_PHI";
      case F3_KNUCKLE_CURL:
        return "F3_KNUCKLE_CURL";
      case WRIST_LENGTH:
        return "WRIST_LENGTH";
      case SCALE:
        return "Scale";
      case WRIST_TWIST:
        return "WRIST_TWIST";
      case NUM_PARAMETERS:
        return "NUM_PARAMETERS";
    }
    return "undefined";
  };

  void HandModel::saveToFile(const std::string& dir, 
    const std::string& filename) const {
    string full_filename = dir + filename;
    std::ofstream file(full_filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + filename);
    }
    file.write(reinterpret_cast<const char*>(coeff_), 
      HAND_NUM_COEFF * sizeof(coeff_[0]));
    file.write(reinterpret_cast<const char*>(&wrist_length), 
      sizeof(wrist_length));
    file.write(reinterpret_cast<const char*>(&scale), sizeof(scale));
    file.flush();
    file.close();
  }

  void HandModel::saveBlankFile(const std::string& dir, 
    const std::string& filename) const {
    string full_filename = dir + filename;
    std::ofstream file(full_filename.c_str(), std::ios::out|std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + filename);
    }

    // Might be kinda slow to do this every time, but saving to file is pretty
    // rare, so OK for now.
    float blank[HAND_NUM_COEFF];
    for (uint32_t i = 0; i < HAND_NUM_COEFF; i++) {
      blank[i] = 0;
    }
    
    file.write(reinterpret_cast<const char*>(blank), 
      HAND_NUM_COEFF * sizeof(blank[0]));
    file.write(reinterpret_cast<const char*>(&wrist_length), 
      sizeof(wrist_length));
    file.write(reinterpret_cast<const char*>(&scale), sizeof(scale));
    file.flush();
    file.close();
  }

  bool HandModel::loadFromFile(const std::string& dir, 
    const std::string& filename) {
    string full_filename = dir + filename;
    std::ifstream file(full_filename.c_str(), std::ios::in | std::ios::binary);
    if (!file.is_open()) {
      return false;
    }
    file.seekg(0, std::ios::beg);
    // Make sure this isn't a blank file (indicating no hands on the screen)
    file.read(reinterpret_cast<char*>(coeff_), 
      HAND_NUM_COEFF * sizeof(coeff_[0]));
    if (coeff_[HAND_POS_X] < EPSILON && coeff_[HAND_POS_Y] < EPSILON && 
      coeff_[HAND_POS_Z] < EPSILON) {
      resetPose();
    }
    file.read(reinterpret_cast<char*>(&wrist_length), sizeof(wrist_length));
    file.read(reinterpret_cast<char*>(&local_scale_), sizeof(local_scale_));
    file.close();
    if (local_scale_ != HAND_MODEL_DEFAULT_SCALE) {
      scale = local_scale_;
    }
    return true;
  }

  std::mutex temp_data_lock;
  const Float4x4 old_rest_transform_wrist(-4.0992e-2f, 1.4916e-2f, -9.9905e-1f,
    0.0f, -9.9912e-1f, 8.3785e-3f, 4.1120e-2f, 0.0f, 8.9838e-3f, 9.9985e-1f,
    1.4559e-2f, 0.0f, 2.5561e0f, 6.2613e-3f,  -1.6060e-2f, 1.0f);
  const Float4x4 old_rest_transform_palm(9.9973e-1f, 1.9035e-2f, -1.3164e-2f,
    0.0f, -1.9968e-2f, 9.9699e-1f, -7.4869e-2f, 0.0f, 1.1699e-2f, 7.5112e-2f, 
    9.9711e-1f, 0.0f, -7.4506e-9f, 9.7399e-1f, 1.8626e-9f, 1.0f);
  Float4x4 root_mat;
  Float4x4 palm_mat;
  Float4x4 mat_tmp1;
  Float4x4 mat_tmp2;
  Float4x4 mat_tmp3;
  float tmp_coeff[HAND_NUM_COEFF+1];

  bool HandModel::loadOldFormatFromFile(const std::string& dir, 
    const std::string& filename) {
    temp_data_lock.lock();  // Just in case
    // 1. The palm was made the root (so the orientation needs to be multiplied
    // by the wrist matrix)
    // 2. An offset was applied
    // 3. The orientation changed from a quaternion to euler angles
    // This gets *"close"* to the correct pose, but not quite.
    string full_filename = dir + filename;
    std::ifstream file(full_filename.c_str(), std::ios::in | std::ios::binary);
    if (!file.is_open()) {
      temp_data_lock.unlock();
      return false;
    }
    file.seekg(0, std::ios::beg);
    // Make sure this isn't a blank file (indicating no hands on the screen)
    
    file.read(reinterpret_cast<char*>(tmp_coeff), 
      (HAND_NUM_COEFF+1) * sizeof(tmp_coeff[0]));
    if (tmp_coeff[HAND_POS_X] < EPSILON && tmp_coeff[HAND_POS_Y] < EPSILON && 
      tmp_coeff[HAND_POS_Z] < EPSILON) {
      resetPose();
    }
    // Copy over the position
    for (uint32_t i = 0; i <= HAND_POS_Z; i++) {
      coeff_[i] = tmp_coeff[i];
    }
    // Convert quaternion to euler
    FloatQuat cur_rot_quat(tmp_coeff[HAND_ORIENT_X], tmp_coeff[HAND_ORIENT_Y], 
      tmp_coeff[HAND_ORIENT_Z], tmp_coeff[HAND_ORIENT_Z+1]);
    FloatQuat::quat2Mat4x4(root_mat, cur_rot_quat);

    // Old code to set the palm matrix --> Now changed, but we need the old
    // subsequent 
    Float4x4::rotateMatXAxis(mat_tmp1, tmp_coeff[WRIST_PHI+1]);
    Float4x4::rotateMatZAxis(mat_tmp2, tmp_coeff[WRIST_THETA+1]);
    Float4x4::mult(mat_tmp3, mat_tmp1, mat_tmp2);
    Float4x4::rotateMatXAxis(mat_tmp1, HandModel::wrist_twist);
    Float4x4::mult(mat_tmp2, mat_tmp1, mat_tmp3);
    Float4x4::mult(palm_mat, old_rest_transform_palm, mat_tmp2);

    Float4x4::mult(mat_tmp1, root_mat, palm_mat);
    // The orientation component of mat_tmp1 is at least what we want.
    Float4x4::extractRotation(palm_mat, mat_tmp1);
    FloatQuat::orthMat4x42Quat(cur_rot_quat, palm_mat);
    // There is also an offset, which is defined in the root's coordinate frame
    // this is roughly the world offset we want at rest (71, 54.75, 127.25); 
    Float3 offset_model(-1.53962708f, -20.5192261f, -154.297485f);
    Float3 offset_world;
    Float3::affineTransformVec(offset_world, root_mat, offset_model);
    coeff_[HAND_POS_X] += offset_world[0];
    coeff_[HAND_POS_Y] += offset_world[1];
    coeff_[HAND_POS_Z] += offset_world[2];

    FloatQuat::quat2EulerAngles(coeff_[HAND_ORIENT_X], coeff_[HAND_ORIENT_Y], 
      coeff_[HAND_ORIENT_Z], cur_rot_quat);
    // All other coefficients are shifted down
    for (uint32_t i = WRIST_THETA; i < HAND_NUM_COEFF; i++) {
      coeff_[i] = tmp_coeff[i+1];
    }
    file.read(reinterpret_cast<char*>(&wrist_length), sizeof(wrist_length));
    file.read(reinterpret_cast<char*>(&local_scale_), sizeof(local_scale_));

    // The wrist now hinges off the palm, so we need the negative angles
    //coeff_[WRIST_THETA] *= -1.0f;
    //coeff_[WRIST_PHI] *= -1.0f;

    file.close();
    if (local_scale_ != HAND_MODEL_DEFAULT_SCALE) {
      scale = local_scale_;
    }
    temp_data_lock.unlock();
    return true;
  }

  const uint8_t labelFromRGB(const float r, const float g, const float b) {
    uint32_t red_type;
    if (r < 0.25f) {
      red_type = 0;
    } else if (r < 0.75f) {
      red_type = 1;
    } else {
      red_type = 2;
    }

    uint32_t green_type;
    if (g < 0.25f) {
      green_type = 0;
    } else if (g < 0.75f) {
      green_type = 1;
    } else {
      green_type = 2;
    }

    uint32_t blue_type;
    if (b < 0.25f) {
      blue_type = 0;
    } else if (b < 0.75f) {
      blue_type = 1;
    } else {
      blue_type = 2;
    }

    int type = (red_type << 4) | (green_type << 2) | blue_type;
    switch (type) {
    case 0:   // bit(000000) = rgb(0.0, 0.0, 0.0)
      return HandLabel::BACKGROUND;
    case 1:   // bit(000001) = rgb(0.0, 0.0, 0.5)  // Dark-blue
      return HandLabel::THUMB_3;
    case 2:   // bit(000010) = rgb(0.0, 0.0, 1.0)  // Blue
      return HandLabel::THUMB_1;

    case 4:   // bit(000100) = rgb(0.0, 0.5, 0.0)  // Dark-Green
      return HandLabel::FINGER_3_2;
    case 5:   // bit(000101) = rgb(0.0, 0.5, 0.5)  // Cyan
      return HandLabel::FINGER_4_2;
    // case 6:   // bit(000110) = rgb(0.0, 0.5, 1.0)

    case 8:   // bit(001000) = rgb(0.0, 1.0, 0.0)  // Green
      return HandLabel::PALM_FRONT;
    // case 9:   // bit(001001) = rgb(0.0, 1.0, 0.5)
    case 10:  // bit(001010) = rgb(0.0, 1.0, 1.0)  // Aqua
      return HandLabel::FINGER_2_1;

    case 16:  // bit(010000) = rgb(0.5, 0.0, 0.0)  // Dark-Red
      return HandLabel::FINGER_3_1;
    case 17:  // bit(010001) = rgb(0.5, 0.0, 0.5)  // Purple
      return HandLabel::FINGER_2_3;
    // case 18:  // bit(010010) = rgb(0.5, 0.0, 1.0)

    case 20:  // bit(010100) = rgb(0.5, 0.5, 0.0)  // Dark-Yellow
      return HandLabel::FINGER_3_3;
    // case 21:  // bit(010101) = rgb(0.5, 0.5, 0.5)
    case 22:  // bit(010110) = rgb(0.5, 0.5, 1.0)  // Light-Blue
      return HandLabel::THUMB_2;

    case 24:  // bit(011000) = rgb(0.5, 1.0, 0.0)  // Lime
      return HandLabel::FINGER_4_1;
    case 25:  // bit(011001) = rgb(0.5, 1.0, 0.5)  // Light-Green
      return HandLabel::FINGER_1_1;
    // case 26:  // bit(011010) = rgb(0.5, 1.0, 1.0)

    case 32:  // bit(100000) = rgb(1.0, 0.0, 0.0)  // Red
      return HandLabel::WRIST;
    case 33:  // bit(100001) = rgb(1.0, 0.0, 0.5)  // Magenta
      return HandLabel::FINGER_4_3;
    case 34:  // bit(100010) = rgb(1.0, 0.0, 1.0)  // Pink
      return HandLabel::FINGER_1_2;

    case 36:  // bit(100100) = rgb(1.0, 0.5, 0.0)  // Gold
      return HandLabel::PALM_BACK;
    case 37:  // bit(100101) = rgb(1.0, 0.5, 0.5)  // Light-Red
      return HandLabel::FINGER_2_2;
    // case 38:  // bit(100110) = rgb(1.0, 0.5, 1.0)

    case 40:  // bit(101000) = rgb(1.0, 1.0, 0.0)  // Yellow
      return HandLabel::FINGER_1_3;
    // case 41:  // bit(101001) = rgb(1.0, 1.0, 0.5)
    // case 42:  // bit(101010) = rgb(1.0, 1.0, 1.0)
    default:
      std::cout << "ERROR: Couldn't classify pixel from RGB!" << std::endl;
      std::cout << "r = " << r << std::endl;
      std::cout << "g = " << g << std::endl;
      std::cout << "b = " << b << std::endl;
      throw std::runtime_error("ERROR: Couldn't classify pixel from RGB!");
    }
  }

  void HandModel::printCoeff() const {
    std::cout << "coeff = [";
    std::cout << std::setprecision(6);
    std::cout << std::fixed;
    for (uint32_t i = 0; i < HAND_NUM_COEFF; i++) {
      std::cout << coeff_[i];
      if (i < HAND_NUM_COEFF - 1) {
        std::cout << ", ";
      }
    }
    std::cout << "]" << std::endl;
  }

  void HandModel::setHandModelVisibility(const bool visibility) {
    if (lhand) {
      lhand->setRenderHierarchy(visibility);
    }
    if (rhand) {
      rhand->setRenderHierarchy(visibility);
    }
  }

  void HandModel::setHandModelPose(const HandType hand, 
    const HandImageGenerator* im_gen, const float* convnet_coeff) {
    if (hand == LEFT) {
#if defined(DEBUG) || defined(_DEBUG)
      if (!lhand) {
        throw std::wruntime_error("HandModel::setHandModelPose() - ERROR: "
          "lhand not yet initialized!");
      }
#endif
      setHandModelPose(lhand, im_gen, convnet_coeff);
    }
    if (hand == RIGHT) {
#if defined(DEBUG) || defined(_DEBUG)
      if (!rhand) {
        throw std::wruntime_error("HandModel::setHandModelPose() - ERROR: "
          "rhand not yet initialized!");
      }
#endif
      setHandModelPose(rhand, im_gen, convnet_coeff);
    }
  }

  void HandModel::setHandModelPose(jtil::renderer::GeometryInstance* hand, 
    const HandImageGenerator* im_gen, const float* convnet_coeff) {
    const Float3& uvd_com = im_gen->uvd_com();
    const Int4& hand_pos_wh = im_gen->hand_pos_wh();

    Float3 pos_uvd(convnet_coeff[HAND_POS_U], convnet_coeff[HAND_POS_V],
      convnet_coeff[HAND_POS_D]);
    // U and V are between 0 and 1 in the hand image --> Convert back to pixels

  }

  void HandModel::loadHandModels(const bool left, const bool right) {
#ifndef LOAD_JBIN_FILES
    if (left) {
      lhand = Renderer::g_renderer()->gm()->loadModelFromFile(
        "./models/lib_hand/", "hand_palm_parent_medium_wrist.dae");
      Renderer::g_renderer()->gm()->saveModelToJBinFile("./models/lib_hand/", 
        "hand_palm_parent_medium_wrist.jbin", lhand);
    }
    if (right) {
      rhand = Renderer::g_renderer()->gm()->loadModelFromFile(
        "./models/lib_hand/", "hand_palm_parent_medium_wrist_right.dae");
      Renderer::g_renderer()->gm()->saveModelToJBinFile("./models/lib_hand/", 
        "hand_palm_parent_medium_wrist_right.jbin", rhand);
    }
#else
    if (left) {
      lhand = Renderer::g_renderer()->gm()->loadModelFromJBinFile(
        "./models/lib_hand/", "hand_palm_parent_medium_wrist.jbin");
    }
    if (right) {
      rhand = Renderer::g_renderer()->gm()->loadModelFromJBinFile(
        "./models/lib_hand/", "hand_palm_parent_medium_wrist_right.jbin");
    }
#endif
  }

  const float HandModel::sph_off_[NUM_BOUNDING_SPHERES*3] = { 
    -0.1355f, -0.00849999f, -0.2875f,  // F1_KNU3_A = 0,
    0.002f, 0.007f, -0.1205f,  // F1_KNU3_B = 1,
    -0.13f, 0.0305f, -0.1975f,  // F1_KNU2_A = 2,
    0.0295f, 0.00149996f, -0.0615f,  // F1_KNU2_B = 2,
    -0.3195f, 0.0315f, -0.211f,  // F1_KNU1_A = 3,
    0.0115f, -0.0235f, -0.1275f,  // F1_KNU1_B = 3,
    -0.2615f, -0.1135f, -0.3965f,  // F2_KNU3_A = 4,
    -0.126f, -0.0245f, -0.131f,  // F2_KNU3_B = 5,
    -0.144f, -0.00450001f, -0.0855f,  // F2_KNU2_A = 6,
    0.0705f, 0.00400001f, 0.1095f,  // F2_KNU2_B = 6,
    -0.3505f, -0.0275f, -0.281f,  // F2_KNU1_A = 7,
    -0.002f, -0.0635f, -0.1945f,  // F2_KNU1_B = 7,
    -0.157f, -0.0285f, -0.279f,  // F3_KNU3_A = 8,
    0.068f, 0.061f, 0.0865f,  // F3_KNU3_B = 9,
    -0.1665f, 0.022f, -0.205f,  // F3_KNU2_A = 10,
    0.068f, 0.0545f, 0.008f,  // F3_KNU2_B = 10,
    -0.419f, 0.0565f, -0.044f,  // F3_KNU1_A = 11,
    -0.0095f, 0.0005f, 0.0085f,  // F3_KNU1_B = 11,
    -0.343f, 0.012f, -0.3445f,  // F4_KNU3_A = 12,
    -0.073f, 0.035f, -0.105f,  // F4_KNU3_B = 13,
    -0.2485f, 0.008f, -0.172f,  // F4_KNU2_A = 14,
    0.0f, 0.0335f, -0.0125f,  // F4_KNU2_B = 14,
    -0.5595f, -0.035f, -0.0315f,  // F4_KNU1_A = 15,
    -0.0325f, -0.0405f, 0.0f,  // F4_KNU1_B = 15,
    -0.432f, 0.0775f, -0.104f,  // TH_KNU3_A = 16,
    0.01f, 0.0950001f, -0.038f,  // TH_KNU3_B = 17,
    -0.341f, 0.017f, 0.0175f,  // TH_KNU2_A = 18,
    -0.0335f, 0.0585f, 0.044f,  // TH_KNU2_B = 18,
    -0.4485f, -0.343f, -0.115f,  // TH_KNU1_A = 19,
    0.0f, 0.0f, 0.0f,  // TH_KNU1_B = 19,
  };

  const float HandModel::sph_size_[NUM_BOUNDING_SPHERES] = {
    0.095f,  // F1_KNU3_A = 0,
    0.12f,   // F1_KNU3_B = 1,
    0.14f,   // F1_KNU2_A = 2,
    0.16f,   // F1_KNU2_B = 2,
    0.17f,   // F1_KNU1_A = 3,
    0.20f,   // F1_KNU1_B = 3,
    0.115f,   // F2_KNU3_A = 4,
    0.14f,   // F2_KNU3_B = 5,
    0.17f,   // F2_KNU2_A = 6,
    0.18f,   // F2_KNU2_B = 6,
    0.19f,   // F2_KNU1_A = 7,
    0.20f,   // F2_KNU1_B = 7,
    0.115f,   // F3_KNU3_A = 8,
    0.17f,   // F3_KNU3_B = 9,
    0.18f,   // F3_KNU2_A = 10,
    0.20f,   // F3_KNU2_B = 10,
    0.20f,   // F3_KNU1_A = 11,
    0.21f,   // F3_KNU1_B = 11,
    0.105f,  // F4_KNU3_A = 12,
    0.16f,  // F4_KNU3_B = 13,
    0.17f,  // F4_KNU2_A = 14,
    0.18f,  // F4_KNU2_B = 14,
    0.20f,  // F4_KNU1_A = 15,
    0.21f,  // F4_KNU1_B = 15,
    0.17f,  // TH_KNU3_A = 16,
    0.19f,  // TH_KNU3_B = 17,
    0.20f,  // TH_KNU2_A = 18,
    0.25f,  // TH_KNU2_B = 18,
    0.28f,  // TH_KNU1_A = 19,
    0.28f,  // TH_KNU1_B = 19,
  };

}  // namespace hand_net
}  // namespace kinect_interface