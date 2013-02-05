//
//  hand_model.h
//
//  Created by Jonathan Tompson on 8/17/12.
//

#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include "hand_model/hand_model.h"
#include "hand_model/hand_model_renderer.h"
#include "math/lm_fitting.h"
#include "math/nm_fitting.h"
#include "math/de_fitting.h"
#include "math/pso_fitting.h"
#include "math/lprpso_fitting.h"
#include "file_io/file_io.h"
#include "Eigen"

#define SAFE_DELETE(target) \
  if (target != NULL) { \
    delete target; \
    target = NULL; \
  }

using Eigen::Matrix;
using Eigen::MatrixXf;
using math::Float4x4;
using math::FloatQuat;
using math::Float3;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using math::Float3;

namespace hand_model {
#ifdef USE_SIMPLE_GEOMETRY
  float HandModel::scale = 1.0;
#else
  float HandModel::scale = HAND_MODEL_DEFAULT_SCALE;
#endif
  float HandModel::wrist_length = 80.0f;
  float HandModel::wrist_twist = 0.0;
 
  HandModel::HandModel(HandType hand_type) {
    coeff_.resize(1, HAND_NUM_COEFF);
    resetPose();
    hand_type_ = hand_type;
  }

  HandModel::~HandModel() {

  }

  void HandModel::resetPose() {
    coeff_(HAND_POS_X) = 0.0;
    coeff_(HAND_POS_Y) = 0.0;
    coeff_(HAND_POS_Z) = 750.0;
    FloatQuat starting_quat;
    starting_quat.identity();
    coeff_(HAND_ORIENT_X) = -0.0511944f;
    coeff_(HAND_ORIENT_Y) = -0.0942133f;
    coeff_(HAND_ORIENT_Z) = -0.6893440f;
    coeff_(HAND_ORIENT_W) = +0.7164550f;
    //coeff_(HAND_ORIENT_X) = starting_quat[0];
    //coeff_(HAND_ORIENT_Y) = starting_quat[1];
    //coeff_(HAND_ORIENT_Z) = starting_quat[2];
    //coeff_(HAND_ORIENT_W) = starting_quat[3];
    coeff_(WRIST_THETA) = 0;
    coeff_(WRIST_PHI) = 0;
    coeff_(THUMB_THETA) = 0;
    coeff_(THUMB_PHI) = 0;
    coeff_(THUMB_K1_THETA) = 0;
    coeff_(THUMB_K1_PHI) = 0;
    coeff_(THUMB_K2_PHI) = 0;
    coeff_(F0_THETA) = 0;
    coeff_(F0_PHI) = 0;
    coeff_(F0_KNUCKLE_CURL) = 0;
    coeff_(F1_THETA) = 0;
    coeff_(F1_PHI) = 0;
    coeff_(F1_KNUCKLE_CURL) = 0;
    coeff_(F2_THETA) = 0;
    coeff_(F2_PHI) = 0;
    coeff_(F2_KNUCKLE_CURL) = 0;
    coeff_(F3_THETA) = 0;
    coeff_(F3_PHI) = 0;
    coeff_(F3_KNUCKLE_CURL) = 0;
    renormalizeCoeffs(coeff_);
  }
    
  void HandModel::setCoeff(uint32_t index, float coeff_value) {
    if (index < HAND_NUM_COEFF) {
      coeff_(index) = coeff_value;
    } else {
      switch (index) {
      case HAND_NUM_COEFF:
        HandModel::wrist_length = coeff_value;
        break;
      case HAND_NUM_COEFF+1:
        HandModel::scale = coeff_value;
        break;
      }
    }
    renormalizeCoeffs(coeff_);
  }
  
  float HandModel::getCoeff(uint32_t index) {
    if (index < HAND_NUM_COEFF) {
      return coeff_(index);
    } else {
      switch (index) {
      case WRIST_LENGTH:
        return HandModel::wrist_length;
      case SCALE:
        return HandModel::scale;
      case WRIST_TWIST:
        return HandModel::wrist_twist;
      default:
        throw std::runtime_error("ERROR: coeff index is out of range");
      }
    }
  }

  void HandModel::setRotation(math::FloatQuat* quat) {
    coeff_(HandCoeff::HAND_ORIENT_X) = quat->m[0];
    coeff_(HandCoeff::HAND_ORIENT_Y) = quat->m[1];
    coeff_(HandCoeff::HAND_ORIENT_Z) = quat->m[2];
    coeff_(HandCoeff::HAND_ORIENT_W) = quat->m[3];
  }

  void HandModel::getRotation(math::FloatQuat* quat) {
    quat->m[0] = coeff_(HandCoeff::HAND_ORIENT_X);
    quat->m[1] = coeff_(HandCoeff::HAND_ORIENT_Y);
    quat->m[2] = coeff_(HandCoeff::HAND_ORIENT_Z);
    quat->m[3] = coeff_(HandCoeff::HAND_ORIENT_W);
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
    angle = Mod(angle, static_cast<float>(M_2PI));
  }

  // wrap [rad] angle to [-PI...PI)
  inline void WrapTwoPI(float& angle) {
    angle = Mod(angle + static_cast<float>(M_PI), 
      static_cast<float>(M_2PI)) - static_cast<float>(M_PI);
  }


  FloatQuat tmp_quat_;
  void HandModel::renormalizeCoeffs(Eigen::MatrixXf& coeff) {
    // Normalize the quaternion
    tmp_quat_.set(static_cast<float>(coeff(HAND_ORIENT_X)),
                  static_cast<float>(coeff(HAND_ORIENT_Y)),
                  static_cast<float>(coeff(HAND_ORIENT_Z)),
                  static_cast<float>(coeff(HAND_ORIENT_W)));
    tmp_quat_.normalize();
    coeff(HAND_ORIENT_X) = tmp_quat_[0];
    coeff(HAND_ORIENT_Y) = tmp_quat_[1];
    coeff(HAND_ORIENT_Z) = tmp_quat_[2];
    coeff(HAND_ORIENT_W) = tmp_quat_[3];
    
    // Set all angles 0 --> 2pi
    WrapTwoPI(coeff(WRIST_THETA));
    WrapTwoPI(coeff(WRIST_PHI));
    WrapTwoPI(coeff(THUMB_THETA));
    WrapTwoPI(coeff(THUMB_PHI));
    WrapTwoPI(coeff(THUMB_K1_THETA));
    WrapTwoPI(coeff(THUMB_K1_PHI));
    WrapTwoPI(coeff(THUMB_K2_PHI));
    WrapTwoPI(coeff(F0_THETA));
    WrapTwoPI(coeff(F0_PHI));
    WrapTwoPI(coeff(F0_KNUCKLE_CURL));
    WrapTwoPI(coeff(F1_THETA));
    WrapTwoPI(coeff(F1_PHI));
    WrapTwoPI(coeff(F1_KNUCKLE_CURL));
    WrapTwoPI(coeff(F2_THETA));
    WrapTwoPI(coeff(F2_PHI));
    WrapTwoPI(coeff(F2_KNUCKLE_CURL));
    WrapTwoPI(coeff(F3_THETA));
    WrapTwoPI(coeff(F3_PHI));
    WrapTwoPI(coeff(F3_KNUCKLE_CURL));
  }
  
  string HandCoeffToString(uint32_t coeff) {
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
      case HAND_ORIENT_W:
        return "HAND_ORIENT_W";
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

  void HandModel::saveToFile(std::string directory, std::string filename) {
    string full_filename = directory + filename;
    std::ofstream file(full_filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + filename);
    }
    float* data = coeff_.data();
    file.write(reinterpret_cast<const char*>(data), HAND_NUM_COEFF * sizeof(data[0]));
    file.write(reinterpret_cast<const char*>(&wrist_length), sizeof(wrist_length));
    file.write(reinterpret_cast<const char*>(&scale), sizeof(scale));
    file.flush();
    file.close();
  }

  void HandModel::saveBlankFile(std::string directory, std::string filename) {
    string full_filename = directory + filename;
    std::ofstream file(full_filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + filename);
    }
    Float3 origin(0, 0, 0);
    file.write(reinterpret_cast<const char*>(origin.m), HAND_NUM_COEFF * sizeof(origin.m[0]));
    file.flush();
    file.close();
  }

  bool HandModel::loadFromFile(std::string directory, std::string filename) {
    string full_filename = directory + filename;
    std::ifstream file(full_filename.c_str(), std::ios::in | std::ios::binary);
    if (!file.is_open()) {
      return false;
    }
    float* data = coeff_.data();
    file.seekg(0, std::ios::beg);
    // Make sure this isn't a blank file (indicating no hands on the screen)
    file.read(reinterpret_cast<char*>(data), 3 * sizeof(data[0]));
    if (data[0] < EPSILON && data[1] < EPSILON && data[2] < EPSILON) {
      resetPose();
    } else {
      float cur_scale;
      file.read(reinterpret_cast<char*>(&data[3]), (HAND_NUM_COEFF-3) * sizeof(data[0]));
      file.read(reinterpret_cast<char*>(&wrist_length), sizeof(wrist_length));
      file.read(reinterpret_cast<char*>(&cur_scale), sizeof(cur_scale));
      if (cur_scale != HAND_MODEL_DEFAULT_SCALE) {
        scale = cur_scale;
      }
      file.close();
    }
    return true;
  }

  uint8_t labelFromRGB(float r, float g, float b) { 
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
    for (uint32_t i = 0; i < HAND_NUM_COEFF; i++) {
      std::cout << coeff_(i);
      if (i < HAND_NUM_COEFF - 1) {
        std::cout << ", ";
      }
    }
    std::cout << "]" << std::endl;
  }

}  // namespace hand_model
