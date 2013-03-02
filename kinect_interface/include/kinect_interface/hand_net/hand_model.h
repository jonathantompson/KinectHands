//
//  hand_model.h
//
//  Created by Jonathan Tompson on 3/1/13.
//
//  Slightly different to the version in HandFit --> Just a container really.
//

#ifndef KINECT_INTERFACE_HAND_FIT_HAND_MODEL_HEADER
#define KINECT_INTERFACE_HAND_FIT_HAND_MODEL_HEADER

#include "jtil/math/math_types.h"
#include "jtil/data_str/vector.h"
#include "kinect_interface/kinect_interface.h"  // for src_width/height

#define HAND_NUM_COEFF 26  // The number of coefficients to use when optimizing
#define FINGER_NUM_COEFF 3
#define HAND_MODEL_DEFAULT_SCALE 50.0f

namespace kinect_interface {
namespace hand_net {

  // NOTE: ALL ANGLES (STARTING AT WRIST_THETA) ARE DEFINED TO BE RESTING AT
  //       ROUGHLY 180 DEG (PI RAD).  This makes the penalty calculation easier.
  typedef enum {
    HAND_POS_X        = 0, 
    HAND_POS_Y        = 1,
    HAND_POS_Z        = 2,
    HAND_ORIENT_X     = 3,
    HAND_ORIENT_Y     = 4,
    HAND_ORIENT_Z     = 5,
    HAND_ORIENT_W     = 6,
    WRIST_THETA       = 7,
    WRIST_PHI         = 8,
    THUMB_THETA       = 9,
    THUMB_PHI         = 10,
    THUMB_K1_THETA    = 11,
    THUMB_K1_PHI      = 12,
    THUMB_K2_PHI      = 13,
    F0_THETA          = 14,
    F0_PHI            = 15,
    F0_KNUCKLE_CURL   = 16,
    F1_THETA          = 17,
    F1_PHI            = 18,
    F1_KNUCKLE_CURL   = 19,
    F2_THETA          = 20,
    F2_PHI            = 21,
    F2_KNUCKLE_CURL   = 22,
    F3_THETA          = 23,
    F3_PHI            = 24,
    F3_KNUCKLE_CURL   = 25,
    WRIST_LENGTH      = 26,  // Not used in optimization
    SCALE             = 27,  // Not used in optimization
    WRIST_TWIST       = 28,  // Not used in optimization
    NUM_PARAMETERS    = 29,  // NOTE: Not to be confused with HAND_NUM_COEFF!!
  } HandCoeff;

  typedef enum {
    WRIST             = 0, 
    PALM_FRONT        = 1,
    PALM_BACK         = 2,
    THUMB_1           = 3,
    THUMB_2           = 4,
    THUMB_3           = 5,
    FINGER_1_1        = 6,
    FINGER_1_2        = 7,
    FINGER_1_3        = 8,
    FINGER_2_1        = 9,
    FINGER_2_2        = 10,
    FINGER_2_3        = 11,
    FINGER_3_1        = 12,
    FINGER_3_2        = 13,
    FINGER_3_3        = 14,
    FINGER_4_1        = 15,
    FINGER_4_2        = 16,
    FINGER_4_3        = 17,
    BACKGROUND        = 18,
    NUM_HANDLABEL     = 19,
  } HandLabel;
  
  typedef enum {
    LEFT = 0,
    RIGHT = 1,
  } HandType;
  
  std::string HandCoeffToString(const uint32_t coeff);
  const uint8_t labelFromRGB(const float r, const float g, const float b);

  class HandModel {
  public:
    // Constructor / Destructor
    HandModel(const HandType hand_type);
    ~HandModel();

    // Accessors
    const float getCoeff(const uint32_t index) const;
    float* coeff() { return coeff_; }
    const HandType hand_type() const { return hand_type_; }
    void setRotation(const jtil::math::FloatQuat& quat);
    void getRotation(jtil::math::FloatQuat& quat) const;
    void printCoeff() const;
    float& local_scale() { return local_scale_; }
    void setCoeff(uint32_t index, float coeff_value);
    
    // FILE IO
    void saveToFile(const std::string& dir, const std::string& filename);
    void saveBlankFile(const std::string& dir, const std::string& filename);
    bool loadFromFile(const std::string& dir, const std::string& filename);

    static void renormalizeCoeffs(float* coeff);
    void resetPose();
    
    static float scale;
    static float wrist_length;
    static float wrist_twist;

  private:
    float coeff_[HAND_NUM_COEFF];  // The current state
    float local_scale_;
    HandType hand_type_;

    // Non-copyable, non-assignable.
    HandModel(HandModel&);
    HandModel& operator=(const HandModel&);
  };

};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_FIT_HAND_MODEL_HEADER