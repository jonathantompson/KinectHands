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

#define FIT_TWIST
#ifdef FIT_TWIST
  #define HAND_NUM_COEFF 30  // The num of coefficients to use when optimizing
#else
  #define HAND_NUM_COEFF 25
#endif
#define FINGER_NUM_COEFF 3
#define NSPH_PER_FING 6
#define HAND_MODEL_DEFAULT_SCALE 58.0f
#define HAND_CAMERA_VIEW_PLANE_NEAR 10.0f
#define HAND_CAMERA_VIEW_PLANE_FAR 3000.0f
// http://www.ros.org/wiki/kinect_calibration/technical
#define HAND_CAMERA_FOV_HOR 57.8f  // Kinect focal length is 585pix (640pix width)
                                   // RGB FOV is 62.7
// #define HAND_CAMERA_FOV 43.35f  // Actual value
#define HAND_CAMERA_FOV 45.25f  // This value works better
#define LOAD_JBIN_FILES

namespace jtil {
namespace renderer {
  class GeometryInstance;
};
};

namespace kinect_interface {
namespace hand_net {

  class HandImageGenerator;

  // NOTE: ALL ANGLES (STARTING AT WRIST_THETA) ARE DEFINED TO BE RESTING AT
  //       ROUGHLY 180 DEG (PI RAD).  This makes the penalty calculation easier.
  typedef enum {
    HAND_POS_X        = 0, 
    HAND_POS_Y        = 1,
    HAND_POS_Z        = 2,
    HAND_ORIENT_X     = 3,
    HAND_ORIENT_Y     = 4,
    HAND_ORIENT_Z     = 5,
    WRIST_THETA       = 6,
    WRIST_PHI         = 7,
    THUMB_THETA       = 8,
    THUMB_PHI         = 9,
    THUMB_K1_THETA    = 10,
    THUMB_K1_PHI      = 11,
    THUMB_K2_PHI      = 12,
    F0_THETA          = 13,
    F0_PHI            = 14,
    F0_KNUCKLE_CURL   = 15,
    F1_THETA          = 16,
    F1_PHI            = 17,
    F1_KNUCKLE_CURL   = 18,
    F2_THETA          = 19,
    F2_PHI            = 20,
    F2_KNUCKLE_CURL   = 21,
    F3_THETA          = 22,
    F3_PHI            = 23,
    F3_KNUCKLE_CURL   = 24,
    F0_TWIST          = 25,  // Not used in optimization
    F1_TWIST          = 26,  // Not used in optimization
    F2_TWIST          = 27,  // Not used in optimization
    F3_TWIST          = 28,  // Not used in optimization
    THUMB_TWIST       = 29,  // Not used in optimization
    F0_LENGTH         = 30,  // Not used in optimization
    F1_LENGTH         = 31,  // Not used in optimization
    F2_LENGTH         = 32,  // Not used in optimization
    F3_LENGTH         = 33,  // Not used in optimization
    THUMB_LENGTH      = 34,  // Not used in optimization
    SCALE             = 35,  // Not used in optimization
    NUM_PARAMETERS    = 36,  // NOTE: Not to be confused with HAND_NUM_COEFF!!
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
    F1_KNU3_A = 0,  // tip
    F1_KNU3_B = 1,
    F1_KNU2_A = 2,
    F1_KNU2_B = 3,
    F1_KNU1_A = 4,
    F1_KNU1_B = 5,  // base
    F2_KNU3_A = 6,  // tip
    F2_KNU3_B = 7,
    F2_KNU2_A = 8,
    F2_KNU2_B = 9,
    F2_KNU1_A = 10,
    F2_KNU1_B = 11,  // base
    F3_KNU3_A = 12,  // tip
    F3_KNU3_B = 13,
    F3_KNU2_A = 14,
    F3_KNU2_B = 15,
    F3_KNU1_A = 16,
    F3_KNU1_B = 17,  // base
    F4_KNU3_A = 18,  // tip
    F4_KNU3_B = 19,
    F4_KNU2_A = 20,
    F4_KNU2_B = 21,
    F4_KNU1_A = 22,
    F4_KNU1_B = 23,  // base
    TH_KNU3_A = 24,  // Tip
    TH_KNU3_B = 25,
    TH_KNU2_A = 26,
    TH_KNU2_B = 27,
    TH_KNU1_A = 28,
    TH_KNU1_B = 29,  // Base
    PALM_1    = 30,
    PALM_2    = 31,
    PALM_3    = 32,
    PALM_4    = 33,
    PALM_5    = 34,
    PALM_6    = 35,  // At (0,0,0)
    NUM_BOUNDING_SPHERES = 36  // Must be a mulitple of NSPH_PER_FING
  } HandSphereIndices;
  
  typedef enum {
    LEFT = 0,
    RIGHT = 1,
    UNDEFINED = 2,
  } HandType;
  
  std::string HandCoeffToString(const uint32_t coeff);
  const uint8_t labelFromRGB(const float r, const float g, const float b);

  class HandModel {
  public:
    // Constructor / Destructor
    HandModel(const HandType hand_type);
    ~HandModel();

    static void loadHandModels(const bool left, const bool right);
    static void setHandModelVisibility(const bool visibility);
    static void setHandModelPose(const HandType hand, 
      const HandImageGenerator* im_gen, const float* convnet_coeff);

    // Accessors
    const float getCoeff(const uint32_t index) const;
    float* coeff() { return coeff_; }
    const HandType hand_type() const { return hand_type_; }
    void setRotation(const jtil::math::Float3& euler);
    void getRotation(jtil::math::Float3& euler) const;
    void printCoeff() const;
    void setCoeff(uint32_t index, float coeff_value);
    
    // FILE IO
    void saveToFile(const std::string& dir, const std::string& filename) const;
    void saveBlankFile(const std::string& dir, const std::string& filename) 
      const;
    bool loadFromFile(const std::string& dir, const std::string& filename);
    // Model update 4/11 with the Primesense 1.09 (added thumb twist and other stuff)
    bool loadOldModelFromFile(const std::string& dir, const std::string& filename);  

    void copyCoeffFrom(const HandModel* model);
    void copyCoeffFrom(const float* coeff, const uint32_t ncoeffs);

    static void renormalizeCoeffs(float* coeff);
    void resetPose();

    // Bounding sphere positions:
    // Bones aren't in the correct position (need offsets)
    static const float sph_off_[NUM_BOUNDING_SPHERES * 3];  
    static const float sph_size_[NUM_BOUNDING_SPHERES];

  private:
    float coeff_[NUM_PARAMETERS];  // The current state
    HandType hand_type_;

    static jtil::renderer::GeometryInstance* lhand;
    static jtil::renderer::GeometryInstance* rhand;
    static void setHandModelPose(jtil::renderer::GeometryInstance* hand, 
      const HandImageGenerator* im_gen, const float* convnet_coeff);

    // Non-copyable, non-assignable.
    HandModel(HandModel&);
    HandModel& operator=(const HandModel&);
  };

};  // namespace hand_net
};  // namespace kinect_interface

#endif  // KINECT_INTERFACE_HAND_FIT_HAND_MODEL_HEADER