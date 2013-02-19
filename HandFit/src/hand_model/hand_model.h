//
//  hand_model.h
//
//  Created by Jonathan Tompson on 8/17/12.
//
//  Code to perform Levenberg-Marquardt non-linear optimization to fit a very
//  simple hand model to input depth data.  It does this by rendering the
//  hand model using openGL then, estimating the jacobian using finite 
//  differences, and using this in the internal loop of LM.
//
//  Obviously, the region of convergence is finite so starting values matter.
//  Secondly, none of the functions here are thread safe.

#ifndef HAND_MODEL_HAND_MODEL_HEADER
#define HAND_MODEL_HAND_MODEL_HEADER

#include "renderer/open_gl_common.h"  // GLfloat
#include "math/math_types.h"
#include "data_str/vector.h"
#include "Eigen"

#define DEPTH_IMAGE_WIDTH 640
#define DEPTH_IMAGE_HEIGHT 480
#define DEPTH_IMAGE_DIM 307200  // 640 * 480
#define HAND_NUM_COEFF 26  // The number of coefficients to use when optimizing
#define FINGER_NUM_COEFF 3
#define HAND_MODEL_DEFAULT_SCALE 50.0f

// #define USE_SIMPLE_GEOMETRY  // Otherwise use dense mesh
#if defined(__APPLE__)
  #define HAND_MODEL_PATH string("../../../../../../../../models/")
#else
  #define HAND_MODEL_PATH string("./models/")
#endif

#define LOAD_HAND_MESH_JFILE  // Much faster and more compact format!
//#define LHAND_MODEL_FILE "hand.dae"
//#define LHAND_MODEL_JFILE "hand.jbin"
//#define LHAND_MODEL_FILE "hand_short_wrist.dae"
//#define LHAND_MODEL_JFILE "hand_short_wrist.jbin"
#define LHAND_MODEL_FILE "hand_medium_wrist.dae"
#define LHAND_MODEL_JFILE "hand_medium_wrist.jbin"
//#define LHAND_MODEL_FILE "hand_vertex_colors.dae"
//#define LHAND_MODEL_JFILE "hand_vertex_colors.jbin"

//#define RHAND_MODEL_FILE "hand_right.dae"
//#define RHAND_MODEL_JFILE "hand_right.jbin"
//#define RHAND_MODEL_FILE "hand_short_wrist_right.dae"
//#define RHAND_MODEL_JFILE "hand_short_wrist_right.jbin"
#define RHAND_MODEL_FILE "hand_medium_wrist_right.dae"
#define RHAND_MODEL_JFILE "hand_medium_wrist_right.jbin"
//#define RHAND_MODEL_FILE "hand_vertex_colors_right.dae"
//#define RHAND_MODEL_JFILE "hand_vertex_colors_right.jbin"

namespace hand_model {
  
  class HandModelRenderer;

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
  
  std::string HandCoeffToString(uint32_t coeff);
  uint8_t labelFromRGB(float r, float g, float b);

  class HandModel {
  public:
    // Constructor / Destructor
    HandModel(HandType hand_type);
    ~HandModel();

    // Accessors
    float getCoeff(uint32_t index);
    Eigen::MatrixXf& coeff() { return coeff_; }
    Eigen::MatrixXf* coeff_ptr() { return &coeff_; }
    HandType hand_type() { return hand_type_; }
    void setRotation(math::FloatQuat* quat);
    void getRotation(math::FloatQuat* quat);
    void printCoeff() const;
    inline const float local_scale() { return local_scale_; }
    inline void local_scale(float scale) { local_scale_ = scale; }

    // Modifiers
    void setCoeff(uint32_t index, float coeff_value);
    static void renormalizeCoeffs(Eigen::MatrixXf& coeff);
    void resetPose();

    void saveToFile(std::string directory, std::string filename);
    void saveBlankFile(std::string directory, std::string filename);
    bool loadFromFile(std::string directory, std::string filename);
  
    // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    static float scale;
    static float wrist_length;
    static float wrist_twist;

  private:
    Eigen::MatrixXf coeff_;  // The current state
    float local_scale_;
    HandType hand_type_;

    // Non-copyable, non-assignable.
    HandModel(HandModel&);
    HandModel& operator=(const HandModel&);
  };
};  // unnamed namespace

#endif  // HAND_MODEL_HAND_MODEL_HEADER
