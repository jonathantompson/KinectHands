#include <random>
#include <thread>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include "kinect_interface/hand_net/robot_hand_model.h"
#include "jtil/renderer/renderer.h"
#include "jtil/renderer/geometry/geometry_manager.h"
#include "jtil/renderer/geometry/geometry_instance.h"
#include "jtil/renderer/geometry/geometry.h"
#include "jtil/renderer/geometry/bone.h"
#include "jtil/data_str/pair.h"
#include "jtil/data_str/vector.h"
#include "kinect_interface/hand_net/hand_net.h"  // for HandCoeffConvnet
#include "kinect_interface/hand_net/hand_image_generator.h"  // for HN_HAND_SIZE

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }

#if defined(__APPLE__)
  #define HAND_ROBOT_MODEL_PATH string("./../../../../../../../../../models/robot_hand/")
#else
  #define HAND_ROBOT_MODEL_PATH string("./models/robot_hand/")
#endif

//#define LOAD_HAND_MESH_JFILE  // Much faster and more compact format!
#define LHAND_ROBOT_MODEL_FILE "robot_hand.dae"
#define LHAND_ROBOT_MODEL_JFILE "robot_hand.jbin"
#define RHAND_ROBOT_MODEL_FILE "robot_hand.dae"
#define RHAND_ROBOT_MODEL_JFILE "robot_hand.jbin"

using namespace jtil::renderer::objects;
using namespace jtil::renderer;
using namespace jtil::math;
using namespace jtil::data_str;
using std::string;

namespace kinect_interface {
namespace hand_net {
  
  RobotHandModel::RobotHandModel(HandType hand_type) {
    index_mesh_node_ = MAX_UINT32;
    index_bone_wrist_ = MAX_UINT32;
    index_bone_palm_ = MAX_UINT32;
    for (uint32_t i = 0; i < 3; i++) {
      index_bone_thumb_[i] = MAX_UINT32;
    }
    for (uint32_t i = 0; i < 4; i++) {
      index_bone_finger1_[i] = MAX_UINT32;
      index_bone_finger2_[i] = MAX_UINT32;
      index_bone_finger3_[i] = MAX_UINT32;
      index_bone_finger4_[i] = MAX_UINT32;
    }


    loadRobotHandGeometry(hand_type);

    visible_ = true;  
  }

  RobotHandModel::~RobotHandModel() {
    // Note, ownership of all geometry is transfered to the renderer class
  }

  void RobotHandModel::loadRobotHandGeometry(HandType type) {
    GeometryManager* gm = Renderer::g_renderer()->geometry_manager();
#ifndef LOAD_HAND_MESH_JFILE
    if (type == HandType::LEFT) {
      model_ = gm->loadModelFromFile(HAND_ROBOT_MODEL_PATH, 
        LHAND_ROBOT_MODEL_FILE, false, false, true);
      gm->saveModelToJBinFile(HAND_ROBOT_MODEL_PATH, LHAND_ROBOT_MODEL_JFILE, 
        model_);
    } else {
      model_ = gm->loadModelFromFile(HAND_ROBOT_MODEL_PATH, 
        RHAND_ROBOT_MODEL_FILE, false, false, true);
      gm->saveModelToJBinFile(HAND_ROBOT_MODEL_PATH, RHAND_ROBOT_MODEL_JFILE, 
        model_);
    }
#else
    if (type == HandType::LEFT) {
      model_ = gm->loadModelFromJBinFile(HAND_ROBOT_MODEL_PATH, 
        LHAND_ROBOT_MODEL_JFILE);
    } else {
      model_ = gm->loadModelFromJBinFile(HAND_ROBOT_MODEL_PATH, 
        RHAND_ROBOT_MODEL_JFILE);
    }
#endif

    // Flatten the render stack to an array of ptrs and parent indices
    // This avoid the overhead of traversing the scene graph using a stack
    // Note: this is O(n^2), but is OK since the number of nodes is low
    Vector<GeometryInstance*> render_stack;
    render_stack.pushBack(model_);
    while (render_stack.size() != 0) {
      GeometryInstance* geom;
      render_stack.popBackUnsafe(geom);
      nodes_.pushBack(geom);

      // In O(n) find the parent node
      uint32_t iparent = (geom == model_) ? MAX_UINT32 - 1 : MAX_UINT32;
      for (uint32_t i = 0; i < nodes_.size() && iparent == MAX_UINT32; i++) {
        if (nodes_[i] == geom->parent()) {
          iparent = i;
        }
      }
      if (iparent == MAX_UINT32) {
        throw std::wruntime_error("INTERNAL ERROR: Couldn't find parent node");
      }
      nodes_parents_.pushBack(iparent);

      // Now add the children to the stack
      for (uint32_t i = 0; i < geom->numChildren(); i ++) {
        render_stack.pushBack(geom->getChild(i));
      }
    }


    // Find all the bones and nodes we need, this is a pretty slow 
    // implementation but it should be OK.
    string base_name;
    if (type == HandType::LEFT) {
      base_name = HAND_ROBOT_MODEL_PATH + LHAND_ROBOT_MODEL_FILE + "/";
    } else {
      base_name = HAND_ROBOT_MODEL_PATH + RHAND_ROBOT_MODEL_FILE + "/";
    }
    for (uint32_t i = 0; i < nodes_.size(); i++) {
      std::string& cur_name = nodes_[i]->name();
      if (cur_name == base_name + "root") {
        index_mesh_node_ = i;
        continue;
      }
      if (cur_name == base_name + "forearm.L") {
        index_bone_wrist_ = i;
        continue;
      }
      if (cur_name == base_name + "hand.L") {
        index_bone_palm_ = i;
        continue;
      }
      
      std::stringstream ss;
      bool found_joint = false;
      for (uint32_t j = 0; j < 3 && !found_joint; j++) {
        ss.str("");
        ss << base_name << "thumb.0" << (j + 1) << ".L";
        if (cur_name == ss.str()) {
          index_bone_thumb_[j] = i;
          found_joint = true;
        }
      }
      if (found_joint) {
        continue;
      }

      // Get the 1st joint of each finger
      found_joint = false;
      for (uint32_t j = 0; j < 4 && !found_joint; j++) {
        ss.str("");
        if (j == 0) {
          ss << base_name << "Bone";
        } else {
          ss << base_name << "Bone.00" << j;
        }
        if (cur_name == ss.str()) {
          index_bone_finger1_[j] = i;
          found_joint = true;
        }
      }
      if (found_joint) {
        continue;
      }

      // Get the 2nd joint of each finger
      found_joint = false;
      for (uint32_t j = 0; j < 4 && !found_joint; j++) {
        ss.str("");
        ss << base_name << "finger" << (j+1) << "joint1";
        if (cur_name == ss.str()) {
          index_bone_finger2_[j] = i;
          found_joint = true;
        }
      }
      if (found_joint) {
        continue;
      }

      // Get the 3rd joint of each finger
      found_joint = false;
      for (uint32_t j = 0; j < 4 && !found_joint; j++) {
        ss.str("");
        ss << base_name << "finger" << (j+1) << "joint2";
        if (cur_name == ss.str()) {
          index_bone_finger3_[j] = i;
          found_joint = true;
        }
      }
      if (found_joint) {
        continue;
      }

      // Get the 4th joint of each finger
      found_joint = false;
      for (uint32_t j = 0; j < 4 && !found_joint; j++) {
        ss.str("");
        ss << base_name << "finger" << (j+1) << "joint3";
        if (cur_name == ss.str()) {
          index_bone_finger4_[j] = i;
          found_joint = true;
        }
      }
      if (found_joint) {
        continue;
      }
    }

    // Now make sure we got all the bone indices
    uint32_t check = 0;
    check = check | index_mesh_node_ | index_bone_wrist_ | index_bone_palm_;
    for (uint32_t i = 0; i < 3; i++) {
      check |= index_bone_thumb_[i];
    }
    for (uint32_t i = 0; i < 4; i++) {
      check |= index_bone_finger1_[i];
      check |= index_bone_finger2_[i];
      check |= index_bone_finger3_[i];
      check |= index_bone_finger4_[i];
    }
    if (check == MAX_UINT32) {
      throw std::wruntime_error("INTERNAL ERROR: Couldn't find a bone");
    }


    // For all meshes in the heirachy set the specular intensity low:
    for (uint32_t i = 0; i < nodes_.size(); i++) {
      nodes_[i]->mtrl().spec_intensity = 0.3f;
    }
  }

  void RobotHandModel::updateMatrices(const float* coeff) {
    jtil::math::Float4x4 mat_tmp1;
    jtil::math::Float4x4 mat_tmp2;
    jtil::math::Float4x4 mat_tmp3;
    // Set the root matrix:
    Float4x4* mat = &model_->mat();
    euler2RotMatGM(*mat, coeff[HAND_ORIENT_X], coeff[HAND_ORIENT_Y],
      coeff[HAND_ORIENT_Z]);
    mat->leftMultTranslation(coeff[HAND_POS_X], coeff[HAND_POS_Y], coeff[HAND_POS_Z]);
    mat->rightMultScale(coeff[SCALE], coeff[SCALE], coeff[SCALE]); 
 
    // Set the palm bone (depending on wrist angle)
    mat = &nodes_[index_bone_wrist_]->mat();
    rotateMatXAxisGM(mat_tmp1, coeff[WRIST_PHI]);
    rotateMatZAxisGM(mat_tmp2, coeff[WRIST_THETA]);
    Float4x4::multSIMD(mat_tmp3, mat_tmp1, mat_tmp2);
    Float4x4::multSIMD(*mat, nodes_[index_bone_wrist_]->bone()->rest_transform, mat_tmp3);

    // Set the finger bones
// #pragma omp parallel for num_threads(4)
    for (int i = 0; i < 4; i++) {
      float theta, phi, psi;
      Float4x4* mat;
      //jtil::math::Float4x4 mat_tmp1;  // OMP Needs separate destination vars
      //jtil::math::Float4x4 mat_tmp2;

      // Root
      theta = coeff[F0_ROOT_THETA + i * FINGER_NUM_COEFF];
      phi = coeff[F0_ROOT_PHI + i * FINGER_NUM_COEFF];
      psi = 0;
      mat = &nodes_[index_bone_finger1_[i]]->mat();
      euler2RotMatGM(mat_tmp1, psi, theta, phi);
      Float4x4::multSIMD(*mat, nodes_[index_bone_finger1_[i]]->bone()->rest_transform,
        mat_tmp1);

      // K1 base
      theta = coeff[F0_THETA + i * FINGER_NUM_COEFF];
      phi = coeff[F0_PHI + i * FINGER_NUM_COEFF];
      psi = coeff[F0_TWIST + i];
      mat = &nodes_[index_bone_finger2_[i]]->mat();
      euler2RotMatGM(mat_tmp1, psi, theta, phi);
      Float4x4::multSIMD(*mat, nodes_[index_bone_finger2_[i]]->bone()->rest_transform, 
        mat_tmp1);
      mat->rightMultScale(1.0f, 1.0f + coeff[F0_LENGTH + i], 1.0f);  // Scale this node

      mat = &nodes_[index_bone_finger3_[i]]->mat();
      float k2_theta = coeff[F0_KNUCKLE_MID + i * FINGER_NUM_COEFF];
      rotateMatXAxisGM(mat_tmp1, k2_theta);
      const Float4x4& bone_mid = nodes_[index_bone_finger3_[i]]->bone()->rest_transform;
      Float3 bone_mid_pos;
      Float4x4::getTranslation(bone_mid_pos, bone_mid);
      float bone_base_length = bone_mid_pos.length();
      mat_tmp2.set(bone_mid);
      // Move bone by fraction of the bone length:
      mat_tmp2.leftMultTranslation(0, bone_base_length * coeff[F0_LENGTH + i], 0);  
      Float4x4::multSIMD(*mat, mat_tmp2, mat_tmp1);
      mat->leftMultScale(1.0f, 1.0f / (1.0f + coeff[F0_LENGTH + i]), 1.0f);  // Undo parent scale
      mat->rightMultScale(1.0f, 1.0f + coeff[F0_LENGTH + i], 1.0f);  // Scale this node

      mat = &nodes_[index_bone_finger4_[i]]->mat();
      float k3_theta = coeff[F0_KNUCKLE_END + i * FINGER_NUM_COEFF];
      rotateMatXAxisGM(mat_tmp1, k3_theta);
      const Float4x4& bone_tip = nodes_[index_bone_finger4_[i]]->bone()->rest_transform;
      Float3 bone_tip_pos;
      Float4x4::getTranslation(bone_tip_pos, bone_tip);
      float bone_mid_length = bone_tip_pos.length();
      mat_tmp2.set(bone_tip);
      // Move bone by fraction of the bone length:
      mat_tmp2.leftMultTranslation(0, bone_mid_length * coeff[F0_LENGTH + i], 0);
      Float4x4::multSIMD(*mat, mat_tmp2, mat_tmp1);

      mat->leftMultScale(1.0f, 1.0f / (1.0f + coeff[F0_LENGTH + i]), 1.0f);  // Undo parent scale
      mat->rightMultScale(1.0f, 1.0f + coeff[F0_LENGTH + i], 1.0f);  // Scale this node
    }

    // Set the thumb bones
    float theta = coeff[THUMB_THETA];
    float phi = coeff[THUMB_PHI];
    float psi = coeff[THUMB_TWIST];
    mat = &nodes_[index_bone_thumb_[0]]->mat();
    euler2RotMatGM(mat_tmp1, psi, theta, phi);
    Float4x4::multSIMD(*mat, nodes_[index_bone_thumb_[0]]->bone()->rest_transform, mat_tmp1);

    theta = coeff[THUMB_K1_THETA];
    phi = coeff[THUMB_K1_PHI];
    mat = &nodes_[index_bone_thumb_[1]]->mat();
    rotateMatZAxisGM(mat_tmp1, theta);
    rotateMatXAxisGM(mat_tmp2, phi);
    Float4x4::multSIMD(mat_tmp3, mat_tmp1, mat_tmp2);

    const Float4x4& bone_mid = nodes_[index_bone_thumb_[1]]->bone()->rest_transform;
    mat_tmp2.set(bone_mid);
    Float4x4::multSIMD(*mat, mat_tmp2, mat_tmp3);
    mat->rightMultScale(1.0f, 1.0f + coeff[THUMB_LENGTH], 1.0f);  // Scale this node

    phi = coeff[THUMB_K2_PHI];
    mat = &nodes_[index_bone_thumb_[2]]->mat();
    rotateMatXAxisGM(mat_tmp1, phi);
    const Float4x4& bone_tip = nodes_[index_bone_thumb_[2]]->bone()->rest_transform;
    Float3 bone_tip_pos;
    Float4x4::getTranslation(bone_tip_pos, bone_tip);
    float bone_mid_length = bone_tip_pos.length();
    mat_tmp2.set(bone_tip);
    // Move bone by fraction of the bone length:
    mat_tmp2.leftMultTranslation(0, bone_mid_length * coeff[THUMB_LENGTH], 0);
    Float4x4::multSIMD(*mat, mat_tmp2, mat_tmp1);
    mat->leftMultScale(1.0f, 1.0f / (1.0f + coeff[THUMB_LENGTH]), 1.0f);  // Undo parent scale
    mat->rightMultScale(1.0f, 1.0f + coeff[THUMB_LENGTH], 1.0f);  // Scale this node
  }

  extern Float4x4 tmp;
  void RobotHandModel::updateHeirachyMatrices() {
    // Since the nodes array is bfs ordered, we just have to traverse it 
    // linearly.
    nodes_[0]->mat_hierarchy().set(nodes_[0]->mat());
    Float4x4::inverse(*nodes_[0]->mat_hierarchy_inv(), 
      nodes_[0]->mat_hierarchy());
    for (uint32_t i = 1; i < nodes_.size(); i++) {
      GeometryInstance* cur_geom = nodes_[i];
      Float4x4::multSIMD(cur_geom->mat_hierarchy(),
        cur_geom->parent()->mat_hierarchy(), cur_geom->mat());

      // Update the bone matrix
      if (cur_geom->bone() != NULL) {  
        // This node is a bone, calculate the local bone space matrix
        Float4x4* cur_bone_final_trans = cur_geom->bone_transform();
        Float4x4* bone_offset = &cur_geom->bone()->bone_offset;

        Float4x4::multSIMD(tmp, cur_geom->mat_hierarchy(), *bone_offset);
        Float4x4::multSIMD(*cur_bone_final_trans, 
          *cur_geom->bone_root_node()->mat_hierarchy_inv(), tmp);
      }
    }
  }

  void RobotHandModel::setRenderVisiblity(const bool visible) {
    if (visible_ != visible) {
      visible_ = visible;
      for (uint32_t i = 0; i < nodes_.size(); i++) {
        nodes_[i]->render() = visible_;
      }
    }
  }

  const bool RobotHandModel::getRenderVisiblity() {
    return visible_;
  }

  // These next few methods are to avoid the cos and sin double functions in 
  // the Mat4x4 template
  // ********************************************************************
  // FLOAT HELPER FUNCTIONS
  void RobotHandModel::euler2RotMatGM(Float4x4& a, const float x_angle, 
    const float y_angle, const float z_angle) {
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
    float c1 = cosf(x_angle);
    float s1 = sinf(x_angle);
    float c2 = cosf(y_angle);
    float s2 = sinf(y_angle);
    float c3 = cosf(z_angle);
    float s3 = sinf(z_angle);
#ifdef COLUMN_MAJOR
    a.m[0] = c1*c2;
    a.m[4] = -c1*s2*c3 + s1*s3;
    a.m[8] = c1*s2*s3 + s1*c3;
    a.m[12] = 0;
    a.m[1] = s2;
    a.m[5] = c2*c3;
    a.m[9] = -c2*s3;
    a.m[13] = 0;
    a.m[2] = -s1*c2;
    a.m[6] = s1*s2*c3 + c1*s3;
    a.m[10] = -s1*s2*s3 + c1*c3;
    a.m[14] = 0;
    a.m[3] = 0;
    a.m[7] = 0;
    a.m[11] = 0;
    a.m[15] = 1;
#endif
#ifdef ROW_MAJOR
    a.m[0] = c1*c2;
    a.m[1] = -c1*s2*c3 + s1*s3;
    a.m[2] = c1*s2*s3 + s1*c3;
    a.m[3] = 0;
    a.m[4] = s2;
    a.m[5] = c2*c3;
    a.m[6] = -c2*s3;
    a.m[7] = 0;
    a.m[8] = -s1*c2;
    a.m[9] = s1*s2*c3 + c1*s3;
    a.m[10] = -s1*s2*s3 + c1*c3;
    a.m[11] = 0;
    a.m[12] = 0;
    a.m[13] = 0;
    a.m[14] = 0;
    a.m[15] = 1;
#endif
  }

  void RobotHandModel::rotateMatZAxisGM(Float4x4& ret, const float angle) {
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
#ifdef ROW_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = -sin_angle;
    ret.m[2] = 0;
    ret.m[3] = 0;
    ret.m[4] = sin_angle;
    ret.m[5] = cos_angle;
    ret.m[6] = 0;
    ret.m[7] = 0;
    ret.m[8] = 0;
    ret.m[9] = 0;
    ret.m[10] = 1;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = sin_angle;
    ret.m[2] = 0;
    ret.m[3] = 0;
    ret.m[4] = -sin_angle;
    ret.m[5] = cos_angle;
    ret.m[6] = 0;
    ret.m[7] = 0;
    ret.m[8] = 0;
    ret.m[9] = 0;
    ret.m[10] = 1;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
  };

  void RobotHandModel::rotateMatYAxisGM(Float4x4& ret, const float angle) {
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
#ifdef ROW_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = 0;
    ret.m[2] = sin_angle;
    ret.m[3] = 0;
    ret.m[4] = 0;
    ret.m[5] = 1;
    ret.m[6] = 0;
    ret.m[7] = 0;
    ret.m[8] = -sin_angle;
    ret.m[9] = 0;
    ret.m[10] = cos_angle;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = 0;
    ret.m[2] = -sin_angle;
    ret.m[3] = 0;
    ret.m[4] = 0;
    ret.m[5] = 1;
    ret.m[6] = 0;
    ret.m[7] = 0;
    ret.m[8] = sin_angle;
    ret.m[9] = 0;
    ret.m[10] = cos_angle;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
  };

  void RobotHandModel::rotateMatXAxisGM(Float4x4& ret, const float angle) {
    float cos_angle = cosf(angle);
    float sin_angle = sinf(angle);
#ifdef ROW_MAJOR
    ret.m[0] = 1;
    ret.m[1] = 0;
    ret.m[2] = 0;
    ret.m[3] = 0;
    ret.m[4] = 0;
    ret.m[5] = cos_angle;
    ret.m[6] = -sin_angle;
    ret.m[7] = 0;
    ret.m[8] = 0;
    ret.m[9] = sin_angle;
    ret.m[10] = cos_angle;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = 1;
    ret.m[1] = 0;
    ret.m[2] = 0;
    ret.m[3] = 0;
    ret.m[4] = 0;
    ret.m[5] = cos_angle;
    ret.m[6] = sin_angle;
    ret.m[7] = 0;
    ret.m[8] = 0;
    ret.m[9] = -sin_angle;
    ret.m[10] = cos_angle;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
  };

  // ********************************************************************
  // DOUBLE HELPER FUNCTIONS
  void RobotHandModel::euler2RotMatGM(Double4x4& a, const double x_angle, 
    const double y_angle, const double z_angle) {
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToMatrix/index.htm
    double c1 = cos(x_angle);
    double s1 = sin(x_angle);
    double c2 = cos(y_angle);
    double s2 = sin(y_angle);
    double c3 = cos(z_angle);
    double s3 = sin(z_angle);
#ifdef COLUMN_MAJOR
    a.m[0] = c1*c2;
    a.m[4] = -c1*s2*c3 + s1*s3;
    a.m[8] = c1*s2*s3 + s1*c3;
    a.m[12] = 0;
    a.m[1] = s2;
    a.m[5] = c2*c3;
    a.m[9] = -c2*s3;
    a.m[13] = 0;
    a.m[2] = -s1*c2;
    a.m[6] = s1*s2*c3 + c1*s3;
    a.m[10] = -s1*s2*s3 + c1*c3;
    a.m[14] = 0;
    a.m[3] = 0;
    a.m[7] = 0;
    a.m[11] = 0;
    a.m[15] = 1;
#endif
#ifdef ROW_MAJOR
    a.m[0] = c1*c2;
    a.m[1] = -c1*s2*c3 + s1*s3;
    a.m[2] = c1*s2*s3 + s1*c3;
    a.m[3] = 0;
    a.m[4] = s2;
    a.m[5] = c2*c3;
    a.m[6] = -c2*s3;
    a.m[7] = 0;
    a.m[8] = -s1*c2;
    a.m[9] = s1*s2*c3 + c1*s3;
    a.m[10] = -s1*s2*s3 + c1*c3;
    a.m[11] = 0;
    a.m[12] = 0;
    a.m[13] = 0;
    a.m[14] = 0;
    a.m[15] = 1;
#endif
  }

  void RobotHandModel::rotateMatZAxisGM(Double4x4& ret, const double angle) {
    double cos_angle = cos(angle);
    double sin_angle = sin(angle);
#ifdef ROW_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = -sin_angle;
    ret.m[2] = 0;
    ret.m[3] = 0;
    ret.m[4] = sin_angle;
    ret.m[5] = cos_angle;
    ret.m[6] = 0;
    ret.m[7] = 0;
    ret.m[8] = 0;
    ret.m[9] = 0;
    ret.m[10] = 1;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = sin_angle;
    ret.m[2] = 0;
    ret.m[3] = 0;
    ret.m[4] = -sin_angle;
    ret.m[5] = cos_angle;
    ret.m[6] = 0;
    ret.m[7] = 0;
    ret.m[8] = 0;
    ret.m[9] = 0;
    ret.m[10] = 1;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
  };

  void RobotHandModel::rotateMatYAxisGM(Double4x4& ret, const double angle) {
    double cos_angle = cos(angle);
    double sin_angle = sin(angle);
#ifdef ROW_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = 0;
    ret.m[2] = sin_angle;
    ret.m[3] = 0;
    ret.m[4] = 0;
    ret.m[5] = 1;
    ret.m[6] = 0;
    ret.m[7] = 0;
    ret.m[8] = -sin_angle;
    ret.m[9] = 0;
    ret.m[10] = cos_angle;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = cos_angle;
    ret.m[1] = 0;
    ret.m[2] = -sin_angle;
    ret.m[3] = 0;
    ret.m[4] = 0;
    ret.m[5] = 1;
    ret.m[6] = 0;
    ret.m[7] = 0;
    ret.m[8] = sin_angle;
    ret.m[9] = 0;
    ret.m[10] = cos_angle;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
  };

  void RobotHandModel::rotateMatXAxisGM(Double4x4& ret, const double angle) {
    double cos_angle = cos(angle);
    double sin_angle = sin(angle);
#ifdef ROW_MAJOR
    ret.m[0] = 1;
    ret.m[1] = 0;
    ret.m[2] = 0;
    ret.m[3] = 0;
    ret.m[4] = 0;
    ret.m[5] = cos_angle;
    ret.m[6] = -sin_angle;
    ret.m[7] = 0;
    ret.m[8] = 0;
    ret.m[9] = sin_angle;
    ret.m[10] = cos_angle;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
#ifdef COLUMN_MAJOR
    ret.m[0] = 1;
    ret.m[1] = 0;
    ret.m[2] = 0;
    ret.m[3] = 0;
    ret.m[4] = 0;
    ret.m[5] = cos_angle;
    ret.m[6] = sin_angle;
    ret.m[7] = 0;
    ret.m[8] = 0;
    ret.m[9] = -sin_angle;
    ret.m[10] = cos_angle;
    ret.m[11] = 0;
    ret.m[12] = 0;
    ret.m[13] = 0;
    ret.m[14] = 0;
    ret.m[15] = 1;
#endif
  };

}  // namespace hand_net
}  // namespace kinect_interface