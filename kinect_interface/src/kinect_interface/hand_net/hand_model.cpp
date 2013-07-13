#include <random>
#include <thread>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include "kinect_interface/hand_net/hand_model.h"
#include "jtil/renderer/renderer.h"
#include "jtil/renderer/geometry/geometry_manager.h"
#include "jtil/renderer/geometry/geometry_instance.h"
#include "jtil/renderer/geometry/geometry.h"
#include "jtil/renderer/geometry/bone.h"
#include "jtil/renderer/objects/bsphere.h"
#include "jtil/data_str/pair.h"
#include "jtil/data_str/vector.h"
#include "kinect_interface/hand_net/hand_net.h"  // for HandCoeffConvnet
#include "kinect_interface/hand_net/hand_image_generator.h"  // for HN_HAND_SIZE

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }

#if defined(__APPLE__)
  #define HAND_MODEL_PATH string("./../../../../../../../../../models/lib_hand/")
#else
  #define HAND_MODEL_PATH string("./models/lib_hand/")
#endif

#define LOAD_HAND_MESH_JFILE  // Much faster and more compact format!
#define LHAND_MODEL_FILE "hand_palm_parent_medium_wrist_dec_0.05.dae"
#define LHAND_MODEL_JFILE "hand_palm_parent_medium_wrist_dec_0.05.jbin"
//#define RHAND_MODEL_FILE "hand_palm_parent_medium_wrist_dec_0.05_right.dae"
//#define RHAND_MODEL_JFILE "hand_palm_parent_medium_wrist_dec_0.05_right.jbin"
#define RHAND_MODEL_FILE "hand_palm_parent_short_wrist_right.dae"
#define RHAND_MODEL_JFILE "hand_palm_parent_short_wrist_right.jbin"

using namespace jtil::renderer::objects;
using namespace jtil::renderer;
using namespace jtil::math;
using namespace jtil::data_str;
using std::string;

namespace kinect_interface {
namespace hand_net {
  
  HandModel::HandModel(HandType hand_type) {
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


    loadHandGeometry(hand_type);

    // HACK: The model's normals are inside out --> Fix them
    if (hand_type == HandType::RIGHT) {
      GeometryManager* gm = Renderer::g_renderer()->geometry_manager();
      Geometry* geom = gm->findGeometryByName(string(HAND_MODEL_PATH) + 
        string(RHAND_MODEL_FILE) + "/Armature_hand_mesh-skin/0");
      if (geom == NULL) {
        throw std::wruntime_error("HandModel::HandModel() - ERROR: "
          "Couldn't find Geoemtry for right hand mesh!");
      }
      geom->unsync();
      Vector<Float3>& norms = geom->nor();
      for (uint32_t i = 0; i < norms.size(); i++) {
        Float3::scale(norms[i], -1.0f);
      }
      geom->sync();
    }

    visible_ = true;  
  }

  HandModel::~HandModel() {
    // Note, ownership of all geometry is transfered to the renderer class
  }

  void HandModel::loadHandGeometry(HandType type) {
    GeometryManager* gm = Renderer::g_renderer()->geometry_manager();
#ifndef LOAD_HAND_MESH_JFILE
    if (type == HandType::LEFT) {
      model_ = gm->loadModelFromFile(HAND_MODEL_PATH, LHAND_MODEL_FILE);
      gm->saveModelToJBinFile(HAND_MODEL_PATH, LHAND_MODEL_JFILE, model_);
    } else {
      model_ = gm->loadModelFromFile(HAND_MODEL_PATH, RHAND_MODEL_FILE);
      gm->saveModelToJBinFile(HAND_MODEL_PATH, RHAND_MODEL_JFILE, model_);
    }
#else
    if (type == HandType::LEFT) {
      model_ = gm->loadModelFromJBinFile(HAND_MODEL_PATH, LHAND_MODEL_JFILE);
    } else {
      model_ = gm->loadModelFromJBinFile(HAND_MODEL_PATH, RHAND_MODEL_JFILE);
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

      // Also copy the model matrix into a double array
      Double4x4 cur_mat;
      for (uint32_t i = 0; i < 16; i++) {
        cur_mat.m[i] = (double)geom->mat()[i];
      }
      nodes_mat_.pushBack(cur_mat);
      nodes_heirachy_mat_.pushBack(cur_mat);

      // Now add the children to the stack
      for (uint32_t i = 0; i < geom->numChildren(); i ++) {
        render_stack.pushBack(geom->getChild(i));
      }
    }


    // Find all the bones and nodes we need, this is a pretty slow 
    // implementation but it should be OK.
    string base_name;
    if (type == HandType::LEFT) {
      base_name = HAND_MODEL_PATH + LHAND_MODEL_FILE + "/";
    } else {
      base_name = HAND_MODEL_PATH + RHAND_MODEL_FILE + "/";
    }
    for (uint32_t i = 0; i < nodes_.size(); i++) {
      std::string& cur_name = nodes_[i]->name();
      if (cur_name == base_name + "hand_mesh") {
        index_mesh_node_ = i;
        continue;
      }
      if (cur_name == base_name + "carpals") {
        index_bone_wrist_ = i;
        continue;
      }
      if (cur_name == base_name + "metacarpals") {
        index_bone_palm_ = i;
        continue;
      }
      
      std::stringstream ss;
      bool found_joint = false;
      for (uint32_t j = 0; j < 3 && !found_joint; j++) {
        ss.str("");
        ss << base_name << "finger5joint" << (j + 1);
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

    // Copy the bone rest transforms (to double)
    nodes_bone_rest_transform_.capacity(nodes_.size());
    nodes_bone_rest_transform_.resize(nodes_.size());
    nodes_bone_transform_.capacity(nodes_.size());
    nodes_bone_transform_.resize(nodes_.size());
    nodes_bone_offset_.capacity(nodes_.size());
    nodes_bone_offset_.resize(nodes_.size());
    for (uint32_t i = 0; i < nodes_.size(); i++) {
      if (nodes_[i]->bone() != NULL) {
        for (uint32_t j = 0; j < 16; j++) {
          nodes_bone_rest_transform_[i].m[j] = 
            (double)nodes_[i]->bone()->rest_transform[j];
        }
        for (uint32_t j = 0; j < 16; j++) {
          nodes_bone_transform_[i].m[j] = 
            (double)nodes_[i]->bone_transform()->m[j];
        }
        for (uint32_t j = 0; j < 16; j++) {
          nodes_bone_offset_[i].m[j] = 
            (double)nodes_[i]->bone()->bone_offset.m[j];
        }
      }
    }

    // For all meshes in the heirachy set the specular intensity low:
    for (uint32_t i = 0; i < nodes_.size(); i++) {
      nodes_[i]->mtrl().spec_intensity = 0.3f;
    }

    // Now create bounding spheres:
    bspheres_.capacity(HandSphereIndices::NUM_BOUNDING_SPHERES);
    bspheres_.resize(HandSphereIndices::NUM_BOUNDING_SPHERES);
    bsphere_parent_ind_.capacity(HandSphereIndices::NUM_BOUNDING_SPHERES);
    bsphere_parent_ind_.resize(HandSphereIndices::NUM_BOUNDING_SPHERES);
    for (uint32_t i = 0; i < 4; i++) {
      addBoneBSphere((uint32_t)F1_KNU3_A + i * num_bspheres_per_group(), 
        nodes_[index_bone_finger4_[i]]);
      addBoneBSphere((uint32_t)F1_KNU3_B + i * num_bspheres_per_group(), 
        nodes_[index_bone_finger4_[i]]);
      addBoneBSphere((uint32_t)F1_KNU2_A + i * num_bspheres_per_group(), 
        nodes_[index_bone_finger3_[i]]);
      addBoneBSphere((uint32_t)F1_KNU2_B + i * num_bspheres_per_group(), 
        nodes_[index_bone_finger3_[i]]);
      addBoneBSphere((uint32_t)F1_KNU1_A + i * num_bspheres_per_group(), 
        nodes_[index_bone_finger2_[i]]);
      addBoneBSphere((uint32_t)F1_KNU1_B + i * num_bspheres_per_group(), 
        nodes_[index_bone_finger2_[i]]);
    }

    addBoneBSphere((uint32_t)TH_KNU3_A, nodes_[index_bone_thumb_[2]]);
    addBoneBSphere((uint32_t)TH_KNU3_B, nodes_[index_bone_thumb_[2]]);
    addBoneBSphere((uint32_t)TH_KNU2_A, nodes_[index_bone_thumb_[1]]);
    addBoneBSphere((uint32_t)TH_KNU2_B, nodes_[index_bone_thumb_[1]]);
    addBoneBSphere((uint32_t)TH_KNU1_A, nodes_[index_bone_thumb_[0]]);
    addBoneBSphere((uint32_t)TH_KNU1_B, nodes_[index_bone_thumb_[0]]);
    
    addBoneBSphere((uint32_t)PALM_1, nodes_[index_bone_palm_]);
    addBoneBSphere((uint32_t)PALM_2, nodes_[index_bone_palm_]);
    addBoneBSphere((uint32_t)PALM_3, nodes_[index_bone_palm_]);
    addBoneBSphere((uint32_t)PALM_4, nodes_[index_bone_palm_]);
    addBoneBSphere((uint32_t)PALM_5, nodes_[index_bone_palm_]);
    addBoneBSphere((uint32_t)PALM_6, nodes_[index_bone_palm_]);
  }

  void HandModel::addBoneBSphere(const uint32_t ibone, 
    GeometryInstance* cur_bone) {
    if (cur_bone->bone() == NULL) {
      throw std::wruntime_error("HandModel::addBoneBSphere() - ERROR: "
        "This GeometryInstance doesn't have an attached bone!");
    }
    Float4x4 bone_offset_inv;
    Float4x4::inverse(bone_offset_inv, cur_bone->bone()->bone_offset);
    const Float3 origin(0, 0, 0);
    Float3 center;
    Float3::affineTransformPos(center, bone_offset_inv, origin);
    center.accum(&HandModelCoeff::sph_off_[ibone * 3]);

    BSphere* cur_sphere = new BSphere(HandModelCoeff::sph_size_[ibone], center,
      cur_bone);
    bspheres_[ibone] = cur_sphere;

    // O(n) search for the correct parent index
    bsphere_parent_ind_[ibone] = MAX_UINT32;
    for (uint32_t i = 0; i < nodes_.size() && 
      bsphere_parent_ind_[ibone] == MAX_UINT32; i++) {
      if (nodes_[i] == cur_bone) {
        bsphere_parent_ind_[ibone] = i;
      }
    }
    if (bsphere_parent_ind_[ibone] == MAX_UINT32) {
      throw std::wruntime_error("INTERNAL ERROR: Couldn't find bsphere parent");
    }
  }

  void HandModel::updateMatrices(const float* coeff, 
    const float hand_size) {
    jtil::math::Float4x4 mat_tmp1;
    jtil::math::Float4x4 mat_tmp2;
    jtil::math::Float4x4 mat_tmp3;
    // Set the root matrix:
    Float4x4* mat = &model_->mat();
    euler2RotMatGM(*mat, coeff[HAND_ORIENT_X], coeff[HAND_ORIENT_Y],
      coeff[HAND_ORIENT_Z]);
    mat->leftMultTranslation(coeff[HAND_POS_X], coeff[HAND_POS_Y], coeff[HAND_POS_Z]);
    mat->rightMultScale(hand_size * coeff[SCALE], hand_size * coeff[SCALE], 
      hand_size * coeff[SCALE]); 
 
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

  void HandModel::updateDoubleMatrices(const double* coeff) {
    jtil::math::Double4x4 mat_tmp1;
    jtil::math::Double4x4 mat_tmp2;
    jtil::math::Double4x4 mat_tmp3;
    // Set the root matrix:
    Double4x4* mat = &nodes_mat_[0];
    euler2RotMatGM(*mat, coeff[HAND_ORIENT_X], coeff[HAND_ORIENT_Y],
      coeff[HAND_ORIENT_Z]);
    mat->leftMultTranslation(coeff[HAND_POS_X], coeff[HAND_POS_Y], coeff[HAND_POS_Z]);
    mat->rightMultScale(coeff[SCALE], coeff[SCALE], coeff[SCALE]); 
 
    // Set the palm bone (depending on wrist angle)
    mat = &nodes_mat_[index_bone_wrist_];
    rotateMatXAxisGM(mat_tmp1, coeff[WRIST_PHI]);
    rotateMatZAxisGM(mat_tmp2, coeff[WRIST_THETA]);
    Double4x4::mult(mat_tmp3, mat_tmp1, mat_tmp2);
    Double4x4::mult(*mat, nodes_bone_rest_transform_[index_bone_wrist_], mat_tmp3);

    // Set the finger bones
//#pragma omp parallel for num_threads(4)
    for (int i = 0; i < 4; i++) {
      double theta, phi, psi;
      Double4x4* mat;
      //jtil::math::Double4x4 mat_tmp1;  // OMP Needs separate destination vars
      //jtil::math::Double4x4 mat_tmp2;

      // Root
      theta = coeff[F0_ROOT_THETA + i * FINGER_NUM_COEFF];
      phi = coeff[F0_ROOT_PHI + i * FINGER_NUM_COEFF];
      psi = 0;
      mat = &nodes_mat_[index_bone_finger1_[i]];
      euler2RotMatGM(mat_tmp1, psi, theta, phi);
      Double4x4::mult(*mat, nodes_bone_rest_transform_[index_bone_finger1_[i]],
        mat_tmp1);

      // K1 base
      theta = coeff[F0_THETA + i * FINGER_NUM_COEFF];
      phi = coeff[F0_PHI + i * FINGER_NUM_COEFF];
      psi = coeff[F0_TWIST + i];
      mat = &nodes_mat_[index_bone_finger2_[i]];
      euler2RotMatGM(mat_tmp1, psi, theta, phi);
      Double4x4::mult(*mat, nodes_bone_rest_transform_[index_bone_finger2_[i]], 
        mat_tmp1);
      mat->rightMultScale(1.0, 1.0 + coeff[F0_LENGTH + i], 1.0);  // Scale this node

      mat = &nodes_mat_[index_bone_finger3_[i]];
      double k2_theta = coeff[F0_KNUCKLE_MID + i * FINGER_NUM_COEFF];
      rotateMatXAxisGM(mat_tmp1, k2_theta);
      const Double4x4& bone_mid = nodes_bone_rest_transform_[index_bone_finger3_[i]];
      Double3 bone_mid_pos;
      Double4x4::getTranslation(bone_mid_pos, bone_mid);
      double bone_base_length = bone_mid_pos.length();
      mat_tmp2.set(bone_mid);
      // Move bone by fraction of the bone length:
      mat_tmp2.leftMultTranslation(0, bone_base_length * coeff[F0_LENGTH + i], 0);  
      Double4x4::mult(*mat, mat_tmp2, mat_tmp1);
      mat->leftMultScale(1.0, 1.0 / (1.0 + coeff[F0_LENGTH + i]), 1.0);  // Undo parent scale
      mat->rightMultScale(1.0, 1.0 + coeff[F0_LENGTH + i], 1.0);  // Scale this node

      mat = &nodes_mat_[index_bone_finger4_[i]];
      double k3_theta = coeff[F0_KNUCKLE_END + i * FINGER_NUM_COEFF];
      rotateMatXAxisGM(mat_tmp1, k3_theta);
      const Double4x4& bone_tip = nodes_bone_rest_transform_[index_bone_finger4_[i]];
      Double3 bone_tip_pos;
      Double4x4::getTranslation(bone_tip_pos, bone_tip);
      double bone_mid_length = bone_tip_pos.length();
      mat_tmp2.set(bone_tip);
      // Move bone by fraction of the bone length:
      mat_tmp2.leftMultTranslation(0, bone_mid_length * coeff[F0_LENGTH + i], 0);
      Double4x4::mult(*mat, mat_tmp2, mat_tmp1);

      mat->leftMultScale(1.0, 1.0 / (1.0 + coeff[F0_LENGTH + i]), 1.0);  // Undo parent scale
      mat->rightMultScale(1.0, 1.0 + coeff[F0_LENGTH + i], 1.0);  // Scale this node
    }

    // Set the thumb bones
    double theta = coeff[THUMB_THETA];
    double phi = coeff[THUMB_PHI];
    double psi = coeff[THUMB_TWIST];
    mat = &nodes_mat_[index_bone_thumb_[0]];
    euler2RotMatGM(mat_tmp1, psi, theta, phi);
    Double4x4::mult(*mat, nodes_bone_rest_transform_[index_bone_thumb_[0]], mat_tmp1);

    theta = coeff[THUMB_K1_THETA];
    phi = coeff[THUMB_K1_PHI];
    mat = &nodes_mat_[index_bone_thumb_[1]];
    rotateMatZAxisGM(mat_tmp1, theta);
    rotateMatXAxisGM(mat_tmp2, phi);
    Double4x4::mult(mat_tmp3, mat_tmp1, mat_tmp2);

    const Double4x4& bone_mid = nodes_bone_rest_transform_[index_bone_thumb_[1]];
    mat_tmp2.set(bone_mid);
    Double4x4::mult(*mat, mat_tmp2, mat_tmp3);
    mat->rightMultScale(1.0, 1.0 + coeff[THUMB_LENGTH], 1.0);  // Scale this node

    phi = coeff[THUMB_K2_PHI];
    mat = &nodes_mat_[index_bone_thumb_[2]];
    rotateMatXAxisGM(mat_tmp1, phi);
    const Double4x4& bone_tip = nodes_bone_rest_transform_[index_bone_thumb_[2]];
    Double3 bone_tip_pos;
    Double4x4::getTranslation(bone_tip_pos, bone_tip);
    double bone_mid_length = bone_tip_pos.length();
    mat_tmp2.set(bone_tip);
    // Move bone by fraction of the bone length:
    mat_tmp2.leftMultTranslation(0, bone_mid_length * coeff[THUMB_LENGTH], 0);
    Double4x4::mult(*mat, mat_tmp2, mat_tmp1);
    mat->leftMultScale(1.0, 1.0 / (1.0 + coeff[THUMB_LENGTH]), 1.0);  // Undo parent scale
    mat->rightMultScale(1.0, 1.0 + coeff[THUMB_LENGTH], 1.0);  // Scale this node
  }

  Float4x4 tmp;
  void HandModel::updateHeirachyMatrices() {
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

  Double4x4 dtmp;
  void HandModel::updateDoubleHeirachyMatrices() {
    // Since the nodes array is bfs ordered, we just have to traverse it 
    // linearly.
    nodes_heirachy_mat_[0].set(nodes_mat_[0]);
    Double4x4::inverse(root_mat_hierachy_inv_, nodes_heirachy_mat_[0]);

    for (uint32_t i = 1; i < nodes_.size(); i++) {
      uint32_t iparent = nodes_parents_[i];

      Double4x4::mult(nodes_heirachy_mat_[i], nodes_heirachy_mat_[iparent], 
        nodes_mat_[i]);

      // Update the bone matrix
      if (nodes_[i]->bone() != NULL) {  
        // This node is a bone, calculate the local bone space matrix
        Double4x4* cur_bone_final_trans = &nodes_bone_transform_[i];
        Double4x4* bone_offset = &nodes_bone_offset_[i];

        Double4x4::mult(dtmp, nodes_heirachy_mat_[i], *bone_offset);
        Double4x4::mult(*cur_bone_final_trans, root_mat_hierachy_inv_, dtmp);
      }
    }
  }

  void HandModel::setRenderVisiblity(const bool visible) {
    if (visible_ != visible) {
      visible_ = visible;
      for (uint32_t i = 0; i < nodes_.size(); i++) {
        nodes_[i]->render() = visible_;
      }
    }
  }

  const bool HandModel::getRenderVisiblity() {
    return visible_;
  }

  // These next few methods are to avoid the cos and sin double functions in 
  // the Mat4x4 template
  // ********************************************************************
  // FLOAT HELPER FUNCTIONS
  void HandModel::euler2RotMatGM(Float4x4& a, const float x_angle, 
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

  void HandModel::rotateMatZAxisGM(Float4x4& ret, const float angle) {
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

  void HandModel::rotateMatYAxisGM(Float4x4& ret, const float angle) {
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

  void HandModel::rotateMatXAxisGM(Float4x4& ret, const float angle) {
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
  void HandModel::euler2RotMatGM(Double4x4& a, const double x_angle, 
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

  void HandModel::rotateMatZAxisGM(Double4x4& ret, const double angle) {
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

  void HandModel::rotateMatYAxisGM(Double4x4& ret, const double angle) {
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

  void HandModel::rotateMatXAxisGM(Double4x4& ret, const double angle) {
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

  // coeff_min_limit_ is the minimum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float HandModel::coeff_min_limit_[HAND_NUM_COEFF] = {
    -std::numeric_limits<float>::infinity(),    // HAND_POS_X
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    -3.14159f,  // HAND_ORIENT_X
    -3.14159f,  // HAND_ORIENT_Y
    -3.14159f,  // HAND_ORIENT_Z
    -0.903f,  // WRIST_THETA
    -1.580f,  // WRIST_PHI
    -0.523f,  // THUMB_THETA
    -0.523f,  // THUMB_PHI
    -0.633f,  // THUMB_K1_THETA
    -1.253f,  // THUMB_K1_PHI
    -1.733f,  // THUMB_K2_PHI
    -0.300f,  // F0_ROOT_THETA
    -0.300f,  // F0_ROOT_PHI
    -0.800f,  // F0_THETA
    -1.443f,  // F0_PHI
    -1.400f,  // F0_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F0_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F1_ROOT_THETA
    -0.300f,  // F1_ROOT_PHI
    -0.800f,  // F1_THETA
    -1.443f,  // F1_PHI
    -1.400f,  // F1_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F1_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F2_ROOT_THETA
    -0.300f,  // F2_ROOT_PHI
    -0.800f,  // F2_THETA
    -1.443f,  // F2_PHI
    -1.400f,  // F2_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F2_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F3_ROOT_THETA
    -0.300f,  // F3_ROOT_PHI
    -0.800f,  // F3_THETA
    -1.443f,  // F3_PHI
    -1.400f,  // F3_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F3_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F0_TWIST
    -0.400f,  // F1_TWIST
    -0.300f,  // F2_TWIST
    -0.300f,  // F3_TWIST
    -0.300f,  // THUMB_TWIST
  };

  // coeff_min_limit_ is the minimum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float HandModel::coeff_min_limit_conservative_[HAND_NUM_COEFF] = {
    -std::numeric_limits<float>::infinity(),    // HAND_POS_X
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    -std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    -3.14159f,  // HAND_ORIENT_X
    -3.14159f,  // HAND_ORIENT_Y
    -3.14159f,  // HAND_ORIENT_Z
    -0.903f,  // WRIST_THETA
    -1.580f,  // WRIST_PHI
    -0.523f,  // THUMB_THETA
    -0.523f,  // THUMB_PHI
    -0.170f,  // THUMB_K1_THETA
    -1.253f,  // THUMB_K1_PHI
    -1.733f,  // THUMB_K2_PHI
    -0.300f,  // F0_ROOT_THETA
    -0.300f,  // F0_ROOT_PHI
    -0.800f,  // F0_THETA
    -1.443f,  // F0_PHI
    -1.400f,  // F0_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F0_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F1_ROOT_THETA
    -0.300f,  // F1_ROOT_PHI
    -0.800f,  // F1_THETA
    -1.443f,  // F1_PHI
    -1.400f,  // F1_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F1_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F2_ROOT_THETA
    -0.300f,  // F2_ROOT_PHI
    -0.800f,  // F2_THETA
    -1.443f,  // F2_PHI
    -1.400f,  // F2_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F2_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F3_ROOT_THETA
    -0.300f,  // F3_ROOT_PHI
    -0.800f,  // F3_THETA
    -1.443f,  // F3_PHI
    -1.400f,  // F3_KNUCKLE_MID  // Formally -1.363 4/12/2013
    -1.500f,  // F3_KNUCKLE_END  // Formally -1.363 4/12/2013
    -0.300f,  // F0_TWIST
    -0.400f,  // F1_TWIST
    -0.300f,  // F2_TWIST
    -0.300f,  // F3_TWIST
    -0.300f,  // THUMB_TWIST
  };
  
  // coeff_max_limit_ is the maximum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float HandModel::coeff_max_limit_[HAND_NUM_COEFF] = {
    std::numeric_limits<float>::infinity(),    // HAND_POS_X
    std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    3.14159f,  // HAND_ORIENT_X
    3.14159f,  // HAND_ORIENT_Y
    3.14159f,  // HAND_ORIENT_Z
    0.905f,  // WRIST_THETA
    1.580f,  // WRIST_PHI
    0.550f,  // THUMB_THETA
    0.580f,  // THUMB_PHI
    0.700f,  // THUMB_K1_THETA
    0.750f,  // THUMB_K1_PHI
    0.500f,  // THUMB_K2_PHI
    0.300f,  // F0_ROOT_THETA
    0.300f,  // F0_ROOT_PHI
    0.600f,  // F0_THETA
    0.670f,  // F0_PHI
    0.560f,  // F0_KNUCKLE_MID
    0.560f,  // F0_KNUCKLE_END
    0.300f,  // F1_ROOT_THETA
    0.300f,  // F1_ROOT_PHI
    0.600f,  // F1_THETA
    0.670f,  // F1_PHI
    0.560f,  // F1_KNUCKLE_MID
    0.560f,  // F1_KNUCKLE_END
    0.300f,  // F2_ROOT_THETA
    0.300f,  // F2_ROOT_PHI
    0.600f,  // F2_THETA
    0.670f,  // F2_PHI
    0.560f,  // F2_KNUCKLE_MID
    0.560f,  // F2_KNUCKLE_END
    0.300f,  // F3_ROOT_THETA
    0.300f,  // F3_ROOT_PHI
    0.600f,  // F3_THETA
    0.670f,  // F3_PHI
    0.560f,  // F3_KNUCKLE_MID
    0.560f,  // F3_KNUCKLE_END
    0.300f,  // F0_TWIST
    0.300f,  // F1_TWIST
    0.300f,  // F2_TWIST
    0.300f,  // F3_TWIST
    0.300f,  // THUMB_TWIST
  };

  // coeff_max_limit_ is the maximum coefficient value before the penalty
  // function heavily penalizes configurations with this value
  const float HandModel::coeff_max_limit_conservative_[HAND_NUM_COEFF] = {
    std::numeric_limits<float>::infinity(),    // HAND_POS_X
    std::numeric_limits<float>::infinity(),    // HAND_POS_Y
    std::numeric_limits<float>::infinity(),    // HAND_POS_Z
    3.14159f,  // HAND_ORIENT_X
    3.14159f,  // HAND_ORIENT_Y
    3.14159f,  // HAND_ORIENT_Z
    0.905f,  // WRIST_THETA
    1.580f,  // WRIST_PHI
    0.350f,  // THUMB_THETA
    0.300f,  // THUMB_PHI
    0.700f,  // THUMB_K1_THETA
    0.550f,  // THUMB_K1_PHI
    0.300f,  // THUMB_K2_PHI
    0.100f,  // F0_ROOT_THETA
    0.100f,  // F0_ROOT_PHI
    0.600f,  // F0_THETA
    0.270f,  // F0_PHI
    0.360f,  // F0_KNUCKLE_MID
    0.360f,  // F0_KNUCKLE_END
    0.300f,  // F1_ROOT_THETA
    0.300f,  // F1_ROOT_PHI
    0.600f,  // F1_THETA
    0.270f,  // F1_PHI
    0.360f,  // F1_KNUCKLE_MID
    0.360f,  // F1_KNUCKLE_END
    0.100f,  // F2_ROOT_THETA
    0.100f,  // F2_ROOT_PHI
    0.600f,  // F2_THETA
    0.270f,  // F2_PHI
    0.360f,  // F2_KNUCKLE_MID
    0.360f,  // F2_KNUCKLE_END
    0.300f,  // F3_ROOT_THETA
    0.300f,  // F3_ROOT_PHI
    0.600f,  // F3_THETA
    0.270f,  // F3_PHI
    0.360f,  // F3_KNUCKLE_MID
    0.360f,  // F3_KNUCKLE_END
    0.300f,  // F0_TWIST
    0.300f,  // F1_TWIST
    0.300f,  // F2_TWIST
    0.300f,  // F3_TWIST
    0.300f,  // THUMB_TWIST
  };
  
  // coeff_penalty_scale_ is the exponential scale to use when penalizing coeffs
  // outside the min and max values.
  const float HandModel::coeff_penalty_scale_[HAND_NUM_COEFF] = {
    0,    // HAND_POS_X
    0,    // HAND_POS_Y
    0,    // HAND_POS_Z
    0,  // HAND_ORIENT_X
    0,  // HAND_ORIENT_Y
    0,  // HAND_ORIENT_Z
    100,  // WRIST_THETA
    100,  // WRIST_PHI
    100,  // THUMB_THETA
    100,  // THUMB_PHI
    100,  // THUMB_K1_THETA
    100,  // THUMB_K1_PHI
    100,  // THUMB_K2_PHI
    100,  // F0_ROOT_THETA
    100,  // F0_ROOT_PHI
    100,  // F0_THETA
    100,  // F0_PHI
    100,  // F0_KNUCKLE_MID
    100,  // F0_KNUCKLE_END
    100,  // F1_ROOT_THETA
    100,  // F1_ROOT_PHI
    100,  // F1_THETA
    100,  // F1_PHI
    100,  // F1_KNUCKLE_MID
    100,  // F1_KNUCKLE_END
    100,  // F2_ROOT_THETA
    100,  // F2_ROOT_PHI
    100,  // F2_THETA
    100,  // F2_PHI
    100,  // F2_KNUCKLE_MID
    100,  // F2_KNUCKLE_END
    100,  // F3_ROOT_THETA
    100,  // F3_ROOT_PHI
    100,  // F3_THETA
    100,  // F3_PHI
    100,  // F3_KNUCKLE_MID
    100,  // F3_KNUCKLE_END
    100,  // F0_TWIST
    100,  // F1_TWIST
    100,  // F2_TWIST
    100,  // F3_TWIST
    100,  // THUMB_TWIST
  };

// angle_coeffs are boolean values indicating if the coefficient represents
  // a pure angle (0 --> 2pi)
  const bool HandModel::angle_coeffs_[HAND_NUM_COEFF] = {
    // Hand 1
    false,  // HAND_POS_X
    false,  // HAND_POS_Y
    false,  // HAND_POS_Z
    true,  // HAND_ORIENT_X
    true,  // HAND_ORIENT_Y
    true,  // HAND_ORIENT_Z
    true,   // WRIST_THETA
    true,   // WRIST_PHI
    true,   // THUMB_THETA
    true,   // THUMB_PHI
    true,   // THUMB_K1_THETA
    true,   // THUMB_K1_PHI
    true,   // THUMB_K2_PHI
    true,   // F0_ROOT_THETA
    true,   // F0_ROOT_PHI
    true,   // F0_THETA
    true,   // F0_PHI
    true,   // F0_KNUCKLE_MID
    true,   // F0_KNUCKLE_END
    true,   // F1_ROOT_THETA
    true,   // F1_ROOT_PHI
    true,   // F1_THETA
    true,   // F1_PHI
    true,   // F1_KNUCKLE_MID
    true,   // F1_KNUCKLE_END
    true,   // F2_ROOT_THETA
    true,   // F2_ROOT_PHI
    true,   // F2_THETA
    true,   // F2_PHI
    true,   // F2_KNUCKLE_MID
    true,   // F2_KNUCKLE_END
    true,   // F3_ROOT_THETA
    true,   // F3_ROOT_PHI
    true,   // F3_THETA
    true,   // F3_PHI
    true,   // F3_KNUCKLE_MID
    true,   // F3_KNUCKLE_END
    true,   // F0_TWIST
    true,   // F1_TWIST
    true,   // F2_TWIST
    true,   // F3_TWIST
    true,   // THUMB_TWIST
  };

  void HandModel::calcBoundingSphereUVDPos(float* uvd, 
    const uint32_t b_sphere_index, const Float4x4& pv_mat) {
    BSphere* sphere = bspheres_[b_sphere_index];

    Float4x4::multSIMD(tmp, nodes_[index_mesh_node_]->mat_hierarchy(), 
      *sphere->parent_node()->bone_transform());

    Float3 xyz_pos;
    Float3::affineTransformPos(xyz_pos, tmp, sphere->center());
    
    Float4 pos(xyz_pos[0], xyz_pos[1], xyz_pos[2], 1.0f);
    Float4 homog_pos;
    Float4::mult(homog_pos, pv_mat, pos);
    uvd[0] = (homog_pos[0] / (homog_pos[3] + LOOSE_EPSILON));  // NDC X: -1 --> 1
    uvd[1] = (homog_pos[1] / (homog_pos[3] + LOOSE_EPSILON));  // NDC Y: -1 --> 1
    // http://www.songho.ca/opengl/gl_transform.html
    uvd[0] = (float)src_width * 0.5f * (-uvd[0] + 1);  // Window X: 0 --> W
    uvd[1] = (float)src_height * 0.5f * (uvd[1] + 1);  // Window Y: 0 --> H
    uvd[2] = xyz_pos[2];
  }

  void HandModel::calcBoundingSphereUVDPos(double* uvd, 
    const uint32_t b_sphere_index, const Double4x4& pv_mat) {
    BSphere* sphere = bspheres_[b_sphere_index];

    Double4x4::mult(dtmp, nodes_heirachy_mat_[index_mesh_node_], 
      nodes_bone_transform_[bsphere_parent_ind_[b_sphere_index]]);

    Double3 xyz_pos;
    Double3 center;
    center[0] = (double)sphere->center()[0];
    center[1] = (double)sphere->center()[1];
    center[2] = (double)sphere->center()[2];
    Double3::affineTransformPos(xyz_pos, dtmp, center);
    
    Double4 pos(xyz_pos[0], xyz_pos[1], xyz_pos[2], 1.0);
    Double4 homog_pos;
    Double4::mult(homog_pos, pv_mat, pos);
    uvd[0] = (homog_pos[0] / (homog_pos[3] + (double)EPSILON));  // NDC X: -1 --> 1
    uvd[1] = (homog_pos[1] / (homog_pos[3] + (double)EPSILON));  // NDC Y: -1 --> 1
    // http://www.songho.ca/opengl/gl_transform.html
    uvd[0] = (double)src_width * 0.5 * (-uvd[0] + 1);  // Window X: 0 --> W
    uvd[1] = (double)src_height * 0.5 * (uvd[1] + 1);  // Window Y: 0 --> H
    uvd[2] = xyz_pos[2];
  }

}  // namespace hand_net
}  // namespace kinect_interface