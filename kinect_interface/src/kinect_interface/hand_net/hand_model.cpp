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
#include "kinect_interface/hand_net/hand_net.h"  // for HandCoeffConvnet
#include "kinect_interface/hand_net/hand_image_generator.h"  // for HN_HAND_SIZE

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }

#if defined(__APPLE__)
  #define HAND_MODEL_PATH string("./../../../../../../../../../models/lib_hand/")
#else
  #define HAND_MODEL_PATH string("./models/lib_hand/")
#endif

//#define LOAD_HAND_MESH_JFILE  // Much faster and more compact format!
#define LHAND_MODEL_FILE "hand_palm_parent_medium_wrist_dec_0.05.dae"
#define LHAND_MODEL_JFILE "hand_palm_parent_medium_wrist_dec_0.05.jbin"
#define RHAND_MODEL_FILE "hand_palm_parent_medium_wrist_dec_0.05_right.dae"
#define RHAND_MODEL_JFILE "hand_palm_parent_medium_wrist_dec_0.05_right.jbin"

using namespace jtil::renderer::objects;
using namespace jtil::renderer;
using namespace jtil::math;
using std::string;

namespace kinect_interface {
namespace hand_net {
  
  HandModel::HandModel(HandType hand_type) {
    loadHandGeometry(hand_type);

    /*
    // HACK: The model's normals are inside out --> Fix them
    if (hand_type == HandType::RIGHT) {
      if (mesh_->type() == GeometryType::GEOMETRY_TEXT_BONED_MESH) {
        Geometry* m = mesh_->geom();
        m->unsyncVAO();
        jtil::data_str::Vector<jtil::math::Float3>* norms = m->normals();
        for (uint32_t i = 0; i < norms->size(); i++) {
          Float3::scale((*norms)[i], -1.0f);
        }
        m->syncVAO();
      }
    }
    */

    renderer_attachment_ = true;  }

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

    // Find all the bones
    string base_name;
    if (type == HandType::LEFT) {
      base_name = HAND_MODEL_PATH + LHAND_MODEL_FILE + "/";
    } else {
      base_name = HAND_MODEL_PATH + RHAND_MODEL_FILE + "/";
    }
    bone_wrist_ = GeometryManager::findGeometryInstanceByName(
      base_name + "carpals", model_);
    if (bone_wrist_ == NULL) {
      throw std::wruntime_error("ERROR: couldn't find the wrist bone!");
    }
    bone_palm_ = GeometryManager::findGeometryInstanceByName(
      base_name + "metacarpals", model_);
    if (bone_palm_ == NULL) {
      throw std::wruntime_error("ERROR: couldn't find the palm bone!");
    }
    // Get the 3 joints of the thumb
    for (uint32_t i = 0; i < 3; i++) {
      std::stringstream ss;
      ss << base_name << "finger5joint" << (i + 1);
      bone_thumb_[i] = GeometryManager::findGeometryInstanceByName(ss.str(), 
        model_);
      if (bone_thumb_[i] == NULL) {
        throw std::wruntime_error("ERROR: couldn't find a finger1 bone!");
      }
    }
    // Get the 1st joint of each finger
    for (uint32_t i = 0; i < 4; i++) {
      std::stringstream ss;
      if (i == 0) {
        ss << base_name << "Bone";
      } else {
        ss << base_name << "Bone.00" << i;
      }
      bone_finger1_[i] = GeometryManager::findGeometryInstanceByName(ss.str(), 
        model_);
      if (bone_finger1_[i] == NULL) {
        throw std::wruntime_error("ERROR: couldn't find a finger1 bone!");
      }
    }
    // Get the 2nd joint of each finger
    for (uint32_t i = 0; i < 4; i++) {
      std::stringstream ss;
      ss << base_name << "finger" << (i+1) << "joint1";
      bone_finger2_[i] = GeometryManager::findGeometryInstanceByName(ss.str(), 
        model_);
      if (bone_finger2_[i] == NULL) {
        throw std::wruntime_error("ERROR: couldn't find a finger2 bone!");
      }
    }
    // Get the 3rd joint of each finger
    for (uint32_t i = 0; i < 4; i++) {
      std::stringstream ss;
      ss << base_name << "finger" << (i+1) << "joint2";
      bone_finger3_[i] = GeometryManager::findGeometryInstanceByName(ss.str(), 
        model_);
      if (bone_finger3_[i] == NULL) {
        throw std::wruntime_error("ERROR: couldn't find a finger3 bone!");
      }
    }
    // Get the 4th joint of each finger
    for (uint32_t i = 0; i < 4; i++) {
      std::stringstream ss;
      ss << base_name << "finger" << (i+1) << "joint3";
      bone_finger4_[i] = GeometryManager::findGeometryInstanceByName(ss.str(), 
        model_);
      if (bone_finger4_[i] == NULL) {
        throw std::wruntime_error("ERROR: couldn't find a finger4 bone!");
      }
    }

    // For all meshes in the heirachy set the specular intensity low:
    renderStackReset(); 
    while (!renderStackEmpty()) {
      GeometryInstance* geom = renderStackPop();
      geom->mtrl().spec_intensity = 0.15f;
    }

    // Now create bounding spheres:
    bspheres_.capacity(HandSphereIndices::NUM_BOUNDING_SPHERES);
    bspheres_.resize(HandSphereIndices::NUM_BOUNDING_SPHERES);
    for (uint32_t i = 0; i < 4; i++) {
      addBoneBSphere((uint32_t)F1_KNU3_A + i * num_bspheres_per_group(), 
        bone_finger4_[i]);
      addBoneBSphere((uint32_t)F1_KNU3_B + i * num_bspheres_per_group(), 
        bone_finger4_[i]);
      addBoneBSphere((uint32_t)F1_KNU2_A + i * num_bspheres_per_group(), 
        bone_finger3_[i]);
      addBoneBSphere((uint32_t)F1_KNU2_B + i * num_bspheres_per_group(), 
        bone_finger3_[i]);
      addBoneBSphere((uint32_t)F1_KNU1_A + i * num_bspheres_per_group(), 
        bone_finger2_[i]);
      addBoneBSphere((uint32_t)F1_KNU1_B + i * num_bspheres_per_group(), 
        bone_finger2_[i]);
    }

    addBoneBSphere((uint32_t)TH_KNU3_A, bone_thumb_[2]);
    addBoneBSphere((uint32_t)TH_KNU3_B, bone_thumb_[2]);
    addBoneBSphere((uint32_t)TH_KNU2_A, bone_thumb_[1]);
    addBoneBSphere((uint32_t)TH_KNU2_B, bone_thumb_[1]);
    addBoneBSphere((uint32_t)TH_KNU1_A, bone_thumb_[0]);
    addBoneBSphere((uint32_t)TH_KNU1_B, bone_thumb_[0]);
    
    addBoneBSphere((uint32_t)PALM_1, bone_palm_);
    addBoneBSphere((uint32_t)PALM_2, bone_palm_);
    addBoneBSphere((uint32_t)PALM_3, bone_palm_);
    addBoneBSphere((uint32_t)PALM_4, bone_palm_);
    addBoneBSphere((uint32_t)PALM_5, bone_palm_);
    addBoneBSphere((uint32_t)PALM_6, bone_palm_);
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
  }

  void HandModel::updateMatrices(const float* coeff) {
    // Set the root matrix:
    Float4x4* mat = &model_->mat();
    euler2RotMatGM(*mat, coeff[HAND_ORIENT_X], coeff[HAND_ORIENT_Y],
      coeff[HAND_ORIENT_Z]);
    mat->leftMultTranslation(coeff[HAND_POS_X], coeff[HAND_POS_Y],
      coeff[HAND_POS_Z]);
    mat->rightMultScale(coeff[SCALE], coeff[SCALE], coeff[SCALE]); 

 
    // Set the palm bone (depending on wrist angle)
    mat = &bone_wrist_->mat();
    rotateMatXAxisGM(mat_tmp1, coeff[WRIST_PHI]);
    rotateMatZAxisGM(mat_tmp2, coeff[WRIST_THETA]);
    Float4x4::mult(mat_tmp3, mat_tmp1, mat_tmp2);
    // rotateMatXAxis(mat_tmp1, HandModelCoeff::wrist_twist);
    // Float4x4::mult(mat_tmp2, mat_tmp1, mat_tmp3);
    Float4x4::mult(*mat, bone_wrist_->bone()->rest_transform, mat_tmp3);

    // Set the finger bones
    for (uint32_t i = 0; i < 4; i++) {
      float theta, phi, psi;
      // Root
      theta = coeff[F0_ROOT_THETA + i * FINGER_NUM_COEFF];
      phi = coeff[F0_ROOT_PHI + i * FINGER_NUM_COEFF];
      psi = 0;
      mat = &bone_finger1_[i]->mat();
      euler2RotMatGM(mat_tmp3, psi, theta, phi);
      Float4x4::multSIMD(*mat, bone_finger1_[i]->bone()->rest_transform,
        mat_tmp3);

      // K1 base
      theta = coeff[F0_THETA + i * FINGER_NUM_COEFF];
      phi = coeff[F0_PHI + i * FINGER_NUM_COEFF];
      psi = coeff[F0_TWIST + i];
      mat = &bone_finger2_[i]->mat();
      euler2RotMatGM(mat_tmp3, psi, theta, phi);
      Float4x4::multSIMD(*mat, bone_finger2_[i]->bone()->rest_transform, 
        mat_tmp3);
      mat->rightMultScale(1.0f, 1.0f + coeff[F0_LENGTH + i], 1.0f);  // Scale this node

      mat = &bone_finger3_[i]->mat();
      float k2_theta = coeff[F0_KNUCKLE_MID + i * FINGER_NUM_COEFF];
      rotateMatXAxisGM(mat_tmp1, k2_theta);
      const Float4x4& bone_mid = bone_finger3_[i]->bone()->rest_transform;
      Float3 bone_mid_pos;
      Float4x4::getTranslation(bone_mid_pos, bone_mid);
      float bone_base_length = bone_mid_pos.length();
      mat_tmp2.set(bone_mid);
      // Move bone by fraction of the bone length:
      mat_tmp2.leftMultTranslation(0, bone_base_length * coeff[F0_LENGTH + i], 0);  
      Float4x4::multSIMD(*mat, mat_tmp2, mat_tmp1);
      mat->leftMultScale(1.0f, 1.0f / (1.0f + coeff[F0_LENGTH + i]), 1.0f);  // Undo parent scale
      mat->rightMultScale(1.0f, 1.0f + coeff[F0_LENGTH + i], 1.0f);  // Scale this node

      mat = &bone_finger4_[i]->mat();
      float k3_theta = coeff[F0_KNUCKLE_END + i * FINGER_NUM_COEFF];
      rotateMatXAxisGM(mat_tmp1, k3_theta);
      const Float4x4& bone_tip = bone_finger4_[i]->bone()->rest_transform;
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
    mat = &bone_thumb_[0]->mat();
    euler2RotMatGM(mat_tmp3, psi, theta, phi);
    Float4x4::multSIMD(*mat, bone_thumb_[0]->bone()->rest_transform, mat_tmp3);

    theta = coeff[THUMB_K1_THETA];
    phi = coeff[THUMB_K1_PHI];
    mat = &bone_thumb_[1]->mat();
    rotateMatZAxisGM(mat_tmp1, theta);
    rotateMatXAxisGM(mat_tmp2, phi);
    Float4x4::multSIMD(mat_tmp3, mat_tmp1, mat_tmp2);

    const Float4x4& bone_mid = bone_thumb_[1]->bone()->rest_transform;
    mat_tmp2.set(bone_mid);
    Float4x4::multSIMD(*mat, mat_tmp2, mat_tmp3);
    mat->rightMultScale(1.0f, 1.0f + coeff[THUMB_LENGTH], 1.0f);  // Scale this node

    phi = coeff[THUMB_K2_PHI];
    mat = &bone_thumb_[2]->mat();
    rotateMatXAxisGM(mat_tmp1, phi);
    const Float4x4& bone_tip = bone_thumb_[2]->bone()->rest_transform;
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
  
  void HandModel::renderStackReset() {
    render_stack_.resize(0);  // empty the stack (without deallocating)
    // Seed the render stack with the root node
    render_stack_.pushBack(model_);
  }

  GeometryInstance* HandModel::renderStackPop() {
    GeometryInstance* ret = NULL;
    if (render_stack_.size() > 0) {
      render_stack_.popBackUnsafe(ret);  // Remove the last element

      // Now add the children to the geometry stack
      for (uint32_t i = 0; i < ret->numChildren(); i ++) {
        render_stack_.pushBack(ret->getChild(i));
      }
    }
    return ret;
  }

  bool HandModel::renderStackEmpty() {
    return render_stack_.size() == 0;
  }

  void HandModel::updateHeirachyMatrices() {
    renderStackReset();
    Float4x4 tmp;
    while (!renderStackEmpty()) {
      GeometryInstance* cur_geom = renderStackPop();
      // Update the render matrix based on our parents position
      if (cur_geom->parent() != NULL) {
        Float4x4::multSIMD(cur_geom->mat_hierarchy(),
          cur_geom->parent()->mat_hierarchy(), cur_geom->mat());

      } else {
        cur_geom->mat_hierarchy().set(cur_geom->mat());
      }

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

  void HandModel::fixBoundingSphereMatrices() {
    /*
    Float4x4 tmp;
    Float4x4 root_inverse;
    Float4x4::inverse(root_inverse, *scene_graph_->mat());
    renderStackReset();
    while (!renderStackEmpty()) {
      Geometry* cur_geom = renderStackPop();
      if (cur_geom->type() == GeometryType::BOUNDING_SPHERE) {
        // This is a bit of a hack, but we have to undo the parent's root
        // transform (by left multiplying by its inverse).
        // Then we have to left multiply by the mesh node's transform
        BoundingSphere* sphere = reinterpret_cast<BoundingSphere*>(cur_geom);
        Float4x4::multSIMD(tmp, root_inverse, *sphere->mat_hierarchy());
        Float4x4::multSIMD(*sphere->mat_hierarchy(), *sphere->mesh_node()->mat_hierarchy(), tmp);
      }
    }
    */
  }

  void HandModel::setRendererAttachement(const bool renderer_attachment) {
    if (renderer_attachment_ != renderer_attachment) {
      renderer_attachment_ = renderer_attachment;
      renderStackReset();
      while (!renderStackEmpty()) {
        GeometryInstance* cur_geom = renderStackPop();
        cur_geom->render() = renderer_attachment_;
      }
    }
  }

  const bool HandModel::getRendererAttachement() {
    return renderer_attachment_;
  }

  // These next few methods are to avoid the cos and sin double functions in 
  // the Mat4x4 template
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

}  // namespace hand_net
}  // namespace kinect_interface