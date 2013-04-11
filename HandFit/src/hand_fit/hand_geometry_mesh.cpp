#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include "hand_fit/hand_renderer.h"
#include "hand_fit/hand_geometry_mesh.h"
#include "hand_fit/bounding_sphere.h"
#include "renderer/open_gl_common.h"
#include "renderer/renderer.h"
#include "renderer/colors.h"
#include "renderer/geometry/geometry_manager.h"
#include "renderer/geometry/geometry.h"
#include "renderer/geometry/geometry_textured_boned_mesh.h"
#include "renderer/geometry/geometry_vertices.h"
#include "renderer/geometry/geometry_colored_mesh.h"
#include "Eigen"
#include "renderer/geometry/bone_info.h"
#include "jtil/data_str/pair.h"

#define SAFE_DELETE(target) \
  if (target != NULL) { \
    delete target; \
    target = NULL; \
  }

#ifndef HAND_FIT
#error "HAND_FIT is not defined!  You need to declare it in the preprocessor"
#endif

using Eigen::Matrix;
using Eigen::MatrixXf;
using jtil::math::Float4x4;
using jtil::math::FloatQuat;
using jtil::math::Float3;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using jtil::math::Float3;
using renderer::Camera;
using renderer::Geometry;
using renderer::GeometryType;
using renderer::GeometryTexturedBonedMesh;
using renderer::Renderer;
using renderer::GeometryManager;
using renderer::Bone;
using renderer::BoneFileInfo;
using renderer::BoundingSphere;
using namespace kinect_interface::hand_net;

namespace hand_fit {
  
  HandGeometryMesh::HandGeometryMesh(HandType hand_type, 
    Renderer* g_renderer, HandRenderer* g_hand_renderer) : 
    HandGeometry(hand_type) {
    // Set all the bones to undefined:
    bone_wrist_index_ = MAX_UINT32;
    bone_palm_index_ = MAX_UINT32;
    bone_thumb_1_index_ = MAX_UINT32;
    bone_thumb_2_index_ = MAX_UINT32;
    bone_thumb_3_index_ = MAX_UINT32;
    for (uint32_t i = 0; i < 4; i++) {
      bone_finger_1_index_[i] = MAX_UINT32;
      bone_finger_2_index_[i] = MAX_UINT32;
      bone_finger_3_index_[i] = MAX_UINT32;
      bone_finger_4_index_[i] = MAX_UINT32;
    }

    createHandGeometry(g_renderer, g_hand_renderer, hand_type);

    // HACK: The model's normals are inside out --> Fix them
    if (hand_type == HandType::RIGHT) {
      if (mesh_->type() == GeometryType::GEOMETRY_TEXTURED_BONED_MESH) {
        GeometryTexturedBonedMesh* m = (GeometryTexturedBonedMesh*)mesh_;
        m->unsyncVAO();
        jtil::data_str::Vector<jtil::math::Float3>* norms = m->normals();
        for (uint32_t i = 0; i < norms->size(); i++) {
          Float3::scale((*norms)[i], -1.0f);
        }
        m->syncVAO();
      }
    }

    // Adding the geometry to the gobal geometry manager's scene graph will
    // transfer ownership of the memory.
    GeometryManager::scene_graph_root()->addChild(scene_graph_);
  }

  HandGeometryMesh::~HandGeometryMesh() {
    // Note, ownership of all geometry is transfered to the renderer class
  }

  uint32_t HandGeometryMesh::sph_bone_ind_[NUM_BOUNDING_SPHERES];

  void HandGeometryMesh::createHandGeometry(Renderer* g_renderer, 
    HandRenderer* g_hand_renderer, HandType type) {
    GeometryManager* gm = GeometryManager::g_geom_manager();
#ifndef LOAD_HAND_MESH_JFILE
    if (type == HandType::LEFT) {
      scene_graph_ = gm->loadFromFile(HAND_MODEL_PATH, LHAND_MODEL_FILE);
      gm->saveToJFile(scene_graph_, HAND_MODEL_PATH, LHAND_MODEL_JFILE);
    } else {
      scene_graph_ = gm->loadFromFile(HAND_MODEL_PATH, RHAND_MODEL_FILE);
      gm->saveToJFile(scene_graph_, HAND_MODEL_PATH, RHAND_MODEL_JFILE);
    }
#else
    if (type == HandType::LEFT) {
      scene_graph_ = gm->loadFromJFile(HAND_MODEL_PATH, LHAND_MODEL_JFILE);
    } else {
      scene_graph_ = gm->loadFromJFile(HAND_MODEL_PATH, RHAND_MODEL_JFILE);
    }
#endif

    // Find all the bone nodes so we can quickly access them later:
    if (type == HandType::LEFT) {
      bones_in_file_ = gm->findBoneFileInfo(LHAND_MODEL_FILE);
    } else {
      bones_in_file_ = gm->findBoneFileInfo(RHAND_MODEL_FILE);
    }
    if (bones_in_file_ == NULL) {
      throw runtime_error(string("HandGeometryMesh::createHandGeometry()")+
        string(" - ERROR: Could not find hand model bone data!"));
    }

    // 2. Now find the bone indices that we care about:
    for (uint32_t i = 0; i < bones_in_file_->bones.size(); i++) {
      Bone* cur_bone = bones_in_file_->bones[i];
      if (cur_bone->name == "carpals") {
        bone_wrist_index_ = i;
      } else if (cur_bone->name == "metacarpals") {
        bone_palm_index_ = i;
      } else if (cur_bone->name == "finger5joint1") {
        bone_thumb_1_index_ = i;
      } else if (cur_bone->name == "finger5joint2") {
        bone_thumb_2_index_ = i;
      } else if (cur_bone->name == "finger5joint3") {
        bone_thumb_3_index_ = i;
      } else if (cur_bone->name == "Bone") {
        bone_finger_1_index_[0] = i;
      } else if (cur_bone->name == "finger1joint1") {
        bone_finger_2_index_[0] = i;
      } else if (cur_bone->name == "finger1joint2") {
        bone_finger_3_index_[0] = i;
      } else if (cur_bone->name == "finger1joint3") {
        bone_finger_4_index_[0] = i;
      } else if (cur_bone->name == "Bone.001") {
        bone_finger_1_index_[1] = i;
      } else if (cur_bone->name == "finger2joint1") {
        bone_finger_2_index_[1] = i;
      } else if (cur_bone->name == "finger2joint2") {
        bone_finger_3_index_[1] = i;
      } else if (cur_bone->name == "finger2joint3") {
        bone_finger_4_index_[1] = i;
      } else if (cur_bone->name == "Bone.002") {
        bone_finger_1_index_[2] = i;
      } else if (cur_bone->name == "finger3joint1") {
        bone_finger_2_index_[2] = i;
      } else if (cur_bone->name == "finger3joint2") {
        bone_finger_3_index_[2] = i;
      } else if (cur_bone->name == "finger3joint3") {
        bone_finger_4_index_[2] = i;
      } else if (cur_bone->name == "Bone.003") {
        bone_finger_1_index_[3] = i;
      } else if (cur_bone->name == "finger4joint1") {
        bone_finger_2_index_[3] = i;
      } else if (cur_bone->name == "finger4joint2") {
        bone_finger_3_index_[3] = i;
      } else if (cur_bone->name == "finger4joint3") {
        bone_finger_4_index_[3] = i;
      } else {
        cout << "WARNING: couldn't associate bone: " << cur_bone->name;
      }
    }

    // Check that we found all the bones we were looking for.
    bool bones_ok = true;
    for (uint32_t i = 0; i < 4; i++) {
      bones_ok = bones_ok && (bone_finger_1_index_[i] != MAX_UINT32);
      bones_ok = bones_ok && (bone_finger_2_index_[i] != MAX_UINT32);
      bones_ok = bones_ok && (bone_finger_3_index_[i] != MAX_UINT32);
      bones_ok = bones_ok && (bone_finger_4_index_[i] != MAX_UINT32);
    }
    if (bone_wrist_index_ == MAX_UINT32 || bone_palm_index_ == MAX_UINT32 ||
       bone_thumb_1_index_ == MAX_UINT32 || bone_thumb_2_index_ == MAX_UINT32 ||
       bone_thumb_3_index_ == MAX_UINT32 || bone_palm_index_ == MAX_UINT32 ||
       !bones_ok) {
      throw runtime_error(string("HandGeometryMesh::createHandGeometry()") +
        string(" - ERROR: couldn't find one of the bones!"));
    }

    // Record the association between bounding spheres and bone indices
    for (uint32_t i = 0; i < 4; i++) {
      sph_bone_ind_[F1_KNU3_A + i * NSPH_PER_FING] = bone_finger_4_index_[i];
      sph_bone_ind_[F1_KNU3_B + i * NSPH_PER_FING] = bone_finger_4_index_[i];
      sph_bone_ind_[F1_KNU2_A + i * NSPH_PER_FING] = bone_finger_3_index_[i];
      sph_bone_ind_[F1_KNU2_B + i * NSPH_PER_FING] = bone_finger_3_index_[i];
      sph_bone_ind_[F1_KNU1_A + i * NSPH_PER_FING] = bone_finger_2_index_[i];
      sph_bone_ind_[F1_KNU1_B + i * NSPH_PER_FING] = bone_finger_2_index_[i];
    }
    sph_bone_ind_[TH_KNU3_A] = bone_thumb_3_index_;
    sph_bone_ind_[TH_KNU3_B] = bone_thumb_3_index_;
    sph_bone_ind_[TH_KNU2_A] = bone_thumb_2_index_;
    sph_bone_ind_[TH_KNU2_B] = bone_thumb_2_index_;
    sph_bone_ind_[TH_KNU1_A] = bone_thumb_1_index_;
    sph_bone_ind_[TH_KNU1_B] = bone_thumb_1_index_;

    sph_bone_ind_[PALM_1] = bone_palm_index_;
    sph_bone_ind_[PALM_2] = bone_palm_index_;
    sph_bone_ind_[PALM_3] = bone_palm_index_; 
    sph_bone_ind_[PALM_4] = bone_palm_index_;
    sph_bone_ind_[PALM_5] = bone_palm_index_;
    sph_bone_ind_[PALM_6] = bone_palm_index_;

    // For all meshes in the heirachy set the specular intensity low:
    renderStackReset(); 
    while (!renderStackEmpty()) {
      Geometry* geom = renderStackPop();
      geom->mtrl()->specular_intensity = 0.15f;
    }

    // Now save the rest transforms for each bone:
    rest_transforms_.capacity(bones_in_file_->bones.size());
    rest_transforms_.resize(bones_in_file_->bones.size());
    for (uint32_t i = 0; i < bones_in_file_->bones.size(); i++) {
      rest_transforms_[i].set(*bones_in_file_->bones[i]->getNode()->mat());
    }

    string mesh_name = string("hand_mesh");
    mesh_ = gm->findGeometryByName(scene_graph_, mesh_name);
    if (mesh_ == NULL) {
      throw std::runtime_error("ERROR: Couldn't find hand_mesh geometry!");
    }

    // Now place static bounding spheres to bone transforms:
    Float4x4 bone_offset_mat;
    Float3 center;
    Float3 origin(0, 0, 0);
    BoundingSphere* cur_sphere;
    Float4x4 temp;
    for (uint32_t i = 0; i < NUM_BOUNDING_SPHERES; i++) {
      Bone* cur_bone = bones_in_file_->bones[sph_bone_ind_[i]];
      Geometry* geom = cur_bone->getNode();
      bone_offset_mat.set(cur_bone->bone_offset); 
      Float4x4::inverse(temp, bone_offset_mat);
      Float3::affineTransformPos(center, temp, origin);
      center.accum(&HandModel::sph_off_[i*3]);
      cur_sphere = new BoundingSphere(HandModel::sph_size_[i], center, mesh_, 
        scene_graph_, cur_bone->bone_offset);
      geom->addChild(cur_sphere);
      g_hand_renderer->addBSphere(cur_sphere);
      bspheres_.pushBack(cur_sphere);
    }
  }
  
  const float finger_twist[4] = {-0.1f, -0.5f, 0.0f, -0.1f};

  void HandGeometryMesh::updateMatrices(const float* coeff) {
    FloatQuat cur_rot_quat;
    Float4x4* mat;

    // Set the root matrix:
    mat = scene_graph_->mat();
    Float4x4::euler2RotMat(*mat, coeff[HAND_ORIENT_X], coeff[HAND_ORIENT_Y],
      coeff[HAND_ORIENT_Z]);
    mat->leftMultTranslation(coeff[HAND_POS_X],
                             coeff[HAND_POS_Y],
                             coeff[HAND_POS_Z]);
    mat->rightMultScale(coeff[SCALE], coeff[SCALE], coeff[SCALE]); 

    // Set the palm bone (depending on wrist angle)
    mat = bones_in_file_->bones[bone_wrist_index_]->getNode()->mat();
    Float4x4::rotateMatXAxis(mat_tmp1, coeff[WRIST_PHI]);
    Float4x4::rotateMatZAxis(mat_tmp2, coeff[WRIST_THETA]);
    Float4x4::mult(mat_tmp3, mat_tmp1, mat_tmp2);
    // Float4x4::rotateMatXAxis(mat_tmp1, HandModel::wrist_twist);
    // Float4x4::mult(mat_tmp2, mat_tmp1, mat_tmp3);
    Float4x4::mult(*mat, rest_transforms_[bone_wrist_index_], mat_tmp3);

    // Set the finger bones
    for (uint32_t i = 0; i < 4; i++) {
      // K1 base
      float theta = coeff[F0_THETA + i * FINGER_NUM_COEFF];
      float phi = coeff[F0_PHI + i * FINGER_NUM_COEFF];
      float psi = coeff[F0_TWIST + i];
      mat = bones_in_file_->bones[bone_finger_2_index_[i]]->getNode()->mat();
      Float4x4::euler2RotMat(mat_tmp3, psi, theta, phi);
      Float4x4::mult(*mat, rest_transforms_[bone_finger_2_index_[i]], mat_tmp3);
      mat->rightMultScale(1.0f, 1.0f + coeff[F0_LENGTH + i], 1.0f);  // Scale this node

      mat = bones_in_file_->bones[bone_finger_3_index_[i]]->getNode()->mat();
      float k2_theta = coeff[F0_KNUCKLE_CURL + i * FINGER_NUM_COEFF];
      Float4x4::rotateMatXAxis(mat_tmp1, k2_theta);
      const Float4x4& bone_mid = rest_transforms_[bone_finger_3_index_[i]];
      Float3 bone_mid_pos;
      Float4x4::getTranslation(bone_mid_pos, bone_mid);
      float bone_base_length = bone_mid_pos.length();
      mat_tmp2.set(bone_mid);
      // Move bone by fraction of the bone length:
      mat_tmp2.leftMultTranslation(0, bone_base_length * coeff[F0_LENGTH + i], 0);  
      Float4x4::mult(*mat, mat_tmp2, mat_tmp1);
      mat->leftMultScale(1.0f, 1.0f / (1.0f + coeff[F0_LENGTH + i]), 1.0f);  // Undo parent scale
      mat->rightMultScale(1.0f, 1.0f + coeff[F0_LENGTH + i], 1.0f);  // Scale this node

      mat = bones_in_file_->bones[bone_finger_4_index_[i]]->getNode()->mat();
      const Float4x4& bone_tip = rest_transforms_[bone_finger_4_index_[i]];
      Float3 bone_tip_pos;
      Float4x4::getTranslation(bone_tip_pos, bone_tip);
      float bone_mid_length = bone_tip_pos.length();
      mat_tmp2.set(bone_tip);
      // Move bone by fraction of the bone length:
      mat_tmp2.leftMultTranslation(0, bone_mid_length * coeff[F0_LENGTH + i], 0);
      Float4x4::mult(*mat, mat_tmp2, mat_tmp1);

      mat->leftMultScale(1.0f, 1.0f / (1.0f + coeff[F0_LENGTH + i]), 1.0f);  // Undo parent scale
      mat->rightMultScale(1.0f, 1.0f + coeff[F0_LENGTH + i], 1.0f);  // Scale this node
    }

    // Set the thumb bones
    float theta = coeff[THUMB_THETA];
    float phi = coeff[THUMB_PHI];
    float psi = coeff[THUMB_TWIST];
    mat = bones_in_file_->bones[bone_thumb_1_index_]->getNode()->mat();
    Float4x4::euler2RotMat(mat_tmp3, psi, theta, phi);
    Float4x4::mult(*mat, rest_transforms_[bone_thumb_1_index_], mat_tmp3);

#ifdef SCALE_THUMB_BASE
    mat->rightMultScale(1.0f, 1.0f + coeff[THUMB_LENGTH], 1.0f);  // Scale this node
#endif

    theta = coeff[THUMB_K1_THETA];
    phi = coeff[THUMB_K1_PHI];
    mat = bones_in_file_->bones[bone_thumb_2_index_]->getNode()->mat();
    Float4x4::rotateMatZAxis(mat_tmp1, theta);
    Float4x4::rotateMatXAxis(mat_tmp2, phi);
    Float4x4::mult(mat_tmp3, mat_tmp1, mat_tmp2);

    const Float4x4& bone_mid = rest_transforms_[bone_thumb_2_index_];
#ifdef SCALE_THUMB_BASE
    Float3 bone_mid_pos;
    Float4x4::getTranslation(bone_mid_pos, bone_mid);
    float bone_base_length = bone_mid_pos.length();
#endif
    mat_tmp2.set(bone_mid);
#ifdef SCALE_THUMB_BASE
    // Move bone by fraction of the bone length:
    mat_tmp2.leftMultTranslation(0, bone_base_length * coeff[THUMB_LENGTH], 0);  
#endif
    Float4x4::mult(*mat, mat_tmp2, mat_tmp3);
#ifdef SCALE_THUMB_BASE
    mat->leftMultScale(1.0f, 1.0f / (1.0f + coeff[THUMB_LENGTH]), 1.0f);  // Undo parent scale
#endif
    mat->rightMultScale(1.0f, 1.0f + coeff[THUMB_LENGTH], 1.0f);  // Scale this node

    phi = coeff[THUMB_K2_PHI];
    mat = bones_in_file_->bones[bone_thumb_3_index_]->getNode()->mat();
    Float4x4::rotateMatXAxis(mat_tmp1, phi);
    const Float4x4& bone_tip = rest_transforms_[bone_thumb_3_index_];
    Float3 bone_tip_pos;
    Float4x4::getTranslation(bone_tip_pos, bone_tip);
    float bone_mid_length = bone_tip_pos.length();
    mat_tmp2.set(bone_tip);
    // Move bone by fraction of the bone length:
    mat_tmp2.leftMultTranslation(0, bone_mid_length * coeff[THUMB_LENGTH], 0);
    Float4x4::mult(*mat, mat_tmp2, mat_tmp1);
    mat->leftMultScale(1.0f, 1.0f / (1.0f + coeff[THUMB_LENGTH]), 1.0f);  // Undo parent scale
    mat->rightMultScale(1.0f, 1.0f + coeff[THUMB_LENGTH], 1.0f);  // Scale this node

  }
  
  void HandGeometryMesh::renderStackReset() {
    render_stack_.resize(0);  // empty the stack (without deallocating)
    // Seed the render stack with the root node
    render_stack_.pushBack(scene_graph_);
  }

  Geometry* HandGeometryMesh::renderStackPop() {
    Geometry* ret = NULL;
    if (render_stack_.size() > 0) {
      ret = render_stack_[render_stack_.size()-1];
      render_stack_.popBack();  // Remove the last element

      // Now add the children to the geometry stack
      for (uint32_t i = 0; i < ret->numChildren(); i ++) {
        render_stack_.pushBack(ret->getChild(i));
      }
    }
    return ret;
  }

  bool HandGeometryMesh::renderStackEmpty() {
    return render_stack_.size() == 0;
  }

  void HandGeometryMesh::updateHeirachyMatrices() {
    renderStackReset();
    while (!renderStackEmpty()) {
      Geometry* cur_geom = renderStackPop();
      // Update the render matrix based on our parents position
      if (cur_geom->parent() != NULL) {
        Float4x4::mult(*cur_geom->mat_hierarchy(),
          *cur_geom->parent()->mat_hierarchy(), *cur_geom->mat());

      } else {
        cur_geom->mat_hierarchy()->set(*cur_geom->mat());
      }
    }
    // Now update bone matrices
    GeometryManager::g_geom_manager()->updateBoneMatrices(bones_in_file_);
  }

  void HandGeometryMesh::fixBoundingSphereMatrices() {
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
        Float4x4::mult(tmp, root_inverse, *sphere->mat_hierarchy());
        Float4x4::mult(*sphere->mat_hierarchy(), *sphere->mesh_node()->mat_hierarchy(), tmp);
      }
    }
  }

}  // namespace hand_fit
