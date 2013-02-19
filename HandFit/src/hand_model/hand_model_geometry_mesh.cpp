#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include "hand_model/hand_model_renderer.h"
#include "hand_model/hand_model_geometry_mesh.h"
#include "hand_model/bounding_sphere.h"
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
#include "data_str/pair.h"

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
using renderer::Camera;
using renderer::Geometry;
using renderer::GeometryType;
using renderer::GeometryTexturedBonedMesh;
using renderer::Renderer;
using renderer::GeometryManager;
using renderer::Bone;
using renderer::BoneFileInfo;
using renderer::BoundingSphere;

namespace hand_model {
  
  HandModelGeometryMesh::HandModelGeometryMesh(HandType hand_type, 
    Renderer* g_renderer, HandModelRenderer* g_hand_renderer) : 
    HandModelGeometry(hand_type) {
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
        data_str::Vector<math::Float3>* norms = m->normals();
        for (uint32_t i = 0; i < norms->size(); i++) {
          norms->at(i)->scale(-1.0f);
        }
        m->syncVAO();
      }
    }

    // Adding the geometry to the gobal geometry manager's scene graph will
    // transfer ownership of the memory.
    GeometryManager::scene_graph_root()->addChild(scene_graph_);
  }

  HandModelGeometryMesh::~HandModelGeometryMesh() {
    // Note, ownership of all geometry is transfered to the renderer class
  }

  uint32_t HandModelGeometryMesh::sph_bone_ind_[NUM_BOUNDING_SPHERES];

  void HandModelGeometryMesh::createHandGeometry(Renderer* g_renderer, 
    HandModelRenderer* g_hand_renderer, HandType type) {
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
      throw runtime_error(string("HandModelGeometryMesh::createHandGeometry()")+
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
       bone_thumb_3_index_ == MAX_UINT32 || !bones_ok) {
      throw runtime_error(string("HandModelGeometryMesh::createHandGeometry()") +
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
      rest_transforms_[i].set(bones_in_file_->bones[i]->getNode()->mat());
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
    for (uint32_t i = 0; i < NUM_BOUNDING_SPHERES; i++) {
      Bone* cur_bone = bones_in_file_->bones[sph_bone_ind_[i]];
      Geometry* geom = cur_bone->getNode();
      bone_offset_mat.set(cur_bone->bone_offset); 
      bone_offset_mat.inverse();
      Float3::affineTransformPos(&center, &bone_offset_mat, &origin);
      center.accum(&sph_off_[i*3]);
      cur_sphere = new BoundingSphere(sph_size_[i], center, mesh_, 
        scene_graph_, cur_bone->bone_offset);
      geom->addChild(cur_sphere);
      g_hand_renderer->addBSphere(cur_sphere);
      bspheres_.pushBack(cur_sphere);
    }
  }
  
  void HandModelGeometryMesh::updateMatrices(const Eigen::MatrixXf& coeff) {
    FloatQuat cur_rot_quat;
    Float4x4* mat;

    // Set the root matrix:
    mat = scene_graph_->mat();
    cur_rot_quat.set(static_cast<float>(coeff(HAND_ORIENT_X)),
                     static_cast<float>(coeff(HAND_ORIENT_Y)),
                     static_cast<float>(coeff(HAND_ORIENT_Z)),
                     static_cast<float>(coeff(HAND_ORIENT_W)));
    cur_rot_quat.quat2Mat4x4(mat);
    mat->leftMultTranslation(static_cast<float>(coeff(HAND_POS_X)),
                             static_cast<float>(coeff(HAND_POS_Y)),
                             static_cast<float>(coeff(HAND_POS_Z)));
    mat->rightMultScale(HandModel::scale, HandModel::scale, HandModel::scale);

    // Set the palm bone (depending on wrist angle)
    mat = bones_in_file_->bones[bone_palm_index_]->getNode()->mat();
    mat_tmp1.rotateMatXAxis(static_cast<float>(coeff(WRIST_PHI)));
    mat_tmp2.rotateMatZAxis(static_cast<float>(coeff(WRIST_THETA)));
    Float4x4::mult(&mat_tmp3, &mat_tmp1, &mat_tmp2);
    mat_tmp1.rotateMatXAxis(HandModel::wrist_twist);
    Float4x4::mult(&mat_tmp2, &mat_tmp1, &mat_tmp3);
    Float4x4::mult(mat, &rest_transforms_[bone_palm_index_], &mat_tmp2);

    // Set the finger bones
    for (uint32_t i = 0; i < 4; i++) {
      // K1 base
      float theta = static_cast<float>(coeff(F0_THETA + i * FINGER_NUM_COEFF));
      float phi = static_cast<float>(coeff(F0_PHI + i * FINGER_NUM_COEFF));
      mat = bones_in_file_->bones[bone_finger_2_index_[i]]->getNode()->mat();
      mat_tmp1.rotateMatZAxis(theta);
      mat_tmp2.rotateMatXAxis(phi);
      Float4x4::mult(&mat_tmp3, &mat_tmp1, &mat_tmp2);
      Float4x4::mult(mat, &rest_transforms_[bone_finger_2_index_[i]], &mat_tmp3);

      mat = bones_in_file_->bones[bone_finger_3_index_[i]]->getNode()->mat();
      float k2_theta = static_cast<float>(coeff(F0_KNUCKLE_CURL + i * FINGER_NUM_COEFF));
      mat_tmp1.rotateMatXAxis(k2_theta);
      Float4x4::mult(mat, &rest_transforms_[bone_finger_3_index_[i]], &mat_tmp1);

      mat = bones_in_file_->bones[bone_finger_4_index_[i]]->getNode()->mat();
      // float k3_theta = static_cast<float>(coeff(F0_KNUCKLE_CURL + i * FINGER_NUM_COEFF));
      // mat_tmp1.rotateMatXAxis(k3_theta);
      Float4x4::mult(mat, &rest_transforms_[bone_finger_4_index_[i]], &mat_tmp1);
    }

    // Set the thumb bones
    float theta = static_cast<float>(coeff(THUMB_THETA));
    float phi = static_cast<float>(coeff(THUMB_PHI));
    mat = bones_in_file_->bones[bone_thumb_1_index_]->getNode()->mat();
    mat_tmp1.rotateMatZAxis(theta);
    mat_tmp2.rotateMatXAxis(phi);
    Float4x4::mult(&mat_tmp3, &mat_tmp1, &mat_tmp2);
    Float4x4::mult(mat, &rest_transforms_[bone_thumb_1_index_], &mat_tmp3);

    theta = static_cast<float>(coeff(THUMB_K1_THETA));
    phi = static_cast<float>(coeff(THUMB_K1_PHI));
    mat = bones_in_file_->bones[bone_thumb_2_index_]->getNode()->mat();
    mat_tmp1.rotateMatZAxis(theta);
    mat_tmp2.rotateMatXAxis(phi);
    Float4x4::mult(&mat_tmp3, &mat_tmp1, &mat_tmp2);
    Float4x4::mult(mat, &rest_transforms_[bone_thumb_2_index_], &mat_tmp3);

    phi = static_cast<float>(coeff(THUMB_K2_PHI));
    mat = bones_in_file_->bones[bone_thumb_3_index_]->getNode()->mat();
    mat_tmp1.rotateMatXAxis(phi);
    Float4x4::mult(mat, &rest_transforms_[bone_thumb_3_index_], &mat_tmp1);
  }
  
  void HandModelGeometryMesh::renderStackReset() {
    render_stack_.resize(0);  // empty the stack (without deallocating)
    // Seed the render stack with the root node
    render_stack_.pushBack(scene_graph_);
  }

  Geometry* HandModelGeometryMesh::renderStackPop() {
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

  bool HandModelGeometryMesh::renderStackEmpty() {
    return render_stack_.size() == 0;
  }

  void HandModelGeometryMesh::updateHeirachyMatrices() {
    renderStackReset();
    while (!renderStackEmpty()) {
      Geometry* cur_geom = renderStackPop();
      // Update the render matrix based on our parents position
      if (cur_geom->parent() != NULL) {
        Float4x4::mult(cur_geom->mat_hierarchy(),
          cur_geom->parent()->mat_hierarchy(), cur_geom->mat());

      } else {
        cur_geom->mat_hierarchy()->set(cur_geom->mat());
      }
    }
    // Now update bone matrices
    GeometryManager::g_geom_manager()->updateBoneMatrices(bones_in_file_);
  }

  void HandModelGeometryMesh::fixBoundingSphereMatrices() {
    Float4x4 tmp;
    Float4x4 root_inverse;
    Float4x4::inverse(&root_inverse, scene_graph_->mat());
    renderStackReset();
    while (!renderStackEmpty()) {
      Geometry* cur_geom = renderStackPop();
      if (cur_geom->type() == GeometryType::BOUNDING_SPHERE) {
        // This is a bit of a hack, but we have to undo the parent's root
        // transform (by left multiplying by its inverse).
        // Then we have to left multiply by the mesh node's transform
        BoundingSphere* sphere = reinterpret_cast<BoundingSphere*>(cur_geom);
        Float4x4::mult(&tmp, &root_inverse, sphere->mat_hierarchy());
        Float4x4::mult(sphere->mat_hierarchy(), sphere->mesh_node()->mat_hierarchy(), &tmp);
      }
    }
  }

  const float HandModelGeometryMesh::sph_off_[NUM_BOUNDING_SPHERES*3] = { 
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

  const float HandModelGeometryMesh::sph_size_[NUM_BOUNDING_SPHERES] = {
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

}  // namespace hand_model
