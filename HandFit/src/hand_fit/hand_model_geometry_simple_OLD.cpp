#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include "hand_model/hand_model_geometry_simple.h"
#include "renderer/open_gl_common.h"
#include "renderer/renderer.h"
#include "renderer/colors.h"
#include "renderer/geometry/geometry.h"
#include "renderer/geometry/geometry_manager.h"
#include "renderer/geometry/geometry_vertices.h"
#include "renderer/geometry/geometry_colored_mesh.h"
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
using renderer::Camera;
using renderer::Geometry;
using renderer::GeometryManager;
using renderer::GeometryColoredMesh;
using renderer::Renderer;

namespace hand_model {

  const int HandModelGeometrySimple::nslices = 16;  // even number
  const int HandModelGeometrySimple::nstacks = 11;  // odd number
  const int HandModelGeometrySimple::knuckle_nslices = 12;  // even number
  const int HandModelGeometrySimple::knuckle_nstacks = 7;  // odd number
  const float HandModelGeometrySimple::finger_width_scale = 1.15f;

  const float HandModelGeometrySimple::hand_palm_width = 110.0f;
  const float HandModelGeometrySimple::hand_palm_thickness = 45.0f;
  const float HandModelGeometrySimple::hand_palm_length = 80.0f;
  const float HandModelGeometrySimple::wrist_width = 80.0f;
  const float HandModelGeometrySimple::wrist_thickness = 35.0f;
  
  const float HandModelGeometrySimple::finger_offsets[4] = {-10.0f, -5.0f, 0.0f, -5.0f};
  const float HandModelGeometrySimple::finger_k1_lengths[4] = {50.0f, 60.0f, 62.5f, 60.0f};
  const float HandModelGeometrySimple::finger_k2_lengths[4] = {25.0f, 35.0f, 37.5f, 30.0f};
  const float HandModelGeometrySimple::finger_k3_lengths[4] = {22.5f, 25.0f, 25.0f, 22.5f};
  const float HandModelGeometrySimple::finger_k1_widths[4] = {19.0f, 22.0f, 22.0f, 22.0f};
  const float HandModelGeometrySimple::finger_k2_widths[4] = {18.0f, 19.0f, 20.0f, 20.0f};
  const float HandModelGeometrySimple::finger_k3_widths[4] = {17.0f, 18.0f, 19.0f, 19.0f};

  const float HandModelGeometrySimple::thumb_k1_length = 50.0f;
  const float HandModelGeometrySimple::thumb_k2_length = 45.0f;
  const float HandModelGeometrySimple::thumb_k3_length = 30.0f;
  const float HandModelGeometrySimple::thumb_k1_width = 35.0f;
  const float HandModelGeometrySimple::thumb_k2_width = 23.0f;
  const float HandModelGeometrySimple::thumb_k3_width = 22.0f;
  
  HandModelGeometrySimple::HandModelGeometrySimple(HandType hand_type, 
    Renderer* g_renderer) : HandModelGeometry(hand_type) {
    createHandGeometry(g_renderer);
    GeometryManager::scene_graph_root()->addChild(scene_graph_);
  }

  HandModelGeometrySimple::~HandModelGeometrySimple() {
    // Note, ownership of all geometry is transfered to the renderer class
  }
  
  void HandModelGeometrySimple::createHandGeometry(Renderer* g_renderer) {
    scene_graph_ = new Geometry();  // Empty geometry
    
    // Palm
    palm_cylinder_ = GeometryColoredMesh::makeOpenCylinder(nslices, 1.0f, 1.0, 
      1.0f, renderer::red);
    scene_graph_->addChild(palm_cylinder_);
    
    palm_sphere1_ = GeometryColoredMesh::makeSphere(nstacks, nslices, 1.0f, 
      renderer::red);
    scene_graph_->addChild(palm_sphere1_);
    
    palm_sphere2_ = GeometryColoredMesh::makeSphere(nstacks, nslices, 1.0f, 
      renderer::red);
    scene_graph_->addChild(palm_sphere2_);
 
    // Wrist
    wrist_base_ = new Geometry();  // Empty geometry
    scene_graph_->addChild(wrist_base_);
    wrist_cylinder_ = GeometryColoredMesh::makeOpenCylinder(nslices, 1.0f, 1.0, 
      1.0f, renderer::blue);
    wrist_base_->addChild(wrist_cylinder_);
    wrist_sphere1_ = GeometryColoredMesh::makeSphere(nstacks, nslices, 1.0f, 
      renderer::blue);
    wrist_base_->addChild(wrist_sphere1_);
    wrist_sphere2_ = GeometryColoredMesh::makeSphere(nstacks, nslices, 1.0f,
      renderer::blue);
    wrist_base_->addChild(wrist_sphere2_);

    // Set the finger geometry
    for (uint32_t i = 0; i < 4; i++) {
      // first knuckle
      finger_k1_base_[i] = new Geometry();  // Empty geometry
      scene_graph_->addChild(finger_k1_base_[i]);
      finger_k1_cylinder_[i] = GeometryColoredMesh::makeOpenCylinder(knuckle_nslices, 
        1.0f, finger_width_scale * finger_k2_widths[i] / finger_k1_widths[i], 
        finger_width_scale, renderer::blue);
      finger_k1_base_[i]->addChild(finger_k1_cylinder_[i]);
      finger_k1_sphere_[i] = GeometryColoredMesh::makeSphere(knuckle_nstacks, 
        knuckle_nslices, finger_width_scale, renderer::blue);
      finger_k1_base_[i]->addChild(finger_k1_sphere_[i]);

      // second knuckle
      finger_k2_base_[i] = new Geometry();  // Empty geometry
      finger_k1_base_[i]->addChild(finger_k2_base_[i]);
      finger_k2_cylinder_[i] = GeometryColoredMesh::makeOpenCylinder(knuckle_nslices, 
        1.0f, finger_width_scale * finger_k3_widths[i] / finger_k2_widths[i], 
        finger_width_scale, renderer::green);
      finger_k2_base_[i]->addChild(finger_k2_cylinder_[i]);
      finger_k2_sphere_[i] = GeometryColoredMesh::makeSphere(knuckle_nstacks, 
        knuckle_nslices, finger_width_scale, renderer::green);
      finger_k2_base_[i]->addChild(finger_k2_sphere_[i]);

      // third knuckle
      finger_k3_base_[i] = new Geometry();  // Empty geometry
      finger_k2_base_[i]->addChild(finger_k3_base_[i]);
      finger_k3_cylinder_[i] = GeometryColoredMesh::makeOpenCylinder(knuckle_nslices,
        1.0f, finger_width_scale, finger_width_scale, renderer::gold);
      finger_k3_base_[i]->addChild(finger_k3_cylinder_[i]);
      finger_k3_sphere_[i] = GeometryColoredMesh::makeSphere(knuckle_nstacks,
        knuckle_nslices, finger_width_scale, renderer::gold);
      finger_k3_base_[i]->addChild(finger_k3_sphere_[i]);
    }

    // Thumb first knuckle
    thumb_k1_base_ = new Geometry();  // Empty geometry
    scene_graph_->addChild(thumb_k1_base_);
    thumb_k1_cylinder_ = GeometryColoredMesh::makeOpenCylinder(knuckle_nslices, 
      1.0f, finger_width_scale * thumb_k2_width / thumb_k1_width, 
      finger_width_scale, renderer::blue);
    thumb_k1_base_->addChild(thumb_k1_cylinder_);
    thumb_k1_sphere1_ = GeometryColoredMesh::makeSphere(knuckle_nstacks, 
      knuckle_nslices, finger_width_scale, renderer::blue);
    thumb_k1_base_->addChild(thumb_k1_sphere1_);
    thumb_k1_sphere2_ = GeometryColoredMesh::makeSphere(knuckle_nstacks, 
      knuckle_nslices, finger_width_scale, renderer::blue);
    thumb_k1_base_->addChild(thumb_k1_sphere2_);

    // Thumb second knuckle
    thumb_k2_base_ = new Geometry();  // Empty geometry
    thumb_k1_base_->addChild(thumb_k2_base_);
    thumb_k2_cylinder_ = GeometryColoredMesh::makeOpenCylinder(knuckle_nslices, 
      1.0f, finger_width_scale * thumb_k3_width / thumb_k2_width, 
      finger_width_scale, renderer::blue);
    thumb_k2_base_->addChild(thumb_k2_cylinder_);
    thumb_k2_sphere_ = GeometryColoredMesh::makeSphere(knuckle_nstacks, 
      knuckle_nslices, finger_width_scale, renderer::blue);
    thumb_k2_base_->addChild(thumb_k2_sphere_);

    // Thumb third knuckle
    thumb_k3_base_ = new Geometry();  // Empty geometry
    thumb_k2_base_->addChild(thumb_k3_base_);
    thumb_k3_cylinder_ = GeometryColoredMesh::makeOpenCylinder(knuckle_nslices,
      1.0f, finger_width_scale, finger_width_scale, renderer::blue);
    thumb_k3_base_->addChild(thumb_k3_cylinder_);
    thumb_k3_sphere_ = GeometryColoredMesh::makeSphere(knuckle_nstacks,
      knuckle_nslices, finger_width_scale, renderer::blue);
    thumb_k3_base_->addChild(thumb_k3_sphere_);
  }
  
 
  void HandModelGeometrySimple::updateMatrices(const Eigen::MatrixXf& coeff) {
    mat_rotateXAxis90.rotateMatXAxis(static_cast<float>(M_PI_2));

    FloatQuat cur_rot_quat;
    Float4x4* mat;
    
    float hand_type_multiplier = hand_type_ == HandType::LEFT ? 1.0f : -1.0f;
    
    // Set the root matrix:
    mat_tmp1.rotateMatZAxis(static_cast<float>(M_PI / 2.0));
    mat_tmp2.rotateMatXAxis(static_cast<float>(-M_PI / 2.0 + 0));
    Float4x4::mult(&mat_tmp3, &mat_tmp1, &mat_tmp2);
    mat = scene_graph_->mat();
    cur_rot_quat.set(static_cast<float>(coeff(HAND_ORIENT_X)),
                     static_cast<float>(coeff(HAND_ORIENT_Y)),
                     static_cast<float>(coeff(HAND_ORIENT_Z)),
                     static_cast<float>(coeff(HAND_ORIENT_W)));
    cur_rot_quat.quat2Mat4x4(&mat_tmp1);
    Float4x4::mult(mat, &mat_tmp1, &mat_tmp3);
    mat->leftMultTranslation(static_cast<float>(coeff(HAND_POS_X))+23.5f,
                             static_cast<float>(coeff(HAND_POS_Y))+90.75f,
                             static_cast<float>(coeff(HAND_POS_Z))+72.5f);
    float scale = 0.8f * HandModel::scale / 50.0f;
    mat->rightMultScale(scale, scale, scale);
    
    // Set the palm matrices
    // Cylinder
    mat = palm_cylinder_->mat();
    mat->set(&mat_rotateXAxis90);
    mat->leftMultScale(hand_palm_width * 0.5f, hand_palm_thickness * 0.5f, hand_palm_length);
    // Sphere 1
    mat = palm_sphere1_->mat();
    mat->scaleMat(hand_palm_width * 0.5f, hand_palm_thickness * 0.5f, hand_palm_length * 0.3f);
    mat->leftMultTranslation(0.0f, 0.0f, -0.5f * hand_palm_length);
    // Sphere 2
    mat = palm_sphere2_->mat();
    mat->scaleMat(hand_palm_width * 0.5f, hand_palm_thickness * 0.5f, hand_palm_length * 0.3f);
    mat->leftMultTranslation(0.0f, 0.0f, +0.5f * hand_palm_length);
    
    // Set the wrist matrices
    // Base
    mat = wrist_base_->mat();
    mat_tmp1.rotateMatXAxis(static_cast<float>(-coeff(WRIST_PHI)) + 0.1f);
    mat_tmp2.rotateMatYAxis(static_cast<float>(coeff(WRIST_THETA) + M_PI) + 0.03f);
    Float4x4::mult(mat, &mat_tmp2, &mat_tmp1);
    mat->leftMultTranslation(0.0f, 0.0f, -0.5f * hand_palm_length);
    // Cylinder
    mat = wrist_cylinder_->mat();
    mat->set(&mat_rotateXAxis90);
    mat->leftMultTranslation(0.0f, 0.0f, 0.5f);
    mat->leftMultScale(wrist_width * 0.5f, wrist_thickness * 0.5f, HandModel::wrist_length);
    // Sphere 1
    mat = wrist_sphere1_->mat();
    mat->scaleMat(wrist_width * 0.5f, wrist_thickness * 0.5f, hand_palm_thickness * 0.5f);
    // Sphere 2
    mat = wrist_sphere2_->mat();
    mat->scaleMat(wrist_width * 0.5f, wrist_thickness * 0.5f, HandModel::wrist_length*0.2f);
    mat->leftMultTranslation(0.0f, 0.0f, HandModel::wrist_length);
    
    // Set the finger geometry
    for (uint32_t i = 0; i < 4; i++) {
      // K1 base
      float theta = static_cast<float>(coeff(F0_THETA + i * FINGER_NUM_COEFF));
      float phi = static_cast<float>(-coeff(F0_PHI + i * FINGER_NUM_COEFF)) + 0.1f;
      mat = finger_k1_base_[i]->mat();
      mat_tmp1.rotateMatXAxis(phi);
      mat_tmp2.rotateMatYAxis(theta);
      Float4x4::mult(mat, &mat_tmp2, &mat_tmp1);
      mat->leftMultTranslation(hand_type_multiplier * (-3.0f + i * 2.0f) * hand_palm_width / 8.0f,
        0.0f, 0.5f * hand_palm_length + finger_offsets[i]);

      // K1 cylinder
      mat = finger_k1_cylinder_[i]->mat();
      mat->set(&mat_rotateXAxis90);
      mat->leftMultTranslation(0.0f, 0.0f, 0.5f);
      mat->leftMultScale(finger_k1_widths[i] * 0.5f, 
        finger_k1_widths[i] * 0.5f, finger_k1_lengths[i]);

      // K1 sphere
      mat = finger_k1_sphere_[i]->mat();
      mat->scaleMat(finger_k2_widths[i] * 0.5f, finger_k2_widths[i] * 0.5f, 
        finger_k2_widths[i] * 0.5f);
      mat->leftMultTranslation(0.0f, 0.0f, finger_k1_lengths[i]);

      // K2 base
      float k2_theta = static_cast<float>(-coeff(F0_KNUCKLE_CURL + i * FINGER_NUM_COEFF)) + 0.3f;
      mat = finger_k2_base_[i]->mat();
      mat->rotateMatXAxis(k2_theta);
      mat->leftMultTranslation(0.0f, 0.0f, finger_k1_lengths[i]);

      // K2 cylinder
      mat = finger_k2_cylinder_[i]->mat();
      mat->set(&mat_rotateXAxis90);
      mat->leftMultTranslation(0.0f, 0.0f, 0.5f);
      mat->leftMultScale(finger_k2_widths[i] * 0.5f, 
        finger_k2_widths[i] * 0.5f, finger_k2_lengths[i]);

      // K2 sphere
      mat = finger_k2_sphere_[i]->mat();
      mat->scaleMat(finger_k3_widths[i] * 0.5f, finger_k3_widths[i] * 0.5f, 
        finger_k3_widths[i] * 0.5f);
      mat->leftMultTranslation(0.0f, 0.0f, finger_k2_lengths[i]);

      // K3 base
      float k3_theta = static_cast<float>(-coeff(F0_KNUCKLE_CURL + i * FINGER_NUM_COEFF)) + 0.3f;
      mat = finger_k3_base_[i]->mat();
      mat->rotateMatXAxis(k3_theta);
      mat->leftMultTranslation(0.0f, 0.0f, finger_k2_lengths[i]);

      // K3 cylinder
      mat = finger_k3_cylinder_[i]->mat();
      mat->set(&mat_rotateXAxis90);
      mat->leftMultTranslation(0.0f, 0.0f, 0.5f);
      mat->leftMultScale(finger_k3_widths[i] * 0.5f, 
        finger_k3_widths[i] * 0.5f, finger_k3_lengths[i]);

      // K3 sphere
      mat = finger_k3_sphere_[i]->mat();
      mat->scaleMat(finger_k3_widths[i] * 0.5f, finger_k3_widths[i] * 0.5f, 
        finger_k3_widths[i] * 0.5f);
      mat->leftMultTranslation(0.0f, 0.0f, finger_k3_lengths[i]);
    }

    // Thumb K1 base
    float theta = (static_cast<float>(coeff(THUMB_THETA)) + 0.48f +
      static_cast<float>((hand_type_ == HandType::LEFT) ? 0 : (-M_PI_4)));
    float phi = -static_cast<float>(coeff(THUMB_PHI)) + 0.4f;
    mat = thumb_k1_base_->mat();
    mat_tmp1.rotateMatXAxis(phi);
    mat_tmp2.rotateMatYAxis(theta);
    Float4x4::mult(mat, &mat_tmp1, &mat_tmp2);
    mat->leftMultTranslation(hand_type_multiplier * 3.0f * hand_palm_width / 8.0f,
                             0.0f, -0.5f * hand_palm_length);

    // Thumb K1 cylinder
    mat = thumb_k1_cylinder_->mat();
    mat->set(&mat_rotateXAxis90);
    mat->leftMultTranslation(0.0f, 0.0f, 0.5f);
    mat->leftMultScale(thumb_k1_width * 0.5f, thumb_k1_width * 0.5f, 
      thumb_k1_length);

    // Thumb K1 sphere 1
    mat = thumb_k1_sphere1_->mat();
    mat->scaleMat(thumb_k1_width * 0.5f, thumb_k1_width * 0.5f, 
      thumb_k1_width * 0.5f);

    // Thumb K1 sphere 2
    mat = thumb_k1_sphere2_->mat();
    mat->scaleMat(thumb_k2_width * 0.5f, thumb_k2_width * 0.5f, 
      thumb_k2_width * 0.5f);
    mat->leftMultTranslation(0.0f, 0.0f, thumb_k1_length);

    // Thumb K2 base
    float k2_theta = static_cast<float>(coeff(THUMB_K1_THETA));
    float k2_phi = -static_cast<float>(coeff(THUMB_K1_PHI)) + 0.4f;
    mat_tmp1.rotateMatXAxis(k2_phi);
    mat_tmp2.rotateMatYAxis(k2_theta);
    mat_tmp3.rotateMatZAxis(hand_type_multiplier * -static_cast<float>(M_PI_4));  // Thumb needs twist
    mat = thumb_k2_base_->mat();
    Float4x4::mult(mat, &mat_tmp1, &mat_tmp2);
    Float4x4::mult(&mat_tmp2, &mat_tmp3, mat);
    mat->set(&mat_tmp2);
    mat->leftMultTranslation(0.0f, 0.0f, thumb_k1_length);

    // Thumb K2 cylinder
    mat = thumb_k2_cylinder_->mat();
    mat->set(&mat_rotateXAxis90);
    mat->leftMultTranslation(0.0f, 0.0f, 0.5f);
    mat->leftMultScale(thumb_k2_width * 0.5f, thumb_k2_width * 0.5f, 
      thumb_k2_length);

    // Thumb K1 sphere
    mat = thumb_k2_sphere_->mat();
    mat->scaleMat(thumb_k3_width * 0.5f, thumb_k3_width * 0.5f, 
      thumb_k3_width * 0.5f);
    mat->leftMultTranslation(0.0f, 0.0f, thumb_k2_length);

    // Thumb K3 base
    float k3_theta = -static_cast<float>(coeff(THUMB_K2_PHI)) + 0.3f;
    mat = thumb_k3_base_->mat();
    mat->rotateMatXAxis(k3_theta);
    mat->leftMultTranslation(0.0f, 0.0f, thumb_k2_length);

    // Thumb K3 cylinder
    mat = thumb_k3_cylinder_->mat();
    mat->set(&mat_rotateXAxis90);
    mat->leftMultTranslation(0.0f, 0.0f, 0.5f);
    mat->leftMultScale(thumb_k3_width * 0.5f, thumb_k3_width * 0.5f, 
      thumb_k3_length);

    // Thumb K3 sphere
    mat = thumb_k3_sphere_->mat();
    mat->scaleMat(thumb_k3_width * 0.5f, thumb_k3_width * 0.5f, 
      thumb_k3_width * 0.5f);
    mat->leftMultTranslation(0.0f, 0.0f, thumb_k3_length);
  }
  
  void HandModelGeometrySimple::renderStackReset() {
    render_stack_.resize(0);  // empty the stack (without deallocating)
    // Seed the render stack with the root node
    render_stack_.pushBack(scene_graph_);
  }

  Geometry* HandModelGeometrySimple::renderStackPop() {
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

  bool HandModelGeometrySimple::renderStackEmpty() {
    return render_stack_.size() == 0;
  }

  void HandModelGeometrySimple::updateHeirachyMatrices() {
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
  }

}  // namespace hand_model
