#include <fstream>
#include <iostream>
#include "jtil/data_str/pair.h"
#include "hand_fit/bounding_sphere.h"
#include "renderer/colors.h"
#include "jtil/math/math_types.h"

#define max std::max

using jtil::math::Float3;
using std::wstring;
using std::runtime_error;
using std::string;
using std::cout;
using std::endl;
using jtil::data_str::Pair;

namespace renderer {
  
  BoundingSphere::BoundingSphere(float rad, Float3& center, Geometry* mesh_node,
    Geometry* hand_root, float* starting_bone_mat) :
  GeometryColoredMesh() {
    GeometryColoredMesh::makeSphere(reinterpret_cast<GeometryColoredMesh*>(this),
      BOUNDING_SPHERE_NSTACKS, BOUNDING_SPHERE_NSLICES, rad, center, white);
    radius_ = rad;
    center_ = center;
    mesh_node_ = mesh_node;
    hand_root_ = hand_root;
    starting_bone_mat_.set(starting_bone_mat);
    mat_.set(starting_bone_mat);
    rad_vec.set(rad, 0.0f, 0.0f);
  }

  BoundingSphere::~BoundingSphere() {
    // parent destructor will be called automatically
  }

  Geometry* BoundingSphere::copy() {
    throw runtime_error(string("BoundingSphere::copy() - ERROR: Not")+
      string(" yet implemented!"));
  }

  void BoundingSphere::loadFromArray(const std::string& path, 
      const std::string& filename, const uint8_t* arr) {
    throw runtime_error(string("BoundingSphere::loadFromArray() - ") +
      string("ERROR: Not yet implemented!"));
  }

  Pair<uint8_t*,uint32_t> BoundingSphere::saveToArray() {
    throw runtime_error(string("BoundingSphere::loadFromArray() - ") +
      string("ERROR: Not yet implemented!"));
  }

  void BoundingSphere::transform() {
    Float3::affineTransformPos(transformed_center_, mat_hierarchy_, center_);
    Float3::affineTransformVec(transformed_rad_vec, mat_hierarchy_, rad_vec);
    transformed_radius_ = transformed_rad_vec.length();
    axis_extent_[0] = transformed_center_[0] - transformed_radius_;  // Xmin
    axis_extent_[1] = transformed_center_[1] - transformed_radius_;  // Ymin
    axis_extent_[2] = transformed_center_[2] - transformed_radius_;  // Zmin
    axis_extent_[3] = transformed_center_[0] + transformed_radius_;  // Xmax
    axis_extent_[4] = transformed_center_[1] + transformed_radius_;  // Ymax
    axis_extent_[5] = transformed_center_[2] + transformed_radius_;  // Zmax
  }

}  // namespace renderer

