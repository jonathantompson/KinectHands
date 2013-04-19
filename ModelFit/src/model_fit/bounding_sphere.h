//
//  bounding_sphere.h
//
//  Created by Jonathan Tompson on 6/7/12.
//
//  Just a wireframe version of the colored mesh.

#ifndef RENDERER_BOUNDING_SPHERE_HEADER
#define RENDERER_BOUNDING_SPHERE_HEADER

#include "renderer/open_gl_common.h"  // For GLtypes
#include "renderer/geometry/geometry_colored_mesh.h"
#include "jtil/math/math_types.h"

#define BOUNDING_SPHERE_NSTACKS 10
#define BOUNDING_SPHERE_NSLICES 13

namespace renderer {

  class BoundingSphere : public GeometryColoredMesh {
  public:
    friend class GeometryManager;

    // Constructor / Destructor
    BoundingSphere(float radius, jtil::math::Float3& center, Geometry* mesh_node,
      Geometry* hand_root, float* starting_bone_mat);
    virtual ~BoundingSphere();
    
    // Virtual methods
    inline virtual GeometryType type() { return BOUNDING_SPHERE; }

    // Perform a deep copy of the entire structure and sync the copy with OGL
    virtual Geometry* copy();

    inline Geometry* mesh_node() { return mesh_node_; }
    inline Geometry* hand_root() { return hand_root_; }
    inline jtil::math::Float4x4* starting_bone_mat() { return &starting_bone_mat_; }

    void transform();
    jtil::math::Float3* transformed_center() { return &transformed_center_; }
    float transformed_radius() { return transformed_radius_; }

  protected:
    float radius_;
    jtil::math::Float3 center_;
    jtil::math::Float3 transformed_rad_vec_;  // Just to avoid putting it on the stack
    jtil::math::Float3 transformed_center_;
    static const jtil::math::Float3 rad_vec_x_;
    static const jtil::math::Float3 rad_vec_y_;
    static const jtil::math::Float3 rad_vec_z_;
    float transformed_radius_;

    jtil::math::Float4x4 starting_bone_mat_;
    Geometry* hand_root_;
    Geometry* mesh_node_;

    // loadFromArray - Internal use for when loading from jbin format.
    virtual void loadFromArray(const std::string& path, 
      const std::string& filename, const uint8_t* arr);

    // Convert the node and it's geometry into a data array for saving to file
    virtual jtil::data_str::Pair<uint8_t*,uint32_t> saveToArray();

    // Non-copyable, non-assignable.
    BoundingSphere(BoundingSphere&);
    BoundingSphere& operator=(const BoundingSphere&);
  };
  
};  // renderer namespace

#endif  // RENDERER_BOUNDING_SPHERE_HEADER
