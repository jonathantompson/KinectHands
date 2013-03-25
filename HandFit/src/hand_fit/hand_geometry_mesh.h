//
//  hand_model_geometry_mesh.h
//
//  Created by Jonathan Tompson on 10/30/12.
//
//  The ball and cylinder hand model

#ifndef HAND_MODEL_HAND_MODEL_GEOMETRY_HESH_HEADER
#define HAND_MODEL_HAND_MODEL_GEOMETRY_HESH_HEADER

#include "renderer/open_gl_common.h"  // GLfloat
#include "kinect_interface/hand_net/hand_model.h"
#include "hand_fit/hand_geometry.h"
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector.h"
#include "jtil/data_str/pair.h"
#include "Eigen"

namespace renderer { class Geometry; }
namespace renderer { struct BoneFileInfo; }
namespace renderer { struct Bone; }
namespace renderer { class Renderer; }
namespace renderer { class BoundingSphere; }

namespace hand_fit {
  class HandRenderer;

  class HandGeometryMesh : public HandGeometry {
  public:
    // Constructor / Destructor
    HandGeometryMesh(kinect_interface::hand_net::HandType hand_type, 
      renderer::Renderer* g_renderer,
      HandRenderer* g_hand_renderer);
    virtual ~HandGeometryMesh();

    // Call before rendering hand depth maps:
    virtual void updateMatrices(const float* coeff);
    virtual void updateHeirachyMatrices();
    virtual void fixBoundingSphereMatrices();
    virtual inline renderer::Geometry* scene_graph() { return scene_graph_; }
    
    jtil::data_str::Vector<renderer::BoundingSphere*>& bspheres() { return bspheres_; }

    virtual void renderStackReset();
    virtual renderer::Geometry* renderStackPop();
    virtual bool renderStackEmpty();

  private:
    // Note all geometry is attached to the global renderer's scene graph and
    // therefore we transfer ownership of the memory to it.
    renderer::Geometry* scene_graph_;  // The renderable geometry - Not owned here
    renderer::BoneFileInfo* bones_in_file_;  // Not owned here
    jtil::data_str::Vector<jtil::math::Float4x4> rest_transforms_;  // per bone
    jtil::data_str::Vector<renderer::BoundingSphere*> bspheres_;  // Attached to scene graph!
    uint32_t bone_wrist_index_;
    uint32_t bone_palm_index_;
    uint32_t bone_thumb_1_index_;
    uint32_t bone_thumb_2_index_;
    uint32_t bone_thumb_3_index_;
    uint32_t bone_finger_1_index_[4];
    uint32_t bone_finger_2_index_[4];
    uint32_t bone_finger_3_index_[4];
    uint32_t bone_finger_4_index_[4];
    renderer::Geometry* mesh_;  // Not owned here

    // Temp matricies are not static to be thread safe.
    jtil::math::Float4x4 mat_tmp1;
    jtil::math::Float4x4 mat_tmp2;
    jtil::math::Float4x4 mat_tmp3;

    static uint32_t sph_bone_ind_[NUM_SPHERES];

    void createHandGeometry(renderer::Renderer* g_renderer,
      HandRenderer* g_hand_renderer, 
      kinect_interface::hand_net::HandType type);

    // Copy of the Renderer's stack interface methods, I'd rather duplicate
    // them and keep the renderer seperate.
    jtil::data_str::Vector<renderer::Geometry*> render_stack_;

    // Non-copyable, non-assignable.
    HandGeometryMesh(HandGeometryMesh&);
    HandGeometryMesh& operator=(const HandGeometryMesh&);
  };
};  // namespace hand_fit

#endif  // HAND_MODEL_HAND_MODEL_GEOMETRY_HESH_HEADER
