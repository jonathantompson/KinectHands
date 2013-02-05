//
//  hand_model_geometry_mesh.h
//
//  Created by Jonathan Tompson on 10/30/12.
//
//  The ball and cylinder hand model

#ifndef HAND_MODEL_HAND_MODEL_GEOMETRY_HESH_HEADER
#define HAND_MODEL_HAND_MODEL_GEOMETRY_HESH_HEADER

#include "renderer/open_gl_common.h"  // GLfloat
#include "hand_model/hand_model.h"
#include "hand_model/hand_model_geometry.h"
#include "math/math_types.h"
#include "data_str/vector.h"
#include "data_str/pair.h"
#include "Eigen"

namespace renderer { class Geometry; }
namespace renderer { struct BoneFileInfo; }
namespace renderer { struct Bone; }
namespace renderer { class Renderer; }

namespace hand_model {
  class HandModelRenderer;

  class HandModelGeometryMesh : public HandModelGeometry {
  public:
    // Constructor / Destructor
    HandModelGeometryMesh(HandType hand_type, renderer::Renderer* g_renderer,
      HandModelRenderer* g_hand_renderer);
    virtual ~HandModelGeometryMesh();

    // Call before rendering hand depth maps:
    virtual void updateMatrices(const Eigen::MatrixXf& coeff);
    virtual void updateHeirachyMatrices();
    virtual void fixBoundingSphereMatrices();
    virtual inline renderer::Geometry* scene_graph() { return scene_graph_; }

    virtual void renderStackReset();
    virtual renderer::Geometry* renderStackPop();
    virtual bool renderStackEmpty();

  private:
    // Note all geometry is attached to the global renderer's scene graph and
    // therefore we transfer ownership of the memory to it.
    renderer::Geometry* scene_graph_;  // The renderable geometry - Not owned here
    renderer::BoneFileInfo* bones_in_file_;  // Not owned here
    data_str::Vector<math::Float4x4> rest_transforms_;  // per bone
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
    math::Float4x4 mat_tmp1;
    math::Float4x4 mat_tmp2;
    math::Float4x4 mat_tmp3;

    // Bones aren't in the correct position (need offsets)
    static const float sph_off_[NUM_BOUNDING_SPHERES * 3];  
    static const float sph_size_[NUM_BOUNDING_SPHERES];
    static uint32_t sph_bone_ind_[NUM_BOUNDING_SPHERES];

    void createHandGeometry(renderer::Renderer* g_renderer,
      HandModelRenderer* g_hand_renderer, HandType type);

    // Copy of the Renderer's stack interface methods, I'd rather duplicate
    // them and keep the renderer seperate.
    data_str::Vector<renderer::Geometry*> render_stack_;

    // Non-copyable, non-assignable.
    HandModelGeometryMesh(HandModelGeometryMesh&);
    HandModelGeometryMesh& operator=(const HandModelGeometryMesh&);
  };
};  // namespace hand_model

#endif  // HAND_MODEL_HAND_MODEL_GEOMETRY_HESH_HEADER
