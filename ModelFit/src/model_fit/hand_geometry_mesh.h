//#ifndef HAND_FIT_
//  hand_geometry_mesh.h
//
//  Created by Jonathan Tompson on 10/30/12.
//
//  A desnse modeled hand mesh
//

#ifndef MODEL_FIT_HAND_GEOMETRY_MESH_HEADER
#define MODEL_FIT_HAND_GEOMETRY_MESH_HEADER

#include "renderer/open_gl_common.h"  // GLfloat
#include "kinect_interface/hand_net/hand_model.h"
#include "model_fit/pose_model.h"
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector.h"
#include "jtil/data_str/pair.h"

#if defined(__APPLE__)
  #define HAND_MODEL_PATH string("./../../../../../../../../../models/")
#else
  #define HAND_MODEL_PATH string("./models/")
#endif

#define LOAD_HAND_MESH_JFILE  // Much faster and more compact format!
#define LHAND_MODEL_FILE "hand_palm_parent_medium_wrist.dae"
#define LHAND_MODEL_JFILE "hand_palm_parent_medium_wrist.jbin"
#define RHAND_MODEL_FILE "hand_palm_parent_medium_wrist_right.dae"
#define RHAND_MODEL_JFILE "hand_palm_parent_medium_wrist_right.jbin"

#define PSO_RAD_FINGERS 0.40f  // Search radius in frac of min - max coeff - Def 0.4
#define PSO_RAD_WRIST 0.40f
#define PSO_RAD_THUMB 0.40f
#define PSO_RAD_EULER 0.40f
#define PSO_RAD_POSITION 25  // Absolute value in mm

#define NUM_SPHERES kinect_interface::hand_net::HandSphereIndices::NUM_BOUNDING_SPHERES

namespace renderer { class Geometry; }
namespace renderer { struct BoneFileInfo; }
namespace renderer { struct Bone; }
namespace renderer { class Renderer; }
namespace renderer { class BoundingSphere; }

namespace model_fit {
  class ModelRenderer;

  class HandGeometryMesh : public PoseModel {
  public:
    // Constructor / Destructor
    HandGeometryMesh(kinect_interface::hand_net::HandType hand_type, 
      renderer::Renderer* g_renderer, ModelRenderer* g_hand_renderer);
    virtual ~HandGeometryMesh();

    // Call before rendering hand depth maps:
    virtual void updateMatrices(const float* coeff);
    virtual void updateHeirachyMatrices();
    virtual void fixBoundingSphereMatrices();
    virtual inline renderer::Geometry* scene_graph() { return scene_graph_; }
    
    jtil::data_str::Vector<renderer::BoundingSphere*>& bspheres() { return bspheres_; }

    void handCoeff2CoeffConvnet(kinect_interface::hand_net::HandModel* hand, 
      float* coeff_convnet, const jtil::math::Int4& hand_pos_wh, 
      const jtil::math::Float3& uvd_com, const jtil::math::Float4x4& proj_mat);

    virtual void renderStackReset();
    virtual renderer::Geometry* renderStackPop();
    virtual bool renderStackEmpty();

    // setRendererAttachement - called when ModelFit wants to detach the model
    // from the global renderer
    virtual void setRendererAttachement(const bool renderer_attachment);

    static const float coeff_min_limit[HAND_NUM_COEFF];
    static const float coeff_max_limit[HAND_NUM_COEFF];
    static const float coeff_penalty_scale[HAND_NUM_COEFF];
    static const bool angle_coeffs[HAND_NUM_COEFF];
    static const float pso_radius_c[HAND_NUM_COEFF];

  private:
    // Note all geometry is attached to the global renderer's scene graph and
    // therefore we transfer ownership of the memory to it.
    renderer::Geometry* scene_graph_;  // The renderable geometry - Not owned here
    renderer::BoneFileInfo* bones_in_file_;  // Not owned here
    jtil::data_str::Vector<jtil::math::Float4x4> rest_transforms_;  // per bone
    jtil::data_str::Vector<renderer::BoundingSphere*> bspheres_;  // Attached to scene graph!
    uint32_t bone_wrist_index_;
    uint32_t bone_palm_index_;
    uint32_t bone_thumb_1_index_;  // tip
    uint32_t bone_thumb_2_index_;
    uint32_t bone_thumb_3_index_;  // base
    uint32_t bone_finger_1_index_[4];
    uint32_t bone_finger_2_index_[4];
    uint32_t bone_finger_3_index_[4];
    uint32_t bone_finger_4_index_[4];
    renderer::Geometry* mesh_;  // Not owned here
    bool renderer_attachment_;  // whether or not the model is attached to the 
                                // global renderer's scene graph

    // Temp matricies are not static to be thread safe.
    jtil::math::Float4x4 mat_tmp1;
    jtil::math::Float4x4 mat_tmp2;
    jtil::math::Float4x4 mat_tmp3;

    static uint32_t sph_bone_ind_[NUM_SPHERES];

    void createHandGeometry(renderer::Renderer* g_renderer,
      ModelRenderer* g_hand_renderer, 
      kinect_interface::hand_net::HandType type);

    // Copy of the Renderer's stack interface methods, I'd rather duplicate
    // them and keep the renderer seperate.
    jtil::data_str::Vector<renderer::Geometry*> render_stack_;

    void euler2RotMatGM(jtil::math::Float4x4& a, const float x_angle, 
      const float y_angle, const float z_angle);
    void rotateMatZAxisGM(jtil::math::Float4x4& ret, const float angle);
    void rotateMatYAxisGM(jtil::math::Float4x4& ret, const float angle);
    void rotateMatXAxisGM(jtil::math::Float4x4& ret, const float angle);

    void extractPositionForConvnet(kinect_interface::hand_net::HandModel* hand, 
      float* coeff_convnet, const jtil::math::Int4& hand_pos_wh, 
      const jtil::math::Float3& uvd_com, const uint32_t b_sphere_index, 
      const uint32_t convnet_U_index, const jtil::math::Float4x4& proj_mat);
    void calcHandImageUVFromXYZ(jtil::math::Float3& xyz_pos, 
      jtil::math::Float2& uv_pos, const jtil::math::Int4& hand_pos_wh,
      const jtil::math::Float4x4& proj_mat);

    // Non-copyable, non-assignable.
    HandGeometryMesh(HandGeometryMesh&);
    HandGeometryMesh& operator=(const HandGeometryMesh&);
  };
};  // namespace hand_fit

#endif  // MODEL_FIT_HAND_GEOMETRY_MESH_HEADER
