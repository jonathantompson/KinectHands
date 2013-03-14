//
//  hand_model_renderer.h
//
//  Created by Jonathan Tompson on 10/02/12.
//
//  Renderers a hand model in OpenGL 3.2 using my simple forward-renderer class.

#ifndef HAND_MODEL_HAND_MODEL_RENDERER_HEADER
#define HAND_MODEL_HAND_MODEL_RENDERER_HEADER

#include "renderer/open_gl_common.h"  // GLfloat
#include "hand_fit/hand_geometry.h"  // NUM_BOUNDING_SPHERES
#include "kinect_interface/hand_net/hand_model.h"
#include "hand_fit/hand_fit.h"
#include "jtil/math/math_types.h"
#include "jtil/data_str/pair.h"
#include "jtil/data_str/vector.h"
#include "jtil/data_str/vector_managed.h"
#include "Eigen"

// Note, raw depth from the kinect is in mm
#define HAND_CAMERA_VIEW_PLANE_NEAR 10.0f
#define HAND_CAMERA_VIEW_PLANE_FAR 3000.0f
// http://www.ros.org/wiki/kinect_calibration/technical
#define HAND_CAMERA_FOV_HOR 57.8f  // Kinect focal length is 585pix (640pix width)
                                   // RGB FOV is 62.7
// #define HAND_CAMERA_FOV 43.35f  // Actual value
#define HAND_CAMERA_FOV 45.25f  // This value works better

// #define SAVE_DEBUG_TEXTURES  // --> Use 'eval_hand_debug_textures.m'
#define NTILES_X 8  // Don't change these --> If you do you'll have to change 
#define NTILES_Y 8  //                        the downsample chain
#define NTILES (NTILES_X * NTILES_Y)

#ifdef DEPTH_ONLY_RESIDUE_FUNC
  #define RESIDUE_INT_FORMAT GL_R32F
  #define RESIDUE_FORMAT GL_RED
  #define RESIDUE_TYPE GL_FLOAT
  #define UNION 1000000.0f  // A made up scaling factor
#else
  #define RESIDUE_INT_FORMAT GL_RGB32F
  #define RESIDUE_FORMAT GL_RGB
  #define RESIDUE_TYPE GL_FLOAT
#endif
#define LINEAR_PENALTY  // otherwise quadratic

// #define PERFORM_BROAD_PHASE_COLLISION_TEST

namespace renderer { class Camera; }
namespace renderer { class Geometry; }
namespace renderer { class Renderer; }
namespace renderer { class TextureRenderable; }
namespace renderer { class Texture; }
namespace renderer { class Shader; }
namespace renderer { class ShaderProgram; }
namespace renderer { class GeometryTexturedBonedMesh; }
namespace renderer { class GeometryColoredMesh; }
namespace renderer { class BoundingSphere; }
namespace renderer { class GeometryColoredBonedMesh; }
namespace windowing { class Window; }

namespace hand_fit {

  class HandGeometry;

  class HandRenderer {
  public:
    // Constructor / Destructor
    HandRenderer(renderer::Renderer* g_renderer, bool left, bool right);
    ~HandRenderer();

    // Call before rendering hand depth maps:
    void uploadKinectDepth(int16_t* kinect_depth);  

    void updateMatrices(const float* coeff, 
      kinect_interface::hand_net::HandType hand_type);
    void visualizeDepthMap(windowing::Window* wnd, bool color = false);
    void drawDepthMap(const Eigen::MatrixXf& coeff, 
      kinect_interface::hand_net::HandModel** hands, 
      uint32_t num_hands, bool color = false);
    void drawDepthMapTiled(jtil::data_str::Vector<float>& residues,
      jtil::data_str::VectorManaged<Eigen::MatrixXf>& coeff, 
      kinect_interface::hand_net::HandModel** hands, 
      uint32_t num_hands_per_tile);
    float calculateResidualDataTerm();
    void calculateResidualDataTermTiled(jtil::data_str::Vector<float>& residues);
    float calcInterpenetrationTerm();
    void extractDepthMap(float* depth_vals);
    void setRendererAttachement(bool render_hand);
    void preBindUniforms();

    inline HandGeometry* l_hand_geom() { return l_hand_geom_; }
    inline HandGeometry* r_hand_geom() { return r_hand_geom_; }
    inline renderer::TextureRenderable* depth_texture() { return depth_texture_; }
    inline renderer::TextureRenderable* cdepth_texture() { return cdepth_texture_; }
    inline float* depth_tmp() { return depth_tmp_; }
    inline renderer::Camera* camera() { return camera_; }
    HandGeometry* geom(kinect_interface::hand_net::HandType hand_type);

    void addBSphere(renderer::BoundingSphere* sph);

    // Manually update internal data --> Only touch if you know what it is!
    void updateHeirachyMatrices(kinect_interface::hand_net::HandType hand_type);
    void fixBoundingSphereMatrices(kinect_interface::hand_net::HandType hand_type);

    void handCoeff2CoeffConvnet(kinect_interface::hand_net::HandModel* hand,
      float* coeff_convnet, const jtil::math::Int4& hand_pos_wh, 
      const jtil::math::Float3& uvd_com);

  private:
    bool render_hand_;
    renderer::Renderer* g_renderer_;  // Not owned here
    renderer::Camera* camera_;
    
    HandGeometry* l_hand_geom_;
    HandGeometry* r_hand_geom_;
    
    renderer::TextureRenderable* depth_texture_;  // 640 x 480
    renderer::TextureRenderable* depth_texture_tiled_;  // 5120 x 3840 (8x8)
    renderer::TextureRenderable* residue_texture_1_;   // 640 x 480
    renderer::TextureRenderable* residue_texture_2_;   // 320 x 240
    renderer::TextureRenderable* residue_texture_4_;   // 160 x 120
    renderer::TextureRenderable* residue_texture_16_;  // 40 x 30
    renderer::TextureRenderable* residue_texture_20_;  // 32 x 24
    renderer::TextureRenderable* residue_texture_32_;  // 20 x 15
    renderer::TextureRenderable* residue_texture_160_;  // 4 x 3
    renderer::TextureRenderable* residue_texture_x2_;   // 1280 x 960
    renderer::TextureRenderable* residue_texture_x8_;   // 5120 x 3840
    renderer::Texture* kinect_depth_texture_;
    renderer::Texture* kinect_depth_texture_tiled_;
    renderer::TextureRenderable* cdepth_texture_;  // depth and color

    float* depth_tmp_;  // DEPTH_IMAGE_DIM * NTILES

    // Shader for rendering depth
    renderer::Shader* v_shader_depth_;
    renderer::Shader* f_shader_depth_;
    renderer::ShaderProgram* sp_depth_;
    GLint h_PVW_mat_sp_depth_;
    GLint h_VW_mat_sp_depth_;

    // Shader for rendering colored depth
    renderer::Shader* v_shader_cdepth_;
    renderer::Shader* f_shader_cdepth_;
    renderer::ShaderProgram* sp_cdepth_;
    GLint h_PVW_mat_sp_cdepth_;
    GLint h_VW_mat_sp_cdepth_;

    // Shader for rendering depth from skinned mesh
    renderer::Shader* v_shader_depth_skinned_;
    renderer::Shader* f_shader_depth_skinned_;
    renderer::ShaderProgram* sp_depth_skinned_;
    GLint h_PVW_mat_sp_depth_skinned_;
    GLint h_VW_mat_sp_depth_skinned_;
    GLint h_bone_trans_sp_depth_skinned_[MAX_BONE_COUNT];

    // Shader for rendering depth and face color from skinned mesh
    renderer::Shader* v_shader_cdepth_skinned_;
    renderer::Shader* f_shader_cdepth_skinned_;
    renderer::ShaderProgram* sp_cdepth_skinned_;
    GLint h_PVW_mat_sp_cdepth_skinned_;
    GLint h_VW_mat_sp_cdepth_skinned_;
    GLint h_bone_trans_sp_cdepth_skinned_[MAX_BONE_COUNT];

    // Shader to visualize depth
    renderer::Shader* v_shader_visualize_depth_;
    renderer::Shader* f_shader_visualize_depth_;
    renderer::ShaderProgram* sp_visualize_depth_;
    GLint h_visualize_depth_texture_sampler_;
    GLint h_f_depth_min_visualize_depth_;
    GLint h_f_depth_max_visualize_depth_;

    // Shader to visualize colored depth
    renderer::Shader* v_shader_visualize_cdepth_;
    renderer::Shader* f_shader_visualize_cdepth_;
    renderer::ShaderProgram* sp_visualize_cdepth_;
    GLint h_visualize_cdepth_texture_sampler_;
    GLint h_f_depth_min_visualize_cdepth_;
    GLint h_f_depth_max_visualize_cdepth_;

    // Shader for calculating residue
    renderer::Shader* v_shader_residue_calc_;
    renderer::Shader* f_shader_residue_calc_;
    renderer::ShaderProgram* sp_residue_calc_;
    GLint h_residue_calc_kinect_depth_;
    GLint h_residue_calc_synth_depth_;
    GLint h_residue_calc_max_depth_;
    GLint h_residue_calc_texel_dim_;

    jtil::math::Float4x4 VW_mat_;
    jtil::math::Float4x4 PVW_mat_;

    jtil::data_str::Vector<float> matrix_data_;  // concatenated matrix data

    jtil::data_str::Vector<renderer::BoundingSphere*> bsph_;  // Not owned here
    
    // Bounding sphere indices sorted along each axis to create intervals
    uint32_t bsph_sx[2 * NUM_BOUNDING_SPHERES];  // start xaxis
    uint32_t bsph_sy[2 * NUM_BOUNDING_SPHERES];  // start xaxis
    uint32_t bsph_sz[2 * NUM_BOUNDING_SPHERES];  // start xaxis
    uint32_t bsph_fx[2 * NUM_BOUNDING_SPHERES];  // finish xaxis
    uint32_t bsph_fy[2 * NUM_BOUNDING_SPHERES];  // finish xaxis
    uint32_t bsph_fz[2 * NUM_BOUNDING_SPHERES];  // finish xaxis

    // Store O(N^2) collision pairs --> Could be more efficient, but OK for now
    bool collision_pairs_xaxis[4*NUM_BOUNDING_SPHERES*NUM_BOUNDING_SPHERES];
    bool collision_pairs_yaxis[4*NUM_BOUNDING_SPHERES*NUM_BOUNDING_SPHERES];
    bool collision_pairs_zaxis[4*NUM_BOUNDING_SPHERES*NUM_BOUNDING_SPHERES];
    jtil::data_str::Vector<uint32_t> active_list_;
    jtil::data_str::Vector<jtil::data_str::Pair<uint32_t, uint32_t>> collisions_;

    void renderTexturedBonedMesh(renderer::GeometryTexturedBonedMesh* geom, 
      bool color = false);
    void renderColoredMesh(renderer::GeometryColoredMesh* geom, 
      bool color = false);
    void renderColoredBonedMesh(renderer::GeometryColoredBonedMesh* geom, 
      bool color = false);

    void insertionSortAxisExtents(uint32_t* arr, 
      uint32_t iextent);
    void findIntersections(uint32_t* min, uint32_t* max, uint32_t iextent_min, 
      uint32_t iextent_max, bool* collisions);
    void findCollisions();

    void drawDepthMapInternal(const Eigen::MatrixXf& coeff, 
      kinect_interface::hand_net::HandModel** hands, 
      uint32_t num_hands, bool color, bool tiled);

    void HandRenderer::calcHandImageUVFromXYZ(jtil::math::Float3& xyz_pos, 
      jtil::math::Float2& uv_pos, const jtil::math::Int4& hand_pos_wh);

    // Non-copyable, non-assignable.
    HandRenderer(HandRenderer&);
    HandRenderer& operator=(const HandRenderer&);
  };

};  // unnamed namespace

#endif  // HAND_MODEL_HAND_MODEL_RENDERER_HEADER
