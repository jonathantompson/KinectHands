#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include "model_fit/model_renderer.h"
#include "model_fit/model_fit.h"
#include "model_fit/bounding_sphere.h"
#include "model_fit/pose_model.h"
#include "kinect_interface/hand_net/hand_model_coeff.h"  // NSPH_PER_GROUP
#include "renderer/open_gl_common.h"
#include "renderer/renderer.h"
#include "renderer/colors.h"
#include "renderer/geometry/geometry.h"
#include "renderer/geometry/geometry_manager.h"
#include "renderer/geometry/bone_info.h"
#include "renderer/geometry/geometry_vertices.h"
#include "renderer/geometry/geometry_colored_mesh.h"
#include "renderer/geometry/geometry_colored_boned_mesh.h"
#include "renderer/geometry/geometry_textured_boned_mesh.h"
#include "kinect_interface/kinect_interface.h"  // depth_dim
#include "jtil/math/pso_parallel.h"
#include "renderer/texture/texture.h"
#include "renderer/texture/texture_renderable.h"
#include "renderer/shader/shader.h"
#include "renderer/shader/shader_program.h"
#include "renderer/camera/camera.h"
#include "jtil/windowing/window.h"
#include "jtil/file_io/file_io.h"
#include "jtil/data_str/vector.h"
#include "jtil/math/math_base.h"
#include "renderer/gl_state.h"
#include "kinect_interface/kinect_interface.h"  // depth_vfov

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::data_str;
using namespace jtil::math;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using namespace renderer;
using namespace kinect_interface;

namespace model_fit {
  
  ModelRenderer::ModelRenderer(const uint32_t num_cameras) {
    // Note: I allocate a square texture so that it makes integration during
    // function evaluation on the GPU very fast.
    tex_size_ = jtil::math::nextPO2(std::max<uint32_t>(depth_w, depth_h));
    uint32_t tex_dim = tex_size_ * tex_size_;

    num_cameras_ = num_cameras;
    depth_tmp_ = new float*[num_cameras_];
    for (uint32_t i = 0; i < num_cameras_; i++) {
      depth_tmp_[i] = new float[tex_dim * NTILES];
    }
    FloatQuat eye_rot; eye_rot.identity();
    Float3 eye_pos(0, 0, 0);
    float fov_vert_deg = depth_vfov;
    cameras_ = new Camera*[num_cameras_];
    for (uint32_t i = 0; i < num_cameras_; i++) {
      cameras_[i] = new Camera(&eye_rot, &eye_pos, depth_w, depth_h, 
        fov_vert_deg, -10.0f, -3000.0f);
      cameras_[i]->updateView();
      cameras_[i]->updateProjection();
    }
    
    g_renderer_ = Renderer::g_renderer();  // Not owned here.
    
    // Renderable texture
    depth_texture_ = new TextureRenderable(GL_R32F, tex_size_,
      tex_size_, GL_RED, GL_FLOAT, 1, true);
    cdepth_texture_ = new TextureRenderable(GL_RGBA32F, tex_size_,
      tex_size_, GL_RGBA, GL_FLOAT, 1, true);
    depth_texture_tiled_ = new TextureRenderable(GL_R32F, 
      tex_size_ * NTILES_DIM, tex_size_ * NTILES_DIM, GL_RED, 
      GL_FLOAT, 1, true);
    
    // Shader to create the depth image
    v_shader_depth_ = new Shader("shaders/depth.vert", ShaderType::VERTEX_SHADER);
    f_shader_depth_ = new Shader("shaders/depth.frag", ShaderType::FRAGMENT_SHADER);
    sp_depth_ = new ShaderProgram(v_shader_depth_, f_shader_depth_);
    sp_depth_->bindVertShaderInputLocation(Renderer::pos);
    sp_depth_->link();
    h_PVW_mat_sp_depth_ = sp_depth_->getUniformLocation("PVW_mat");
    h_VW_mat_sp_depth_ = sp_depth_->getUniformLocation("VW_mat");

    // Shader to create the colored depth image
    v_shader_cdepth_ = new Shader("shaders/cdepth.vert", ShaderType::VERTEX_SHADER);
    f_shader_cdepth_ = new Shader("shaders/cdepth.frag", ShaderType::FRAGMENT_SHADER);
    sp_cdepth_ = new ShaderProgram(v_shader_cdepth_, f_shader_cdepth_);
    sp_cdepth_->bindVertShaderInputLocation(Renderer::pos);
    sp_cdepth_->bindVertShaderInputLocation(Renderer::col);
    sp_cdepth_->link();
    h_PVW_mat_sp_cdepth_ = sp_cdepth_->getUniformLocation("PVW_mat");
    h_VW_mat_sp_cdepth_ = sp_cdepth_->getUniformLocation("VW_mat");

    // Shader for rendering depth from skinned mesh
#ifdef LINEAR_BLEND_SKINNING
    v_shader_depth_skinned_ = new Shader("shaders/depth_skinned_lbs.vert", ShaderType::VERTEX_SHADER);
#else
    v_shader_depth_skinned_ = new Shader("shaders/depth_skinned.vert", ShaderType::VERTEX_SHADER);
#endif
    f_shader_depth_skinned_ = new Shader("shaders/depth.frag", ShaderType::FRAGMENT_SHADER);
    sp_depth_skinned_ = new ShaderProgram(v_shader_depth_skinned_, f_shader_depth_skinned_);
    sp_depth_skinned_->bindVertShaderInputLocation(Renderer::pos);
    sp_depth_skinned_->bindVertShaderInputLocation(Renderer::bone_ids_03);
    sp_depth_skinned_->bindVertShaderInputLocation(Renderer::bone_weights_03);
    //sp_depth_skinned_->bindVertShaderInputLocation(Renderer::bone_ids_47);
    //sp_depth_skinned_->bindVertShaderInputLocation(Renderer::bone_weights_47);
    sp_depth_skinned_->link();
    h_PVW_mat_sp_depth_skinned_ = sp_depth_skinned_->getUniformLocation("PVW_mat");
    h_VW_mat_sp_depth_skinned_ = sp_depth_skinned_->getUniformLocation("VW_mat");
    std::stringstream ss;
    for (uint32_t i = 0; i < MAX_BONE_COUNT; i++) {
      ss.str(std::string());  // Clear the string stream
      ss << "bone_trans[" << i << "]";
      h_bone_trans_sp_depth_skinned_[i] = 
        sp_depth_skinned_->getUniformLocation(ss.str().c_str());
    }

    // Shader for rendering depth and face color from skinned mesh
#ifdef LINEAR_BLEND_SKINNING
    v_shader_cdepth_skinned_ = new Shader("shaders/cdepth_skinned_lbs.vert", ShaderType::VERTEX_SHADER);
#else
    v_shader_cdepth_skinned_ = new Shader("shaders/cdepth_skinned.vert", ShaderType::VERTEX_SHADER);
#endif
    f_shader_cdepth_skinned_ = new Shader("shaders/cdepth.frag", ShaderType::FRAGMENT_SHADER);
    sp_cdepth_skinned_ = new ShaderProgram(v_shader_cdepth_skinned_, f_shader_cdepth_skinned_);
    sp_cdepth_skinned_->bindVertShaderInputLocation(Renderer::pos);
    sp_cdepth_skinned_->bindVertShaderInputLocation(Renderer::col);
    sp_cdepth_skinned_->bindVertShaderInputLocation(Renderer::bone_ids_03);
    sp_cdepth_skinned_->bindVertShaderInputLocation(Renderer::bone_weights_03);
    //sp_cdepth_skinned_->bindVertShaderInputLocation(Renderer::bone_ids_47);
    //sp_cdepth_skinned_->bindVertShaderInputLocation(Renderer::bone_weights_47);
    sp_cdepth_skinned_->link();
    h_PVW_mat_sp_cdepth_skinned_ = sp_cdepth_skinned_->getUniformLocation("PVW_mat");
    h_VW_mat_sp_cdepth_skinned_ = sp_cdepth_skinned_->getUniformLocation("VW_mat");
    for (uint32_t i = 0; i < MAX_BONE_COUNT; i++) {
      ss.str(std::string());  // Clear the string stream
      ss << "bone_trans[" << i << "]";
      h_bone_trans_sp_cdepth_skinned_[i] = 
        sp_cdepth_skinned_->getUniformLocation(ss.str().c_str());
    }
    
    // Shader to visualize the depth image (on screen)
    v_shader_visualize_depth_ = new Shader("shaders/visualize_depth.vert", ShaderType::VERTEX_SHADER);
    f_shader_visualize_depth_ = new Shader("shaders/visualize_depth.frag", ShaderType::FRAGMENT_SHADER);
    sp_visualize_depth_ = new ShaderProgram(v_shader_visualize_depth_, f_shader_visualize_depth_);
    sp_visualize_depth_->bindVertShaderInputLocation(Renderer::pos);
    sp_visualize_depth_->link();
    h_visualize_depth_texture_sampler_ = sp_visualize_depth_->getUniformLocation("f_texture_sampler");
    h_f_depth_min_visualize_depth_ = sp_visualize_depth_->getUniformLocation("f_depth_min");
    h_f_depth_max_visualize_depth_ = sp_visualize_depth_->getUniformLocation("f_depth_max");

    // Shader to visualize the colored depth image (on screen)
    v_shader_visualize_cdepth_ = new Shader("shaders/visualize_cdepth.vert", ShaderType::VERTEX_SHADER);
    f_shader_visualize_cdepth_ = new Shader("shaders/visualize_cdepth.frag", ShaderType::FRAGMENT_SHADER);
    sp_visualize_cdepth_ = new ShaderProgram(v_shader_visualize_cdepth_, f_shader_visualize_cdepth_);
    sp_visualize_cdepth_->bindVertShaderInputLocation(Renderer::pos);
    sp_visualize_cdepth_->link();
    h_visualize_cdepth_texture_sampler_ = sp_visualize_cdepth_->getUniformLocation("f_texture_sampler");
    h_f_depth_min_visualize_cdepth_ = sp_visualize_cdepth_->getUniformLocation("f_depth_min");
    h_f_depth_max_visualize_cdepth_ = sp_visualize_cdepth_->getUniformLocation("f_depth_max");

    // Shader to calculate the residue
    v_shader_residue_calc_ = new Shader("shaders/residue_calc.vert", ShaderType::VERTEX_SHADER);
    f_shader_residue_calc_ = new Shader("shaders/residue_calc_depth_only.frag", ShaderType::FRAGMENT_SHADER);
    sp_residue_calc_ = new ShaderProgram(v_shader_residue_calc_, f_shader_residue_calc_);
    sp_residue_calc_->bindVertShaderInputLocation(Renderer::pos);
    sp_residue_calc_->link();
    h_residue_calc_kinect_depth_ = sp_residue_calc_->getUniformLocation("kinect_depth");
    h_residue_calc_synth_depth_ = sp_residue_calc_->getUniformLocation("synth_depth");
    h_residue_calc_max_depth_ = sp_residue_calc_->getUniformLocation("max_depth");
    //h_residue_calc_texel_dim_ = sp_residue_calc_->getUniformLocation("texel_dim");

    // Allocate enough textures so that we have the whole pyramid going down
    uint32_t i = tex_size_;
    while (i >= 1) {
      TextureRenderable* tex = new TextureRenderable(RESIDUE_INT_FORMAT, 
        i, i, RESIDUE_FORMAT, RESIDUE_TYPE, 1, false);
      residue_tex_.pushBack(tex);
      if (i / 4 <= 0) {
        i /= 2;
      } else {
        i /= 4;
      }
    }

    // Now we also need tiled residue textures.  There's lots of overlap here,
    // ie, we could just reuse some of the textures above.  But it's much
    // easier from an engineering perspective to just reuse the textures
    i = tex_size_ * NTILES_DIM;
    while (i >= NTILES_DIM) {
      TextureRenderable* tex = new TextureRenderable(RESIDUE_INT_FORMAT, 
        i, i, RESIDUE_FORMAT, RESIDUE_TYPE, 1, false);
      tiled_residue_tex_.pushBack(tex);
      if (i / 4 < NTILES_DIM) {
        i /= 2;
      } else {
        i /= 4;
      }
    }

    kinect_depth_textures_ = new Texture*[num_cameras_];
    kinect_depth_textures_tiled_ = new Texture*[num_cameras_];
    for (uint32_t i = 0; i < num_cameras_; i++) {
      kinect_depth_textures_[i] = NULL;
      kinect_depth_textures_tiled_[i] = NULL;
    }
  }

  ModelRenderer::~ModelRenderer() {
    if (depth_tmp_) {
      for (uint32_t i = 0; i < num_cameras_; i++) {
        SAFE_DELETE_ARR(depth_tmp_[i]);
      }
    }
    SAFE_DELETE_ARR(depth_tmp_);
    if (cameras_) {
      for (uint32_t i = 0; i < num_cameras_; i++) {
        SAFE_DELETE(cameras_[i]);
      }
    }
    if (kinect_depth_textures_) {
      for (uint32_t i = 0; i < num_cameras_; i++) {
        SAFE_DELETE(kinect_depth_textures_[i]);
      }
    }
    SAFE_DELETE_ARR(kinect_depth_textures_);
    if (kinect_depth_textures_tiled_) {
      for (uint32_t i = 0; i < num_cameras_; i++) {
        SAFE_DELETE(kinect_depth_textures_tiled_[i]);
      }
    }
    SAFE_DELETE_ARR(kinect_depth_textures_tiled_);
    SAFE_DELETE_ARR(cameras_);
    SAFE_DELETE(depth_texture_);
    SAFE_DELETE(depth_texture_tiled_);
    for (uint32_t i = 0; i < residue_tex_.size(); i++) {
      SAFE_DELETE(residue_tex_[i]);
    }
    for (uint32_t i = 0; i < tiled_residue_tex_.size(); i++) {
      SAFE_DELETE(tiled_residue_tex_[i]);
    }
    SAFE_DELETE(cdepth_texture_);
    SAFE_DELETE(sp_depth_);
    SAFE_DELETE(v_shader_depth_);
    SAFE_DELETE(f_shader_depth_);
    SAFE_DELETE(sp_cdepth_);
    SAFE_DELETE(v_shader_cdepth_);
    SAFE_DELETE(f_shader_cdepth_);
    SAFE_DELETE(sp_cdepth_skinned_);
    SAFE_DELETE(v_shader_cdepth_skinned_);
    SAFE_DELETE(f_shader_cdepth_skinned_);
    SAFE_DELETE(sp_depth_skinned_);
    SAFE_DELETE(v_shader_depth_skinned_);
    SAFE_DELETE(f_shader_depth_skinned_);
    SAFE_DELETE(sp_visualize_depth_);
    SAFE_DELETE(v_shader_visualize_depth_);
    SAFE_DELETE(f_shader_visualize_depth_);
    SAFE_DELETE(sp_visualize_cdepth_);
    SAFE_DELETE(v_shader_visualize_cdepth_);
    SAFE_DELETE(f_shader_visualize_cdepth_);
    SAFE_DELETE(sp_residue_calc_);
    SAFE_DELETE(f_shader_residue_calc_);
    SAFE_DELETE(v_shader_residue_calc_);
  }

  void ModelRenderer::drawDepthMap(const float* coeff, 
    const uint32_t num_coeff_per_model, PoseModel** models, 
    const uint32_t num_models, const uint32_t i_camera, const bool color) {
    GLState::glsViewport(0, 0, depth_w, depth_h);
    // NOTE: We will perform a buffer clear in drawDepthMapInternal
    drawDepthMapInternal(coeff, num_coeff_per_model, models, num_models, 
      i_camera, color, false);
  }

  void ModelRenderer::drawDepthMapTiled(Vector<float*>& coeff, 
    const uint32_t num_coeff_per_model, PoseModel** models, 
    const uint32_t num_models, const uint32_t i_camera, 
    Vector<float>* interpenetration, const uint32_t max_num_interpen_groups) {
    depth_texture_tiled_->begin();

    GLState::glsClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    uint32_t nimages = coeff.size();
    if (nimages > NTILES) {
      throw std::runtime_error("Too many images to draw tiled!");
    }
    for (uint32_t i = 0; i < nimages; i++) {
      // Set up the viewport
      uint32_t xpos = i % NTILES_DIM;
      uint32_t ypos = i / NTILES_DIM;
      GLState::glsViewport(xpos * tex_size_, ypos * tex_size_, 
        depth_w, depth_h);
      drawDepthMapInternal(coeff[i], num_coeff_per_model, models, num_models,
        i_camera, false, true);
      // Before destorying the matrix heirachy, calculate the interpenetration
      if (interpenetration) {
        (*interpenetration)[i] += 
          calcInterpenetrationTerm(max_num_interpen_groups);
      }
    }

    depth_texture_tiled_->end();
  }

  void ModelRenderer::drawDepthMapInternal(const float* coeff, 
    const uint32_t num_coeff_per_model, PoseModel** models, 
    const uint32_t num_models, const uint32_t i_camera, const bool color, 
    const bool tiled) {
    GLState::glsClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    GLState::glsDepthMask(GL_TRUE);
    GLState::glsDisable(GL_BLEND);
    GLState::glsEnable(GL_DEPTH_TEST);
    GLState::glsDepthFunc(GL_LESS);
    GLState::glsPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    GLState::glsEnable(GL_CULL_FACE);

    // NOTE.  In depth.vert we are setting "gl_Position.y *= -1" so that the
    //        output image is flipped.  For this reason we actually need to
    //        unwind the polygon order!
    GLState::glsCullFace(GL_FRONT);

    if (!tiled) {
      if (color) {
        cdepth_texture_->begin(false);
      } else {
        depth_texture_->begin(false);
      }
      GLState::glsClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    }

    // Render each model individually:
    for (uint32_t i = 0; i < num_models && models[i] != NULL; i++) {
      // Updates the individual nodes
      const float* coeff_data = &coeff[i * num_coeff_per_model];

      // Update the matricies
      models[i]->updateMatrices(coeff_data);  
      models[i]->updateHeirachyMatrices();
      models[i]->fixBoundingSphereMatrices();

      // Render all the colored mesh elements
      if (color) {
        sp_cdepth_->useProgram();
      } else {
        sp_depth_->useProgram();
      }
      models[i]->renderStackReset();
      while (!models[i]->renderStackEmpty()) {
        Geometry* cur_geom = models[i]->renderStackPop();
        if (cur_geom->type() == renderer::GeometryType::GEOMETRY_COLORED_MESH) {
          renderColoredMesh((GeometryColoredMesh*) cur_geom, i_camera, color);
        }
      }

      // Render all the textured and colored boned mesh elements
      if (color) {
        sp_cdepth_skinned_->useProgram();
      } else {
        sp_depth_skinned_->useProgram();
      }
      models[i]->renderStackReset();
      while (!models[i]->renderStackEmpty()) {
        Geometry* cur_geom = models[i]->renderStackPop();
        if (cur_geom->type() == renderer::GeometryType::GEOMETRY_TEXTURED_BONED_MESH) {
          renderTexturedBonedMesh((GeometryTexturedBonedMesh*) cur_geom, i_camera, color);
        }
        if (cur_geom->type() == renderer::GeometryType::GEOMETRY_COLORED_BONED_MESH) {
          renderColoredBonedMesh((GeometryColoredBonedMesh*) cur_geom, i_camera, color);
        }
      }
    }

    if (!tiled) {
      if (color) {
        cdepth_texture_->end();
      } else {
        depth_texture_->end();
      }
    }
  }

  void ModelRenderer::visualizeDepthMap(jtil::windowing::Window* wnd, 
    const uint32_t i_camera, const bool color) {
    // Get the depth data, and find the min and max values
    if (color) {
      cdepth_texture_->getTexture0Data<float>(depth_tmp_[i_camera]);
    } else {
      depth_texture_->getTexture0Data<float>(depth_tmp_[i_camera]);
    }
    float min_depth = std::numeric_limits<float>::infinity();
    float max_depth = -std::numeric_limits<float>::infinity();
    for (uint32_t i = 0; i < depth_w * depth_h; i++) {
      uint32_t index = color ? i * 4 : i;
      if (depth_tmp_[i_camera][index] > EPSILON) {
        if (depth_tmp_[i_camera][index] < min_depth) {
          min_depth = depth_tmp_[i_camera][index];
        }
        if (depth_tmp_[i_camera][index] > max_depth) {
          max_depth = depth_tmp_[i_camera][index];
        }
      }
    }

    if (min_depth == std::numeric_limits<float>::infinity()) {
      min_depth = 10.0f;
    }
    if (max_depth == -std::numeric_limits<float>::infinity()) {
      max_depth = 1000.0f;
    }
    
    // Render to the framebuffer
    GLState::glsBindFramebuffer(GL_FRAMEBUFFER, 0);
    // TO DO: Fix this --> Need width and height of the framebuffer!
    GLState::glsViewport(0, 0, wnd->width(), wnd->height());
    
    GLState::glsClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    GLState::glsClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    if (color) {
      sp_visualize_cdepth_->useProgram();
    } else {
      sp_visualize_depth_->useProgram();
    }
    
    if (color) {
      cdepth_texture()->bind(0, GL_TEXTURE0, h_visualize_cdepth_texture_sampler_);
      g_renderer_->bindFloat1(min_depth, h_f_depth_min_visualize_cdepth_);
      g_renderer_->bindFloat1(max_depth, h_f_depth_max_visualize_cdepth_);
    } else {
      depth_texture()->bind(0, GL_TEXTURE0, h_visualize_depth_texture_sampler_);
      g_renderer_->bindFloat1(min_depth, h_f_depth_min_visualize_depth_);
      g_renderer_->bindFloat1(max_depth, h_f_depth_max_visualize_depth_);
    }
    
    GLState::glsDisable(GL_DEPTH_TEST);
    GLState::glsDisable(GL_CULL_FACE);
    GLState::glsDisable(GL_BLEND);
    
    g_renderer_->quad()->draw();
  }

  void ModelRenderer::extractDepthMap(float* depth_vals) {
    depth_texture_->getTexture0Data<float>(depth_vals);
  }

  bool first_run = true;

  void ModelRenderer::uploadDepth(const uint32_t i_camera, 
    const int16_t* depth_vals) {
    // First clear the depth
    for (uint32_t i = 0; i < tex_size_ * tex_size_; i++) {
      depth_tmp_[i_camera][i] = 0.0f;
    }
    // Now copy the depth into the viewport (0,0,w,h)
    for (uint32_t v = 0; v < depth_h; v++) {
      for (uint32_t u = 0; u < depth_w; u++) {
        uint32_t i_src = v * depth_w + u;
        uint32_t i_dst = v * tex_size_ + u;
        depth_tmp_[i_camera][i_dst] = static_cast<float>(depth_vals[i_src]);
      }
    }

    // TODO: This might be slow...  We don't want to regenerate the texture
    // but just update the information.  In profile this doesn't seem too slow,
    // but the profile time might not reflect the true impact.
    // I EVEN HAVE: reloadData --> TRY IT!
    SAFE_DELETE(kinect_depth_textures_[i_camera]);
    kinect_depth_textures_[i_camera] = new Texture(GL_R32F, tex_size_,
      tex_size_, GL_RED, GL_FLOAT, 
      reinterpret_cast<unsigned char*>(depth_tmp_[i_camera]), 
      renderer::TEXTURE_WRAP_MODE::TEXTURE_CLAMP, false);

    // Again clear the depth
    for (uint32_t i = 0; i < tex_size_ * tex_size_ * NTILES; i++) {
      depth_tmp_[i_camera][i] = 0.0f;
    }

    // Now copy the depth into the viewports (i*tex_size_, i*tex_size_, w, h)
    for (uint32_t tile_y = 0; tile_y < NTILES_DIM; tile_y++) {
      uint32_t y_offst = tile_y * tex_size_;
      for (uint32_t tile_x = 0; tile_x < NTILES_DIM; tile_x++) {
        uint32_t x_offst = tile_x * tex_size_;
        for (uint32_t v = 0; v < depth_h; v++) {
          for (uint32_t u = 0; u < depth_w; u++) {
            uint32_t src_index = v * depth_w + u;
            uint32_t dst_index = (y_offst + v) * (tex_size_ * NTILES_DIM)
              + x_offst + u;
            depth_tmp_[i_camera][dst_index] = 
              static_cast<float>(depth_vals[src_index]);
          }
        }
      }
    }

    SAFE_DELETE(kinect_depth_textures_tiled_[i_camera]);
    kinect_depth_textures_tiled_[i_camera] = new Texture(GL_R32F, 
      tex_size_ * NTILES_DIM, tex_size_ * NTILES_DIM, 
      GL_RED, GL_FLOAT, reinterpret_cast<unsigned char*>(depth_tmp_[i_camera]), 
      renderer::TEXTURE_WRAP_MODE::TEXTURE_CLAMP, false);
  }

  float ModelRenderer::calculateResidualDataTerm(const uint32_t i_camera) {
    // Render to the dst texture
    residue_tex_[0]->begin();
    sp_residue_calc_->useProgram();

#if defined(DEBUG) || defined(_DEBUG)
    // Check that the texture sizes match
    if (depth_texture_->w() != residue_tex_[0]->w() || 
      depth_texture_->h() != residue_tex_[0]->h() || 
      kinect_depth_textures_[i_camera]->w() != residue_tex_[0]->w() ||
      kinect_depth_textures_[i_camera]->h() != residue_tex_[0]->h()) {
      throw std::wruntime_error("Incorrect Texture Sizes!");
    }
#endif

    depth_texture_->bind(0, GL_TEXTURE0, h_residue_calc_synth_depth_);
    kinect_depth_textures_[i_camera]->bind(GL_TEXTURE1, h_residue_calc_kinect_depth_);

    g_renderer_->bindFloat1(MAX_DEPTH_IN_RESIDUE, h_residue_calc_max_depth_);
    
    GLState::glsDisable(GL_DEPTH_TEST);
    GLState::glsEnable(GL_CULL_FACE);
    GLState::glsCullFace(GL_BACK);
    GLState::glsDisable(GL_BLEND);
    
    g_renderer_->quad()->draw();
    residue_tex_[0]->end();

    // Now downsample to a size of 1x1
    for (uint32_t i = 0; i < residue_tex_.size() - 1; i++) {
      if (residue_tex_[i]->w() / residue_tex_[i+1]->w() == 4) {
        g_renderer_->downsample4IntegTexture(residue_tex_[i+1], residue_tex_[i]);
      } else {
        g_renderer_->downsample2IntegTexture(residue_tex_[i+1], residue_tex_[i]);
      }
    }

    residue_tex_[residue_tex_.size()-1]->getTexture0Data<float>(depth_tmp_[i_camera]);
    float depth_integral = depth_tmp_[i_camera][0];  // Integral(abs(d_kin - d_syn))
    float o_s_union_r_s = UNION;  // union of kinect and depth pixels
    float o_s_intersect_r_s = 0.0f;  // intersection of kinect and depth pixels

    // Finally, calculate the objective function value
    static float lambda = DATA_TERM_LAMBDA;
    float data_term = lambda * (depth_integral / (o_s_union_r_s + EPSILON));  // Actually this is just =(k * depth_integral)

#ifdef EQUAL_CAMERA_IMPORTANCE
    return data_term;
#else
    return data_term / (i_camera + 1);
#endif
  }

  void ModelRenderer::calculateResidualDataTermTiled(Vector<float>& residues, 
    const uint32_t i_camera) {
    // Render to the dst texture
    tiled_residue_tex_[0]->begin();
    sp_residue_calc_->useProgram();

#if defined(DEBUG) || defined(_DEBUG)
    // Check that the texture sizes match
    if (depth_texture_tiled_->w() != tiled_residue_tex_[0]->w() || 
      depth_texture_tiled_->h() != tiled_residue_tex_[0]->h() || 
      kinect_depth_textures_tiled_[i_camera]->w() != tiled_residue_tex_[0]->w() ||
      kinect_depth_textures_tiled_[i_camera]->h() != tiled_residue_tex_[0]->h()) {
      throw std::wruntime_error("Incorrect Texture Sizes!");
    }
#endif

    // Residual shader uniforms:
    depth_texture_tiled_->bind(0, GL_TEXTURE0, h_residue_calc_synth_depth_);
    kinect_depth_textures_tiled_[i_camera]->bind(GL_TEXTURE1, h_residue_calc_kinect_depth_);
    g_renderer_->bindFloat1(MAX_DEPTH_IN_RESIDUE, h_residue_calc_max_depth_);
    //math::Float2 texel_dim(1.0f / depth_texture_tiled_->w(), 1.0f / depth_texture_tiled_->h());
    //g_renderer_->bindFloat2(&texel_dim, h_residue_calc_texel_dim_);
    
    GLState::glsDisable(GL_DEPTH_TEST);
    GLState::glsEnable(GL_CULL_FACE);
    GLState::glsCullFace(GL_BACK);
    GLState::glsDisable(GL_BLEND);
    
    g_renderer_->quad()->draw();

    tiled_residue_tex_[0]->end();

    // Now downsample to a size of NTILES_DIM x NTILES_DIM
    for (uint32_t i = 0; i < tiled_residue_tex_.size() - 1; i++) {
      if (tiled_residue_tex_[i]->w() / tiled_residue_tex_[i+1]->w() == 4) {
        g_renderer_->downsample4IntegTexture(tiled_residue_tex_[i+1], tiled_residue_tex_[i]);
      } else {
        g_renderer_->downsample2IntegTexture(tiled_residue_tex_[i+1], tiled_residue_tex_[i]);
      }
    }

    TextureRenderable* final_tex = tiled_residue_tex_[tiled_residue_tex_.size()-1];

    final_tex->getTexture0Data<float>(depth_tmp_[i_camera]);

    static float lambda = DATA_TERM_LAMBDA;

    for (uint32_t tile_v = 0; tile_v < NTILES_DIM; tile_v++) {
      for (uint32_t tile_u = 0; tile_u < NTILES_DIM; tile_u++) {
        uint32_t tile_i = tile_v*NTILES_DIM + tile_u;
        if (tile_i < residues.size()) {
          // Integral(abs(d_kin - d_syn))
          float depth_integral = depth_tmp_[i_camera][tile_i];
          float o_s_union_r_s = UNION;  // union of kinect and depth pixels
          float o_s_intersect_r_s = 0.0f;  // intersection of kinect and depth pixels

          float data_term = lambda * (depth_integral / (o_s_union_r_s + EPSILON));

#ifdef EQUAL_CAMERA_IMPORTANCE
          residues[tile_v*NTILES_DIM + tile_u] += data_term;
#else
          residues[tile_v*NTILES_DIM + tile_u] += data_term / (i_camera + 1);
#endif
        }
      }
    }
  }

  void ModelRenderer::renderTexturedBonedMesh(GeometryTexturedBonedMesh* geom,
    const uint32_t i_camera, bool color) {
    // Calculate the model view matrix and bind it to the shader
    Float4x4::mult(VW_mat_, *(cameras_[i_camera]->view()), *geom->mat_hierarchy());
    if (color) {
      g_renderer_->bindFloat4x4(&VW_mat_, h_VW_mat_sp_cdepth_skinned_);
    } else {
      g_renderer_->bindFloat4x4(&VW_mat_, h_VW_mat_sp_depth_skinned_);
    }

    // Calculate model view projection matrix and bind it to the shader
    Float4x4::mult(PVW_mat_, *(cameras_[i_camera]->proj()), VW_mat_ );
    if (color) {
      g_renderer_->bindFloat4x4(&PVW_mat_, h_PVW_mat_sp_cdepth_skinned_);
    } else {
      g_renderer_->bindFloat4x4(&PVW_mat_, h_PVW_mat_sp_depth_skinned_);
    }

    // Send the bone matrix heirachy down to the shader:
    BoneFileInfo* bones_in_file = geom->bones();
    // First copy the bone data into a flat float array:
    if (matrix_data_.capacity() < bones_in_file->bones.size() * 16) {
      matrix_data_.capacity(bones_in_file->bones.size() * 16);
      matrix_data_.resize(bones_in_file->bones.size() * 16);
    }

    for (uint32_t i = 0; i < bones_in_file->bones.size(); i++) {
#ifndef LINEAR_BLEND_SKINNING
      memcpy(matrix_data_.at(i*8), 
        bones_in_file->bones[i]->uniform_dual_quaternion, 8 * 
        sizeof(matrix_data_[0]));
#else
      memcpy(matrix_data_.at(i*16), bones_in_file->bones[i]->final_trans.m,
        16 * sizeof(matrix_data_[0]));
#endif
    }
    // Now BIND the entire array at once
#ifndef LINEAR_BLEND_SKINNING
    if (color) {
      glUniformMatrix2x4fv(h_bone_trans_sp_depth_skinned_[0], 
        bones_in_file->bones.size(), GL_FALSE, matrix_data_.at(0));
    } else {
      glUniformMatrix2x4fv(h_bone_trans_sp_cdepth_skinned_[0], 
        bones_in_file->bones.size(), GL_FALSE, matrix_data_.at(0));
    }
#else
    if (color) {
      glUniformMatrix4fv(h_bone_trans_sp_depth_skinned_[0], 
        bones_in_file->bones.size(), GL_FALSE, matrix_data_.at(0));
    } else {
      glUniformMatrix4fv(h_bone_trans_sp_cdepth_skinned_[0], 
        bones_in_file->bones.size(), GL_FALSE, matrix_data_.at(0));
    }
#endif
    ERROR_CHECK;

    // Draw the current geometry
    geom->draw();
  }

  void ModelRenderer::renderColoredBonedMesh(GeometryColoredBonedMesh* geom, 
    const uint32_t i_camera, bool color) {
    // Calculate the model view matrix and bind it to the shader
    Float4x4::mult(VW_mat_, *(cameras_[i_camera]->view()), *geom->mat_hierarchy());
    if (color) {
      g_renderer_->bindFloat4x4(&VW_mat_, h_VW_mat_sp_cdepth_skinned_);
    } else {
      g_renderer_->bindFloat4x4(&VW_mat_, h_VW_mat_sp_depth_skinned_);
    }

    // Calculate model view projection matrix and bind it to the shader
    Float4x4::mult(PVW_mat_, *(cameras_[i_camera]->proj()), VW_mat_ );
    if (color) {
      g_renderer_->bindFloat4x4(&PVW_mat_, h_PVW_mat_sp_cdepth_skinned_);
    } else {
      g_renderer_->bindFloat4x4(&PVW_mat_, h_PVW_mat_sp_depth_skinned_);
    }

    // Send the bone matrix heirachy down to the shader:
    BoneFileInfo* bones_in_file = geom->bones();
    for (uint32_t i = 0; i < bones_in_file->bones.size(); i++) {
#ifndef LINEAR_BLEND_SKINNING
      if (color) {
        g_renderer_->bindFloat2x4(bones_in_file->bones[i]->uniform_dual_quaternion, 
          h_bone_trans_sp_cdepth_skinned_[i]);
      } else {
        g_renderer_->bindFloat2x4(bones_in_file->bones[i]->uniform_dual_quaternion, 
          h_bone_trans_sp_depth_skinned_[i]);
      }
#else
      if (color) {
        g_renderer_->bindFloat4x4(&bones_in_file->bones[i]->final_trans, 
          h_bone_trans_sp_depth_skinned_[i]);
      } else {
        g_renderer_->bindFloat4x4(&bones_in_file->bones[i]->final_trans, 
          h_bone_trans_sp_cdepth_skinned_[i]);
      }
#endif
    }

    // Draw the current geometry
    geom->draw();
  }

  void ModelRenderer::renderColoredMesh(GeometryColoredMesh* geom, 
    const uint32_t i_camera, bool color) {
    // Calculate the model view matrix and bind it to the shader
    Float4x4::mult(VW_mat_, *(cameras_[i_camera]->view()), *(geom->mat_hierarchy()));
    if (color) {
      g_renderer_->bindFloat4x4(&VW_mat_, h_VW_mat_sp_cdepth_);
    } else {
      g_renderer_->bindFloat4x4(&VW_mat_, h_VW_mat_sp_depth_);
    }

    // Calculate model view projection matrix and bind it to the shader
    Float4x4::mult(PVW_mat_, *(cameras_[i_camera]->proj()), VW_mat_ );
    if (color) {
      g_renderer_->bindFloat4x4(&PVW_mat_, h_PVW_mat_sp_cdepth_);
    } else {
      g_renderer_->bindFloat4x4(&PVW_mat_, h_PVW_mat_sp_depth_);
    }
    

    // Draw the current geometry
    geom->draw();
  }

  float ModelRenderer::calcInterpenetrationTerm(const uint32_t max_groups) {
    jtil::data_str::Vector<BoundingSphere*>& bsph = PoseModel::g_b_spheres;
    const uint32_t num_spheres = bsph.size();
    // Assume bounding sphere matrices have been updated (they should be)!
    // For each bounding sphere transform the center and the radius:
//#pragma omp parallel for num_threads(4)
    for (int32_t i = 0; i < (int32_t)num_spheres; i++) {
      bsph[i]->transform();
    }

    float total_penetration_distance = 0;

    // Just do O(n^2) tests --> I did have a sweap and prune implementation but
    // it was actually slower 
//#pragma omp parallel for num_threads(4)
    for (int32_t i = 0; i < (int32_t)num_spheres; i++) {
      Float3 vec;
      for (int32_t j = i+1; j < (int32_t)num_spheres; j++) {
        int32_t objA_group = i / NSPH_PER_GROUP;  // Which finger
        int32_t objB_group = j / NSPH_PER_GROUP;
        int32_t objA_sphere = i % NSPH_PER_GROUP;  // which sphere on the finger
        int32_t objB_sphere = j % NSPH_PER_GROUP;
        // ignore penetration within the same finger
        if (objA_group != objB_group && objA_group < (int32_t)max_groups && 
          objB_group < (int32_t)max_groups) {
          BoundingSphere* objA = bsph[i];
          BoundingSphere* objB = bsph[j];
          Float3::sub(vec, *objA->transformed_center(), *objB->transformed_center());
          float center_dist_sq = Float3::dot(vec, vec);
          float min_dist = objA->transformed_radius() + objB->transformed_radius();
          float min_dist_sq = min_dist * min_dist;
          float sep_dist_sq = center_dist_sq - min_dist_sq;
          if (sep_dist_sq < 0) {
            //#pragma omp atomic
            total_penetration_distance += -sep_dist_sq;
          }
        }
      }
    }

    return 1.0f + INTERPENETRATION_CONSTANT * total_penetration_distance;
  }

}  // namespace model_fit
