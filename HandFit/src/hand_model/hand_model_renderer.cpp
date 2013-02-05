//
//  hand_model.h
//
//  Created by Jonathan Tompson on 8/17/12.
//

#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include "hand_model/hand_model_renderer.h"
#include "hand_model/hand_model_geometry.h"
#include "hand_model/hand_model_geometry_simple.h"
#include "hand_model/hand_model_geometry_mesh.h"
#include "hand_model/hand_model_fit.h"
#include "hand_model/bounding_sphere.h"
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
#include "renderer/texture/texture.h"
#include "renderer/texture/texture_renderable.h"
#include "renderer/shader/shader.h"
#include "renderer/shader/shader_program.h"
#include "renderer/camera/camera.h"
#include "windowing/window.h"
#include "math/lm_fitting.h"
#include "math/nm_fitting.h"
#include "math/de_fitting.h"
#include "math/pso_fitting.h"
#include "math/lprpso_fitting.h"
#include "file_io/file_io.h"
#include "data_str/vector.h"
#include "Eigen"
#include "renderer/gl_state.h"

#define SAFE_DELETE(target) \
  if (target != NULL) { \
    delete target; \
    target = NULL; \
  }

using data_str::Vector;
using data_str::VectorManaged;
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
using renderer::GeometryColoredBonedMesh;
using renderer::GeometryTexturedBonedMesh;
using renderer::Renderer;
using renderer::TextureRenderable;
using renderer::Texture;
using renderer::Shader;
using renderer::ShaderProgram;
using renderer::ShaderType;
using renderer::BoneFileInfo;
using renderer::BoundingSphere;
using renderer::GLState;

namespace hand_model {
  
  HandModelRenderer::HandModelRenderer(renderer::Renderer* g_renderer, 
    bool left, bool right) {
    render_hand_ = true;
    depth_tmp_ = new float[DEPTH_IMAGE_DIM * NTILES];
    math::FloatQuat eye_rot; eye_rot.identity();
    math::Float3 eye_pos(0, 0, 0);
    camera_ = new Camera(&eye_rot, &eye_pos, DEPTH_IMAGE_WIDTH, 
      DEPTH_IMAGE_HEIGHT, HAND_CAMERA_FOV, -HAND_CAMERA_VIEW_PLANE_NEAR, 
      -HAND_CAMERA_VIEW_PLANE_FAR);
    camera_->updateView();
    camera_->updateProjection();
    
    g_renderer_ = g_renderer;  // Not owned here.
    if (left) {
#ifdef USE_SIMPLE_GEOMETRY
      l_hand_geom_ = new HandModelGeometrySimple(HandType::LEFT, g_renderer, this);
#else
      l_hand_geom_ = new HandModelGeometryMesh(HandType::LEFT, g_renderer, this);
#endif
    } else {
      l_hand_geom_ = NULL;
    }
    if (right) {
#ifdef USE_SIMPLE_GEOMETRY
      r_hand_geom_ = new HandModelGeometrySimple(HandType::RIGHT, g_renderer, this);
#else
      r_hand_geom_ = new HandModelGeometryMesh(HandType::RIGHT, g_renderer, this);
#endif
    } else {
      r_hand_geom_ = NULL;
    }
    
    // Renderable texture
    depth_texture_ = new TextureRenderable(GL_R32F, DEPTH_IMAGE_WIDTH,
      DEPTH_IMAGE_HEIGHT, GL_RED, GL_FLOAT, 1, true);
    cdepth_texture_ = new TextureRenderable(GL_RGBA32F, DEPTH_IMAGE_WIDTH,
      DEPTH_IMAGE_HEIGHT, GL_RGBA, GL_FLOAT, 1, true);
    depth_texture_tiled_ = new TextureRenderable(GL_R32F, 
      DEPTH_IMAGE_WIDTH * NTILES_X, DEPTH_IMAGE_HEIGHT * NTILES_Y, GL_RED, 
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
    sp_depth_skinned_->bindVertShaderInputLocation(Renderer::bone_ids_47);
    sp_depth_skinned_->bindVertShaderInputLocation(Renderer::bone_weights_47);
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
    sp_cdepth_skinned_->bindVertShaderInputLocation(Renderer::bone_ids_47);
    sp_cdepth_skinned_->bindVertShaderInputLocation(Renderer::bone_weights_47);
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
    f_shader_residue_calc_ = new Shader("shaders/residue_calc.frag", ShaderType::FRAGMENT_SHADER);
    sp_residue_calc_ = new ShaderProgram(v_shader_residue_calc_, f_shader_residue_calc_);
    sp_residue_calc_->bindVertShaderInputLocation(Renderer::pos);
    sp_residue_calc_->link();
    h_residue_calc_kinect_depth_ = sp_residue_calc_->getUniformLocation("kinect_depth");
    h_residue_calc_synth_depth_ = sp_residue_calc_->getUniformLocation("synth_depth");
    h_residue_calc_max_depth_ = sp_residue_calc_->getUniformLocation("max_depth");
    //h_residue_calc_texel_dim_ = sp_residue_calc_->getUniformLocation("texel_dim");

    // Textures to downsample and integrate
    residue_texture_1_ = new TextureRenderable(GL_RGB32F, 
      DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT, GL_RGB, GL_FLOAT, 1, false);
    residue_texture_2_ = new TextureRenderable(GL_RGB32F, 
      DEPTH_IMAGE_WIDTH/2, DEPTH_IMAGE_HEIGHT/2, GL_RGB, GL_FLOAT, 1, false);
    residue_texture_4_ = new TextureRenderable(GL_RGB32F, 
      DEPTH_IMAGE_WIDTH/4, DEPTH_IMAGE_HEIGHT/4, GL_RGB, GL_FLOAT, 1, false);
    residue_texture_16_ = new TextureRenderable(GL_RGB32F, 
      DEPTH_IMAGE_WIDTH/16, DEPTH_IMAGE_HEIGHT/16, GL_RGB, GL_FLOAT, 1, false);
    residue_texture_20_ = new TextureRenderable(GL_RGB32F, 
      DEPTH_IMAGE_WIDTH/20, DEPTH_IMAGE_HEIGHT/20, GL_RGB, GL_FLOAT, 1, false);
    residue_texture_32_ = new TextureRenderable(GL_RGB32F, 
      DEPTH_IMAGE_WIDTH/32, DEPTH_IMAGE_HEIGHT/32, GL_RGB, GL_FLOAT, 1, false);
    residue_texture_160_ = new TextureRenderable(GL_RGB32F, 
      DEPTH_IMAGE_WIDTH/160, DEPTH_IMAGE_HEIGHT/160, GL_RGB, GL_FLOAT, 1, false);

    residue_texture_x2_ = new TextureRenderable(GL_RGB32F, 
      DEPTH_IMAGE_WIDTH*2, DEPTH_IMAGE_HEIGHT*2, GL_RGB, GL_FLOAT, 1, false);
    residue_texture_x8_ = new TextureRenderable(GL_RGB32F, 
      DEPTH_IMAGE_WIDTH*8, DEPTH_IMAGE_HEIGHT*8, GL_RGB, GL_FLOAT, 1, false);

    kinect_depth_texture_ = NULL;
    kinect_depth_texture_tiled_ = NULL;

    active_list_.capacity(NUM_BOUNDING_SPHERES*2);

    for (uint32_t i = 0; i < NUM_BOUNDING_SPHERES*2; i++) {
      // Add it to the collision detection sort and sweep arrays
      bsph_sx[i] = i;
      bsph_sy[i] = i;
      bsph_sz[i] = i;
      bsph_fx[i] = i;
      bsph_fy[i] = i;
      bsph_fz[i] = i;
    }
  }

  HandModelRenderer::~HandModelRenderer() {
    SAFE_DELETE(depth_tmp_);
    SAFE_DELETE(camera_);
    SAFE_DELETE(l_hand_geom_);
    SAFE_DELETE(r_hand_geom_);
    SAFE_DELETE(depth_texture_);
    SAFE_DELETE(depth_texture_tiled_);
    SAFE_DELETE(residue_texture_1_);
    SAFE_DELETE(residue_texture_2_);
    SAFE_DELETE(residue_texture_4_);
    SAFE_DELETE(residue_texture_16_);
    SAFE_DELETE(residue_texture_20_);
    SAFE_DELETE(residue_texture_32_);
    SAFE_DELETE(residue_texture_160_);
    SAFE_DELETE(residue_texture_x2_);
    SAFE_DELETE(residue_texture_x8_);
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
    SAFE_DELETE(kinect_depth_texture_);
    SAFE_DELETE(kinect_depth_texture_tiled_);
    SAFE_DELETE(f_shader_residue_calc_);
    SAFE_DELETE(v_shader_residue_calc_);
  }

  void HandModelRenderer::updateMatrices(const MatrixXf& coeff, 
    HandType hand_type) {
    if (hand_type == HandType::LEFT) {
      l_hand_geom_->updateMatrices(coeff);
    } else {
      r_hand_geom_->updateMatrices(coeff);
    }
  }

  void HandModelRenderer::drawDepthMapTiled(Vector<float>& residues,
    VectorManaged<MatrixXf>& coeff, HandModel** hands, 
    uint32_t num_hands_per_tile) {
    depth_texture_tiled_->begin();

    GLState::glsClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    uint32_t nimages = coeff.size();
    if (nimages > NTILES) {
      throw std::runtime_error("Too many images to draw tiled!");
    }
    for (uint32_t i = 0; i < nimages; i++) {
      // Set up the viewport
      uint32_t xpos = i % NTILES_X;
      uint32_t ypos = i / NTILES_X;
      GLState::glsViewport(xpos * DEPTH_IMAGE_WIDTH, ypos * DEPTH_IMAGE_HEIGHT, 
        DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT);
      drawDepthMapInternal(coeff[i], hands, num_hands_per_tile, false, true);
      // Before destorying the matrix heirachy, calculate the interpenetration
      residues[i] += calcInterpenetrationTerm();
    }

    depth_texture_tiled_->end();
  }

  void HandModelRenderer::drawDepthMapInternal(const Eigen::MatrixXf& coeff,
    HandModel** hands, uint32_t num_hands, bool color, bool tiled) {
    HandModelGeometry* hand[2];
    hand[0] = (hands[0]->hand_type() == HandType::LEFT) ? 
    l_hand_geom_ : r_hand_geom_;
    if (num_hands > 1) {
      hand[1] = (hands[1]->hand_type() == HandType::LEFT) ? 
        l_hand_geom_ : r_hand_geom_;
    } else {
      hand[1] = NULL;
    }
    if (hand[0] == NULL || (num_hands>1 && hand[1] == NULL)) {
    throw std::runtime_error(string("drawDepthMap - ERROR: hand_geom") +
      string(" not initialized"));
    }

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
        cdepth_texture_->begin();
      } else {
        depth_texture_->begin();
      }
      GLState::glsClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    }

    for (uint32_t i = 0; i < 2 && hand[i] != NULL; i++) {
      // Updates the individual nodes
      if (coeff.rows() > 1) {  // row matrix
        hand[i]->updateMatrices(coeff.block<HAND_NUM_COEFF, 1>(i*HAND_NUM_COEFF, 0));  
      } else {  // column matrix
        hand[i]->updateMatrices(coeff.block<1, HAND_NUM_COEFF>(0, i*HAND_NUM_COEFF));  
      }
      hand[i]->updateHeirachyMatrices();
      hand[i]->fixBoundingSphereMatrices();

      // Render all the colored mesh elements
      if (color) {
        sp_cdepth_->useProgram();
      } else {
        sp_depth_->useProgram();
      }
      hand[i]->renderStackReset();
      while (!hand[i]->renderStackEmpty()) {
        Geometry* cur_geom = hand[i]->renderStackPop();
        if (cur_geom->type() == renderer::GeometryType::GEOMETRY_COLORED_MESH) {
          renderColoredMesh((GeometryColoredMesh*) cur_geom, color);
        }
      }

      // Render all the textured and colored boned mesh elements
      if (color) {
        sp_cdepth_skinned_->useProgram();
      } else {
        sp_depth_skinned_->useProgram();
      }
      hand[i]->renderStackReset();
      while (!hand[i]->renderStackEmpty()) {
        Geometry* cur_geom = hand[i]->renderStackPop();
        if (cur_geom->type() == renderer::GeometryType::GEOMETRY_TEXTURED_BONED_MESH) {
          renderTexturedBonedMesh((GeometryTexturedBonedMesh*) cur_geom, color);
        }
        if (cur_geom->type() == renderer::GeometryType::GEOMETRY_COLORED_BONED_MESH) {
          renderColoredBonedMesh((GeometryColoredBonedMesh*) cur_geom, color);
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

  void HandModelRenderer::drawDepthMap(const Eigen::MatrixXf& coeff,
    HandModel** hands, uint32_t num_hands, bool color) {
    drawDepthMapInternal(coeff, hands, num_hands, color, false);
  }

  void HandModelRenderer::visualizeDepthMap(windowing::Window* wnd, 
    bool color) {
    // Get the depth data, and find the min and max values
    if (color) {
      cdepth_texture_->getTexture0Data<float>(depth_tmp_);
    } else {
      depth_texture_->getTexture0Data<float>(depth_tmp_);
    }
    float min_depth = std::numeric_limits<float>::infinity();
    float max_depth = -std::numeric_limits<float>::infinity();
    for (uint32_t i = 0; i < DEPTH_IMAGE_WIDTH * DEPTH_IMAGE_HEIGHT; i++) {
      uint32_t index = color ? i * 4 : i;
      if (depth_tmp_[index] > EPSILON) {
        if (depth_tmp_[index] < min_depth) {
          min_depth = depth_tmp_[index];
        }
        if (depth_tmp_[index] > max_depth) {
          max_depth = depth_tmp_[index];
        }
      }
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

  void HandModelRenderer::extractDepthMap(float* depth_vals) {
    depth_texture_->getTexture0Data<float>(depth_vals);
  }

  bool first_run = true;

  void HandModelRenderer::uploadKinectDepth(int16_t* depth_vals) {
    for (uint32_t i = 0; i < DEPTH_IMAGE_DIM; i++) {
      depth_tmp_[i] = static_cast<float>(depth_vals[i]);
    }

    SAFE_DELETE(kinect_depth_texture_);
    kinect_depth_texture_ = new Texture(GL_R32F, DEPTH_IMAGE_WIDTH,
      DEPTH_IMAGE_HEIGHT, GL_RED, GL_FLOAT, 
      reinterpret_cast<unsigned char*>(depth_tmp_), 
      renderer::TEXTURE_WRAP_MODE::TEXTURE_CLAMP, false);

    for (uint32_t tile_y = 0; tile_y < NTILES_Y; tile_y++) {
      uint32_t y_offst = tile_y * DEPTH_IMAGE_HEIGHT;
      for (uint32_t tile_x = 0; tile_x < NTILES_X; tile_x++) {
        uint32_t x_offst = tile_x * DEPTH_IMAGE_WIDTH;
        for (uint32_t v = 0; v < DEPTH_IMAGE_HEIGHT; v++) {
          for (uint32_t u = 0; u < DEPTH_IMAGE_WIDTH; u++) {
            uint32_t src_index = v * DEPTH_IMAGE_WIDTH + u;
            uint32_t dst_index = (y_offst + v) * (DEPTH_IMAGE_WIDTH * NTILES_X)
              + x_offst + u;
            depth_tmp_[dst_index] = static_cast<float>(depth_vals[src_index]);
          }
        }
      }
    }

    SAFE_DELETE(kinect_depth_texture_tiled_);
    kinect_depth_texture_tiled_ = new Texture(GL_R32F, 
      DEPTH_IMAGE_WIDTH * NTILES_X, DEPTH_IMAGE_HEIGHT * NTILES_Y, 
      GL_RED, GL_FLOAT, reinterpret_cast<unsigned char*>(depth_tmp_), 
      renderer::TEXTURE_WRAP_MODE::TEXTURE_CLAMP, false);
  }

  float HandModelRenderer::calculateResidualDataTerm() {
    // Render to the dst texture
    residue_texture_1_->begin();
    sp_residue_calc_->useProgram();

    
    depth_texture_->bind(0, GL_TEXTURE0, h_residue_calc_synth_depth_);
    kinect_depth_texture_->bind(GL_TEXTURE1, h_residue_calc_kinect_depth_);
    g_renderer_->bindFloat1(MAX_DEPTH_IN_RESIDUE, h_residue_calc_max_depth_);
    //math::Float2 texel_dim(1.0f / depth_texture_->w(), 1.0f / depth_texture_->h());
    //g_renderer_->bindFloat2(&texel_dim, h_residue_calc_texel_dim_);
    
    GLState::glsDisable(GL_DEPTH_TEST);
    GLState::glsEnable(GL_CULL_FACE);
    GLState::glsCullFace(GL_BACK);
    GLState::glsDisable(GL_BLEND);
    
    g_renderer_->quad()->draw();

    residue_texture_1_->end();

    // Now downsample and integrate to 1/160 the size in each dimension
    g_renderer_->downsample4IntegTexture(residue_texture_4_, residue_texture_1_);
    g_renderer_->downsample4IntegTexture(residue_texture_16_, residue_texture_4_);
    g_renderer_->downsample2IntegTexture(residue_texture_32_, residue_texture_16_);
    g_renderer_->downsample5IntegTexture(residue_texture_160_, residue_texture_32_);

    // Now we need to finish the rest of the integration on the CPU since the 
    // resolution is now 4 x 3
    glFlush();
    residue_texture_160_->getTexture0Data<float>(depth_tmp_);

    float depth_integral = 0.0f;  // Integral(abs(d_kin - d_syn))
    float o_s_union_r_s = 0.0f;  // union of kinect and depth pixels
    float o_s_intersect_r_s = 0.0f;  // union of kinect and depth pixels
    const int decimation = 4 * 4 * 2 * 5;
    for (uint32_t i = 0; i < DEPTH_IMAGE_DIM / (decimation*decimation); i++) {
      depth_integral += depth_tmp_[i*3];
      o_s_union_r_s += depth_tmp_[i*3+1];
      o_s_intersect_r_s += depth_tmp_[i*3+2];
    }

    // Finally, calculate the objective function value
    static float lambda = DATA_TERM_LAMBDA;
#ifdef DEPTH_ONLY_RESIDUE_FUNC
    float data_term = lambda * (depth_integral / (o_s_union_r_s + EPSILON));
#else
    float data_term = lambda * (depth_integral / (o_s_union_r_s + EPSILON)) +
      (1.0f - (2.0f*o_s_intersect_r_s / (o_s_intersect_r_s + o_s_union_r_s)));
#endif

#ifdef SAVE_DEBUG_TEXTURES
    if (first_run) {
      std::cout << "Saving synthetic depth and residue textures to file.";
      std::cout << std::endl;
      depth_texture_->saveTexture0Data("depth_texture.bin");
      depth_texture_->begin();
      g_renderer_->renderFullscreenQuad(kinect_depth_texture_);
      depth_texture_->saveTexture0Data("kinect_depth_texture.bin");
      residue_texture_1_->saveTexture0Data("residue_texture_1.bin");
      residue_texture_4_->saveTexture0Data("residue_texture_4.bin");
      residue_texture_16_->saveTexture0Data("residue_texture_16.bin");
      residue_texture_32_->saveTexture0Data("residue_texture_32.bin");
      residue_texture_160_->saveTexture0Data("residue_texture_160.bin");
      cout << "first run residue is: " << data_term << endl;
      first_run = false;
#if defined(WIN32) || defined(_WIN32)
      system("pause");
#endif
      exit(0);
    }
#endif  // SAVE_DEBUG_TEXTURES

    return data_term;
  }

  void HandModelRenderer::preBindUniforms() {
    
  }

  void HandModelRenderer::calculateResidualDataTermTiled(Vector<float>& residues) {
    // Render to the dst texture
    residue_texture_x8_->begin();
    sp_residue_calc_->useProgram();

    // Residual shader uniforms:
    depth_texture_tiled_->bind(0, GL_TEXTURE0, h_residue_calc_synth_depth_);
    kinect_depth_texture_tiled_->bind(GL_TEXTURE1, h_residue_calc_kinect_depth_);
    g_renderer_->bindFloat1(MAX_DEPTH_IN_RESIDUE, h_residue_calc_max_depth_);
    //math::Float2 texel_dim(1.0f / depth_texture_tiled_->w(), 1.0f / depth_texture_tiled_->h());
    //g_renderer_->bindFloat2(&texel_dim, h_residue_calc_texel_dim_);
    
    GLState::glsDisable(GL_DEPTH_TEST);
    GLState::glsEnable(GL_CULL_FACE);
    GLState::glsCullFace(GL_BACK);
    GLState::glsDisable(GL_BLEND);
    
    g_renderer_->quad()->draw();

    residue_texture_x8_->end();

    // 5120 x 3840 -> 1280 x 960 (tiles of 160 x 120)
    g_renderer_->downsample4IntegTexture(residue_texture_x2_, residue_texture_x8_); 

    // 1280 x 960 -> 320 x 240 (tiles of 40 x 30)
    g_renderer_->downsample4IntegTexture(residue_texture_2_, residue_texture_x2_);  
    // 320 x 240 -> 64 x 48 (tiles of 8 x 6)
    g_renderer_->downsample2IntegTexture(residue_texture_4_, residue_texture_2_);  
    // 64 x 48 -> 32 x 24 (tiles of 4 x 3)
    g_renderer_->downsample5IntegTexture(residue_texture_20_, residue_texture_4_); 

    // Now we need to finish the rest of the integration on the CPU since the 
    // tile resolution is now 4 x 3
    glFlush();
    residue_texture_20_->getTexture0Data<float>(depth_tmp_);

    const int decimation = 4 * 4 * 5 * 2;
    static float lambda = DATA_TERM_LAMBDA;

    for (uint32_t tile_v = 0; tile_v < NTILES_Y; tile_v++) {
      uint32_t v_off = tile_v * (DEPTH_IMAGE_HEIGHT / decimation);
      for (uint32_t tile_u = 0; tile_u < NTILES_X; tile_u++) {
        if (tile_v*NTILES_X + tile_u < residues.size()) {
          uint32_t u_off = tile_u * (DEPTH_IMAGE_WIDTH / decimation);

          float depth_integral = 0.0f;  // Integral(abs(d_kin - d_syn))
          float o_s_union_r_s = 0.0f;  // union of kinect and depth pixels
          float o_s_intersect_r_s = 0.0f;  // union of kinect and depth pixels

          for (uint32_t v = 0; v < (DEPTH_IMAGE_HEIGHT / decimation); v++) {
            for (uint32_t u = 0; u < (DEPTH_IMAGE_WIDTH / decimation); u++) {
              uint32_t i = (v_off + v) * ((NTILES_X * DEPTH_IMAGE_WIDTH) / decimation) + u_off + u;
              depth_integral += depth_tmp_[i*3];
              o_s_union_r_s += depth_tmp_[i*3+1];
              o_s_intersect_r_s += depth_tmp_[i*3+2];
            }
          }
#ifdef DEPTH_ONLY_RESIDUE_FUNC
          float data_term = lambda * (depth_integral / (o_s_union_r_s + EPSILON));
#else
          float data_term = lambda * (depth_integral / (o_s_union_r_s + EPSILON)) +
            (1.0f - (2.0f*o_s_intersect_r_s / (o_s_intersect_r_s + o_s_union_r_s)));
#endif
          residues[tile_v*NTILES_X + tile_u] += data_term;
        }
      }
    }

#ifdef SAVE_DEBUG_TEXTURES
    if (first_run) {
      std::cout << "Saving synthetic depth and residue textures to file.";
      std::cout << std::endl;
      depth_texture_tiled_->saveTexture0Data("depth_texture_tiled.bin");
      depth_texture_tiled_->begin();
      g_renderer_->renderFullscreenQuad(kinect_depth_texture_tiled_);
      depth_texture_tiled_->saveTexture0Data("kinect_depth_texture_tiled.bin");
      residue_texture_x8_->saveTexture0Data("residue_texture_x8_tiled.bin");
      residue_texture_x2_->saveTexture0Data("residue_texture_x2_tiled.bin");
      residue_texture_2_->saveTexture0Data("residue_texture_2_tiled.bin");
      residue_texture_4_->saveTexture0Data("residue_texture_4_tiled.bin");
      residue_texture_20_->saveTexture0Data("residue_texture_20_tiled.bin");
      cout << "first run residues are: ";
      for (uint32_t v = 0; v < NTILES_Y; v++) { 
        for (uint32_t u = 0; u < NTILES_X; u++) { 
          cout << residues[v * NTILES_X + u] << " "; 
        }
        cout << endl;
      }
      first_run = false;
#if defined(WIN32) || defined(_WIN32)
      system("pause");
#endif
      exit(0);
    }
#endif  // SAVE_DEBUG_TEXTURES
  }

  void HandModelRenderer::renderTexturedBonedMesh(GeometryTexturedBonedMesh* geom,
    bool color) {
    // Calculate the model view matrix and bind it to the shader
    Float4x4::mult(&VW_mat_, camera_->view(), geom->mat_hierarchy());
    if (color) {
      g_renderer_->bindFloat4x4(&VW_mat_, h_VW_mat_sp_cdepth_skinned_);
    } else {
      g_renderer_->bindFloat4x4(&VW_mat_, h_VW_mat_sp_depth_skinned_);
    }

    // Calculate model view projection matrix and bind it to the shader
    Float4x4::mult(&PVW_mat_, camera_->proj(), &VW_mat_ );
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
//#ifndef LINEAR_BLEND_SKINNING
//      if (color) {
//        g_renderer_->bindFloat2x4(bones_in_file->bones[i]->uniform_dual_quaternion, 
//          h_bone_trans_sp_cdepth_skinned_[i]);
//      } else {
//        g_renderer_->bindFloat2x4(bones_in_file->bones[i]->uniform_dual_quaternion, 
//          h_bone_trans_sp_depth_skinned_[i]);
//      }
//#else
//      if (color) {
//        g_renderer_->bindFloat4x4(&bones_in_file->bones[i]->final_trans, 
//          h_bone_trans_sp_depth_skinned_[i]);
//      } else {
//        g_renderer_->bindFloat4x4(&bones_in_file->bones[i]->final_trans, 
//          h_bone_trans_sp_cdepth_skinned_[i]);
//      }
//#endif
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

  void HandModelRenderer::renderColoredBonedMesh(GeometryColoredBonedMesh* geom, 
    bool color) {
    // Calculate the model view matrix and bind it to the shader
    Float4x4::mult(&VW_mat_, camera_->view(), geom->mat_hierarchy());
    if (color) {
      g_renderer_->bindFloat4x4(&VW_mat_, h_VW_mat_sp_cdepth_skinned_);
    } else {
      g_renderer_->bindFloat4x4(&VW_mat_, h_VW_mat_sp_depth_skinned_);
    }

    // Calculate model view projection matrix and bind it to the shader
    Float4x4::mult(&PVW_mat_, camera_->proj(), &VW_mat_ );
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

  void HandModelRenderer::renderColoredMesh(GeometryColoredMesh* geom, 
    bool color) {
    // Calculate the model view matrix and bind it to the shader
    Float4x4::mult(&VW_mat_, camera_->view(), geom->mat_hierarchy());
    if (color) {
      g_renderer_->bindFloat4x4(&VW_mat_, h_VW_mat_sp_cdepth_);
    } else {
      g_renderer_->bindFloat4x4(&VW_mat_, h_VW_mat_sp_depth_);
    }

    // Calculate model view projection matrix and bind it to the shader
    Float4x4::mult(&PVW_mat_, camera_->proj(), &VW_mat_ );
    if (color) {
      g_renderer_->bindFloat4x4(&PVW_mat_, h_PVW_mat_sp_cdepth_);
    } else {
      g_renderer_->bindFloat4x4(&PVW_mat_, h_PVW_mat_sp_depth_);
    }
    

    // Draw the current geometry
    geom->draw();
  }

  void HandModelRenderer::setHandRendering(bool render_hand) {
    if (render_hand_ != render_hand) {
      render_hand_ = render_hand;
      if (!render_hand_) {
        if (l_hand_geom_) {
          Geometry* g = l_hand_geom_->scene_graph();
          g->parent()->removeChild(g);
        }
        if (r_hand_geom_) {
          Geometry* g = r_hand_geom_->scene_graph();
          g->parent()->removeChild(g);
        }
      } else {
        if (l_hand_geom_) {
          Geometry* g = l_hand_geom_->scene_graph();
          GeometryManager::g_geom_manager()->scene_graph_root()->addChild(g);
        }
        if (r_hand_geom_) {
          Geometry* g = r_hand_geom_->scene_graph();
          GeometryManager::g_geom_manager()->scene_graph_root()->addChild(g);
        }
      }
    }
  }

  float HandModelRenderer::calcInterpenetrationTerm() {
    const uint32_t num_spheres = bsph_.size();
    // Assume bounding sphere matrices have been updated (they should be)!
    // For each bounding sphere transform the center and the radius:
    for (uint32_t i = 0; i < num_spheres; i++) {
      bsph_[i]->transform();
    }

    float total_penetration_distance = 0;
#ifdef PERFORM_BROAD_PHASE_COLLISION_TEST
    // Now sort the start and end points of each bounding box (described by the
    // bounding sphere) along each axis:
    insertionSortAxisExtents(bsph_sx, 0);
    insertionSortAxisExtents(bsph_sy, 1);
    insertionSortAxisExtents(bsph_sz, 2);
    insertionSortAxisExtents(bsph_fx, 3);
    insertionSortAxisExtents(bsph_fy, 4);
    insertionSortAxisExtents(bsph_fz, 5);

    // Now sweep along each axis, looking for overlaps
    findIntersections(bsph_sx, bsph_fx, 0, 3, collision_pairs_xaxis);
    findIntersections(bsph_sy, bsph_fy, 1, 4, collision_pairs_yaxis);
    findIntersections(bsph_sz, bsph_fz, 2, 5, collision_pairs_zaxis);

    // Now see which axis pairs intersect in all three collisions
    findCollisions();  // AABB collisions

    // Now check each potential collision for a sphere collision:
    Float3 vec;
    for (uint32_t i = 0; i < collisions_.size(); i++) {
      uint32_t objA_finger = collisions_[i].first / NSPH_PER_FING;
      uint32_t objB_finger = collisions_[i].second / NSPH_PER_FING;
      if (objA_finger != objB_finger) {
        BoundingSphere* objA = bsph_[collisions_[i].first];
        BoundingSphere* objB = bsph_[collisions_[i].second];
        Float3::sub(&vec, objA->transformed_center(), objB->transformed_center());
        float center_dist = vec.length();
        float min_dist = objA->transformed_radius() + objB->transformed_radius();
        float sep_dist = center_dist - min_dist + INTERPENETRATION_ALLOWENCE;
        if (sep_dist < 0) {
          total_penetration_distance += -sep_dist;
        }
      }
    }
#else
    // Just do O(n^2) tests
    Float3 vec;
    for (uint32_t i = 0; i < num_spheres; i++) {
      for (uint32_t j = i+1; j < num_spheres; j++) {
        uint32_t objA_finger = i / NSPH_PER_FING;
        uint32_t objB_finger = j / NSPH_PER_FING;
        if (objA_finger != objB_finger) {
          BoundingSphere* objA = bsph_[i];
          BoundingSphere* objB = bsph_[j];
          Float3::sub(&vec, objA->transformed_center(), objB->transformed_center());
          float center_dist = vec.length();
          float min_dist = objA->transformed_radius() + objB->transformed_radius();
          float sep_dist = center_dist - min_dist + INTERPENETRATION_ALLOWENCE;
          if (sep_dist < 0) {
            total_penetration_distance += -sep_dist;
          }
        }
      }
    }
#endif

    return total_penetration_distance * total_penetration_distance;
  }

  void HandModelRenderer::insertionSortAxisExtents(uint32_t* arr,
    uint32_t iextent) {
    for (uint32_t i = 1; i < bsph_.size(); i++) {
      uint32_t j = i;
      while (j > 0 && bsph_[arr[j - 1]]->extent(iextent) > bsph_[arr[j]]->extent(iextent)) {
        uint32_t tmp = arr[j];
        arr[j] = arr[j - 1];
        arr[j - 1] = tmp;
        j--;
      }
    }
  }

#define COLLISION_ARRAY_INDEX(i, j) ((i < j) ? \
  j * (NUM_BOUNDING_SPHERES*2) + i : i * (NUM_BOUNDING_SPHERES*2) + j)

  // This can definitely be faster, but is O(n+num_collisions) --> Instead you 
  // can keep track of insertion sort swaps and dynamically update an active 
  // list.
  void HandModelRenderer::findIntersections(uint32_t* min, uint32_t* max, 
    uint32_t iextent_min, uint32_t iextent_max, bool* col) {
    // Set all collisions to false and initialize the active list
    memset(col, 0, sizeof(col[0])*4*NUM_BOUNDING_SPHERES*NUM_BOUNDING_SPHERES);
    active_list_.resize(0);
    active_list_.pushBack(min[0]);

    //// DEBUG SPEW TO COPY AND PASTE INTO sweep_and_prune_test.m
    //cout << endl << "min indices: " << endl;
    //for (uint32_t i = 0; i < NUM_BOUNDING_SPHERES; i++) {
    //  cout << min[i] << ", ";
    //}
    //cout << endl << "min values: " << endl;
    //for (uint32_t i = 0; i < NUM_BOUNDING_SPHERES; i++) {
    //  cout << bsph_[min[i]]->extent(iextent_min) << ", ";
    //}
    //cout << endl << "max indices: " << endl;
    //for (uint32_t i = 0; i < NUM_BOUNDING_SPHERES; i++) {
    //  cout << max[i] << ", ";
    //}
    //cout << endl << "max values: " << endl;
    //for (uint32_t i = 0; i < NUM_BOUNDING_SPHERES; i++) {
    //  cout << bsph_[max[i]]->extent(iextent_max) << ", ";
    //}
    //cout << endl;

    uint32_t min_i = 1;
    uint32_t max_i = 0;

    while (min_i < bsph_.size()) {
      // See if the next start point is less than the next end point
      uint32_t next_start = min[min_i];
      uint32_t next_end = max[max_i];
      if (bsph_[next_start]->extent(iextent_min) < bsph_[next_end]->extent(iextent_max)) {
        // The next minimum is closer --> This object collides with everyone
        // on the active list (if they exist), so record it:
        for (uint32_t i = 0; i < active_list_.size(); i++) {
          uint32_t obj_a = active_list_[i];
          uint32_t obj_b = next_start;
          uint32_t col_index = COLLISION_ARRAY_INDEX(obj_a, obj_b);
          col[col_index] = true;
        }
        active_list_.pushBack(next_start);
        min_i++;
      } else {
        // The next maximum is closer, remove it from the active list:
        uint32_t i = 0;
        while (active_list_[i] != next_end) {
          i++;
        }
#if defined(DEBUG) || defined(_DEBUG)
        if (i >= active_list_.size()) {
          throw runtime_error("ERROR: Went over the end of active list!");
        }
#endif
        active_list_[i] = active_list_[active_list_.size()-1];
        active_list_.resize(active_list_.size()-1);
        max_i++;
      }
    }
  }

  void HandModelRenderer::findCollisions() {
    // This could be much faster, but is probably not too bad O(n^2)
    collisions_.resize(0);
    for (uint32_t i = 0; i < bsph_.size() - 1; i++) {
      for (uint32_t j = i+1; j < bsph_.size(); j++) {
        uint32_t col_index = COLLISION_ARRAY_INDEX(j, i);
        if (collision_pairs_xaxis[col_index] &&
            collision_pairs_yaxis[col_index] &&
            collision_pairs_zaxis[col_index]) {
          data_str::Pair<uint32_t, uint32_t> col(j, i);
          collisions_.pushBack(col);
        }
      }
    }
  }

  void HandModelRenderer::addBSphere(renderer::BoundingSphere* sph) {
    bsph_.pushBack(sph);
  }

}  // namespace hand_model