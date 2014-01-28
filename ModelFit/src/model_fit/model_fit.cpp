#include <random>
#include <thread>
#include <iostream>
#include <stdexcept>
#include "model_fit/model_fit.h"
#include "model_fit/model_renderer.h"
#include "model_fit/pose_model.h"
#include "jtil/data_str/vector_managed.h"
#include "jtil/data_str/vector.h"
#include "jtil/string_util/string_util.h"
#include "jtil/math/pso_parallel.h"
#include "jtil/math/common_optimization.h"
#include "jtil/file_io/file_io.h"
#include "jtil/file_io/csv_handle_write.h"
#include "renderer/gl_state.h"
#include "renderer/camera/camera.h"
#include "renderer/texture/texture_renderable.h"
#include "renderer/texture/texture.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"
#include "kinect_interface/depth_images_io.h"
#include "kinect_interface/kinect_interface.h"  // depth_dim

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

using namespace jtil::math;
using namespace jtil::data_str;
using std::string;
using std::runtime_error;
using std::cout;
using std::endl;
using jtil::file_io::CSVHandleWrite;
using jtil::data_str::VectorManaged;
using renderer::GLState;
using namespace kinect_interface;
using renderer::Texture;
using renderer::TextureRenderable;
using namespace jtil::data_str;

namespace model_fit {
  
  // Static variables
  ModelFit* ModelFit::cur_fit_ = NULL;
  uint64_t ModelFit::func_eval_count_ = 0;
 
  ModelFit::ModelFit(const uint32_t num_models, 
    const uint32_t coeff_dim_per_model, const uint32_t num_cameras) {
    if (num_models == 0 || coeff_dim_per_model == 0) {
      throw std::wruntime_error("ModelFit::ModelFit() - ERROR: check inputs!");
    }

    save_next_image_set_ = false;
    num_cameras_ = num_cameras;
    num_models_ = num_models;
    coeff_dim_per_model_ = coeff_dim_per_model;
    coeff_dim_ = coeff_dim_per_model_ * num_models_;

    coeff_optim_prev_ = new float[coeff_dim_];
    coeff_optim_ = new float[coeff_dim_];
    coeff_tmp_ = new float[coeff_dim_];

    pso_ = new PSOParallel(coeff_dim_, PSO_SWARM_SIZE);
    pso_->max_iterations = PSO_MAX_ITERATIONS;
    pso_->delta_coeff_termination = PSO_DELTA_C_TERMINATION;

    kinect_depth_masked_ = new int16_t[depth_dim];

    model_renderer_ = new ModelRenderer(num_cameras_);
  }

  ModelFit::~ModelFit() {
    SAFE_DELETE(pso_);
    SAFE_DELETE_ARR(kinect_depth_masked_);
    SAFE_DELETE_ARR(coeff_optim_);
    SAFE_DELETE_ARR(coeff_optim_prev_);
    SAFE_DELETE_ARR(coeff_tmp_);
    SAFE_DELETE(model_renderer_);
  }

  void ModelFit::prepareKinectData(int16_t** depth) {
    for (uint32_t i_camera = 0; i_camera < num_cameras_; i_camera++) {
      int16_t* cur_depth = depth[i_camera];
      memcpy(kinect_depth_masked_, cur_depth, sizeof(kinect_depth_masked_[0]) * 
        depth_dim);
      model_renderer_->uploadDepth(i_camera, kinect_depth_masked_);
    }
  }

  void ModelFit::setCameraView(const uint32_t i_camera, 
    const jtil::math::Float4x4& view) {
    model_renderer_->camera(i_camera)->set_view_mat_directly = true;
    model_renderer_->camera(i_camera)->view()->set(view);
  }

  void ModelFit::getCameraView(const uint32_t i_camera, 
    jtil::math::Float4x4& view) {
    view.set(*model_renderer_->camera(i_camera)->view());
  }

  void ModelFit::fitModel(int16_t** depth, PoseModel** models, float** coeffs, 
    float** prev_coeffs, CoeffUpdateFuncPtr coeff_update_func) {
    Vector<bool> old_attachement_vals(num_models_);
    prepareOptimization(depth, models, coeffs, prev_coeffs,
      old_attachement_vals);

    // This is a hack: There's something wrong with the first iteration using
    // tiled rendering --> Doing one regular render pass helps.
    float start_func_val = objectiveFunc(coeff_optim_);
    cout << "Starting objective function value = ";
    printf("%.9e", start_func_val);
    cout << endl;
    
    // PSO fitting
    for (uint32_t i = 0; i < PSO_REPEATS; i++) {
      pso_->minimize(coeff_tmp_, coeff_optim_, models_[0]->pso_radius_c(),
         models_[0]->angle_coeffs(), objectiveFuncTiled, coeff_update_func);
      // Now copy back the new coeff values
      memcpy(coeff_optim_, coeff_tmp_, sizeof(coeff_optim_[0]) * coeff_dim_);
    }

    // Copy the local coeff values back into the return value
    for (uint32_t i = 0; i < num_models_; i++) {
      memcpy(coeffs[i], &coeff_optim_[i * coeff_dim_per_model_], 
        sizeof(coeffs[i][0]) * coeff_dim_per_model_);
    }

    for (uint32_t i = 0; i < num_models_; i++) {
      models_[i]->setRendererAttachement(old_attachement_vals[i]);
    }

    float end_func_val = objectiveFunc(coeff_optim_);
    cout << "Final objective function value = ";
    printf("%.9e", end_func_val);
    cout << endl;
  }


  void ModelFit::prepareOptimization(int16_t** depth, PoseModel** models, 
    float** coeffs, float** prev_coeffs, Vector<bool>& old_attachement_vals) {
    models_ = models;
    
    // Detach the models from the global renderer so that there aren't issues
    // with scene graph parent transformation inheritance.
    for (uint32_t i = 0; i < num_models_; i++) {
      old_attachement_vals.pushBack(models_[i]->getRendererAttachement());
      models_[i]->setRendererAttachement(false);
    }

    for (uint32_t i = 0; i < num_models_; i++) {
      memcpy(&coeff_optim_[i * coeff_dim_per_model_], coeffs[i], 
        sizeof(coeff_optim_[0]) * coeff_dim_per_model_);
    }
    if (prev_coeffs != NULL) {
      for (uint32_t i = 0; i < num_models_; i++) {
        memcpy(&coeff_optim_prev_[i * coeff_dim_per_model_], prev_coeffs[i], 
          sizeof(coeff_optim_prev_[0]) * coeff_dim_per_model_);
      }
    } else {
      memcpy(coeff_optim_prev_, coeff_optim_, 
        sizeof(coeff_optim_prev_[0]) * coeff_dim_);
    }

    cur_fit_ = this;
    prepareKinectData(depth);
  }

  // queryObjFunc is mostly for debugging.  It saves a set of images so that
  // we can test the result against matlab's.  It is NOT meant to be fast.
  float ModelFit::queryObjFunc(int16_t** depth, PoseModel** models, 
    float** coeffs) {
    Vector<bool> old_attachement_vals(num_models_);
    prepareOptimization(depth, models, coeffs, NULL, old_attachement_vals);

    save_next_image_set_ = true;
    float func_val = objectiveFunc(coeff_optim_, false);
    cout << "objective function value (no constraints) = ";
    printf("%.9e", func_val);
    cout << endl;
    func_val = objectiveFunc(coeff_optim_, true);
    cout << "objective function value (with constraints) = ";
    printf("%.9e", func_val);
    cout << endl;

    save_next_image_set_ = true;
    jtil::data_str::Vector<float*> coeff_vals;
    jtil::data_str::Vector<float> residue_vals;
    jtil::data_str::Vector<float> residue_vals_wconst;
    // Create a set of coeffs by just preturbing the coeffs a little bit.
    MERSINE_TWISTER_ENG eng;
    UNIFORM_REAL_DISTRIBUTION c_dist(-0.5f, 0.5f);
    for (uint32_t t = 0; t < NTILES; t++) {
      float* cur_coeffs = new float[num_models_ * coeff_dim_per_model_];
      for (uint32_t i = 0; i < num_models_; i++) {
        for (uint32_t j = 0; j < coeff_dim_per_model_; j++) {
          cur_coeffs[i * coeff_dim_per_model_ + j] = coeffs[i][j] + c_dist(eng);
        }
      }
      coeff_vals.pushBack(cur_coeffs);
      residue_vals.pushBack(std::numeric_limits<float>::infinity());
      residue_vals_wconst.pushBack(std::numeric_limits<float>::infinity());
    }
    objectiveFuncTiled(residue_vals, coeff_vals, false);
    objectiveFuncTiled(residue_vals_wconst, coeff_vals, true);
    cout << "tiled objective function value (no constraints) = " << endl;
    for (uint32_t v = 0; v < NTILES_DIM; v++) {
      for (uint32_t u = 0; u < NTILES_DIM; u++) {
        uint32_t t = v * NTILES_DIM + u;
        SAFE_DELETE_ARR(coeff_vals[t]);
        printf("  %.9e", residue_vals[t]);
      }
      std::cout << endl;
    }
    cout << "tiled objective function value (with constraints) = " << endl;
    for (uint32_t v = 0; v < NTILES_DIM; v++) {
      for (uint32_t u = 0; u < NTILES_DIM; u++) {
        uint32_t t = v * NTILES_DIM + u;
        printf("  %.9e", residue_vals_wconst[t]);
      }
      std::cout << endl;
    }

    for (uint32_t i = 0; i < num_models_; i++) {
      models_[i]->setRendererAttachement(old_attachement_vals[i]);
    }

    return func_val;
  }

  char* float2CStr(float val) {
    string str = jtil::string_util::Num2Str<float>(val);
    char* c_str = new char[str.length()+1];
    std::copy(str.begin(), str.end(), c_str);
    c_str[str.size()] = '\0';
    return c_str;
  }
  
  // modulu - similar to matlab's mod()
  // result is always possitive. not similar to fmod()
  // Mod(-3,4)= 1   fmod(-3,4)= -3
#if defined(__APPLE__) || defined(_WIN32)
  float inline __fastcall Mod(float x, float y) {
    if (0 == y) {
      return x;
    }
    
    return x - y * floor(x / y);
  }
#else
  float inline Mod(float x, float y) {
    if (0 == y) {
      return x;
    }

    return x - y * floor(x / y);
  }
#endif
  
#if defined(WIN32) || defined(_WIN32)
  std::tr1::mt19937 eng;  // a core engine class
  std::tr1::normal_distribution<float> dist;
#else
  std::mt19937 eng;  // a core engine class
  std::normal_distribution<float> dist;
#endif

  void ModelFit::calculateResidual(const float* coeff, const uint32_t i_camera,
    float* depth_term, float* penalty_term, float* interpen_term) {
    // Residue is calculated on the GPU
    if (depth_term) {
      *depth_term += 
        cur_fit_->model_renderer_->calculateResidualDataTerm(i_camera);
    }
    
    if (interpen_term) {
      const uint32_t max_groups = cur_fit_->models_[0]->max_bsphere_groups();
      *interpen_term += 
        cur_fit_->model_renderer_->calcInterpenetrationTerm(max_groups);
    }

    if (penalty_term) {
      *penalty_term += calcPenalty(coeff);  // formally scaled by 0.1
    }
  }

  void ModelFit::calculateResidualTiled(Vector<float>* depth_term, 
    Vector<float>* penalty_term, Vector<float*>& coeffs, 
    const uint32_t i_camera) {
    // Note: at this point interpenetration term is already calculated and is
    // sitting in residues[i]
    if (depth_term) {
      cur_fit_->model_renderer_->calculateResidualDataTermTiled(*depth_term, 
        i_camera);
    }

    // Now calculate the penalty terms:
    if (penalty_term) {
      for (uint32_t i = 0; i < coeffs.size(); i++) {
        (*penalty_term)[i] += calcPenalty(coeffs[i]);
      }
    }
  }

  float ModelFit::calcPenalty(const float* coeff) {
    float penalty = 1.0f;
    const uint32_t coeff_dim = cur_fit_->coeff_dim_;
    const uint32_t coeff_dim_per_model = cur_fit_->coeff_dim_per_model_;
    const float* penalty_scale = cur_fit_->models_[0]->coeff_penalty_scale();
    const float* max_limit = cur_fit_->models_[0]->coeff_max_limit();
    const float* min_limit = cur_fit_->models_[0]->coeff_min_limit();

    for (uint32_t i = 0; i < coeff_dim; i++) {
      if (penalty_scale[i % coeff_dim_per_model] > EPSILON) {
        float cur_coeff_val = coeff[i];

#ifdef LINEAR_PENALTY
        // Linear penalty
        if (cur_coeff_val > max_limit[i % coeff_dim_per_model]) {
          penalty += penalty_scale[i % coeff_dim_per_model] * 
            fabsf(cur_coeff_val - max_limit[i % coeff_dim_per_model]) / 10.0f;
        }
        if (cur_coeff_val < min_limit[i % coeff_dim_per_model]) { 
          penalty += penalty_scale[i % coeff_dim_per_model] * 
            fabsf(min_limit[i % coeff_dim_per_model] - cur_coeff_val) / 10.0f;
        }
#else
        // Quadratic penalty
        if (cur_coeff_val > max_limit[i % coeff_dim_per_model]) {
          float cur_penalty = penalty_scale[i % coeff_dim_per_model] * 
            fabsf(cur_coeff_val - max_limit[i % coeff_dim_per_model]) / 2.0f;
          penalty += (cur_penalty * cur_penalty);
        }
        if (cur_coeff_val < min_limit[i % coeff_dim_per_model]) { 
          float cur_penalty = penalty_scale[i % coeff_dim_per_model] * 
            fabsf(min_limit[i % coeff_dim_per_model] - cur_coeff_val) / 2.0f;
          penalty += (cur_penalty * cur_penalty);
        }
#endif
      }
    }

#ifdef PREV_FRAME_DIST_PENALTY
      penalty += calcDistPenalty(cur_fit_->coeff_optim_prev_, coeff);
#endif

    return penalty;
  }

  float ModelFit::calcDistPenalty(const float* coeff0, const float* coeff1) {
    float dist = 0;  // Prevent NANs
    const uint32_t coeff_dim_per_model = cur_fit_->coeff_dim_per_model_;
    const float* penalty_scale = cur_fit_->models_[0]->coeff_penalty_scale();
    const float* max_limit = cur_fit_->models_[0]->coeff_max_limit();
    const float* min_limit = cur_fit_->models_[0]->coeff_min_limit();
    for (uint32_t i = 0; i < cur_fit_->coeff_dim_; i++) {
      if (penalty_scale[i % coeff_dim_per_model] > EPSILON) {
        float delta = fabsf(coeff0[i] - coeff1[i]);
        float err = delta - PREV_FRAME_DIST_PENALTY_THRESHOLD * 
          (max_limit[i] - min_limit[i]);
        if (err > 0) {
          dist += err * err;
        }
      }
    }
    return PREV_FRAME_DIST_PENALTY_SCALE * dist;
  }

  float ModelFit::objectiveFunc(const float* coeff, 
    const bool include_constraints) {
    float depth_term = 0.0f;
    float penalty_term = 0.0f;
    float interpenetration_term = 0.0f;
    for (uint32_t i_camera = 0; i_camera < cur_fit_->num_cameras_; i_camera++) {
      cur_fit_->model_renderer_->drawDepthMap(coeff, 
        cur_fit_->coeff_dim_per_model_, cur_fit_->models_, 
        cur_fit_->num_models_, i_camera, false);
      // Only calculate interpenetration and penalty terms on the first camera 
      // angle to save computation time
      if (i_camera == 0) {
        calculateResidual(coeff, i_camera, &depth_term, &penalty_term, 
          &interpenetration_term);
      } else {
        calculateResidual(coeff, i_camera, &depth_term, NULL, NULL);
      }

      if (cur_fit_->save_next_image_set_) {
        int32_t dim = cur_fit_->model_renderer_->tex_size();
        float* data_temp = new float[dim * dim];

        // Get the synthetic depth texture
        TextureRenderable* rtex = cur_fit_->model_renderer_->depth_texture();
        rtex->getTexture0Data<float>(data_temp);
        jtil::file_io::SaveArrayToFile<float>(data_temp, dim * dim, 
          "./synth_texture.bin");

        // Get the residue texture
        rtex = cur_fit_->model_renderer_->residue_texture();
        rtex->getTexture0Data<float>(data_temp);
        jtil::file_io::SaveArrayToFile<float>(data_temp, dim * dim, 
          "./residue_texture.bin");

        // Get the kinect texture.  Don't copy it back down, just get the CPU
        // version (which is a copy of what was sent to the GPU).
        Texture* tex = cur_fit_->model_renderer_->kinect_depth_textures()[0];
        tex->getTextureData<float>(data_temp);
        jtil::file_io::SaveArrayToFile<float>(data_temp, dim * dim, 
          "./kinect_texture.bin");

        cur_fit_->save_next_image_set_ = false;
        delete[] data_temp;
      }
    }

    if (include_constraints) {
      return depth_term * penalty_term * interpenetration_term;
    } else {
      return depth_term;
    }
  }

  void ModelFit::objectiveFuncTiled(Vector<float>& residues, 
    Vector<float*>& coeffs) {
    objectiveFuncTiled(residues, coeffs, true);
  }

  void ModelFit::objectiveFuncTiled(Vector<float>& residues, 
    Vector<float*>& coeffs, const bool include_constraints) {
    if (coeffs.size() > NTILES) {
      throw runtime_error("objectiveFuncTiled() - coeffs.size() > NTILES");
    }
    if (residues.capacity() < coeffs.size()) {
      residues.capacity(coeffs.size());
    }

    // Zero out some accumulators
    Vector<float> depth_term(coeffs.size());
    Vector<float> penalty_term(coeffs.size());
    Vector<float> interpenetration_term(coeffs.size());
    for (uint32_t i = 0; i < coeffs.size(); i++) {
      depth_term.pushBack(0);
      penalty_term.pushBack(0);
      interpenetration_term.pushBack(0);
    }
    
    for (uint32_t i_camera = 0; i_camera < cur_fit_->num_cameras_; i_camera++) {
      // Only calculate interpenetration and penalty terms on the first camera 
      // angle to save computation time
      if (i_camera == 0) {
        cur_fit_->model_renderer_->drawDepthMapTiled(coeffs, 
          cur_fit_->coeff_dim_per_model_, cur_fit_->models_, cur_fit_->num_models_, 
          i_camera, &interpenetration_term, 
          cur_fit_->models_[0]->max_bsphere_groups());
      } else {
        cur_fit_->model_renderer_->drawDepthMapTiled(coeffs, 
          cur_fit_->coeff_dim_per_model_, cur_fit_->models_, cur_fit_->num_models_, 
          i_camera, NULL, cur_fit_->models_[0]->max_bsphere_groups());
      }
      if (i_camera == 0) {
        calculateResidualTiled(&depth_term, &penalty_term, coeffs, i_camera);
      } else {
        calculateResidualTiled(&depth_term, NULL, coeffs, i_camera);
      }

      if (cur_fit_->save_next_image_set_) {
        int32_t dim = cur_fit_->model_renderer_->tex_size();
        float* data_temp = new float[dim * dim * NTILES];

        // Get the synthetic depth texture
        TextureRenderable* rtex = cur_fit_->model_renderer_->depth_texture_tiled();
        rtex->getTexture0Data<float>(data_temp);
        jtil::file_io::SaveArrayToFile<float>(data_temp, dim * dim * NTILES, 
          "./synth_texture_tiled.bin");

        // Get the residue texture
        rtex = cur_fit_->model_renderer_->residue_texture_tiled();
        rtex->getTexture0Data<float>(data_temp);
        jtil::file_io::SaveArrayToFile<float>(data_temp, dim * dim * NTILES, 
          "./residue_texture_tiled.bin");

        // Get the kinect texture.  Don't copy it back down, just get the CPU
        // version (which is a copy of what was sent to the GPU).
        Texture* tex = cur_fit_->model_renderer_->kinect_depth_textures_tiled()[0];
        tex->getTextureData<float>(data_temp);
        jtil::file_io::SaveArrayToFile<float>(data_temp, dim * dim * NTILES, 
          "./kinect_texture_tiled.bin");

        cur_fit_->save_next_image_set_ = false;
        delete[] data_temp;
      }
    }
    func_eval_count_ += NTILES;

    // Now calculate the final residue term
    residues.resize(0);
    for (uint32_t i = 0; i < coeffs.size(); i++) {
      if (include_constraints) {
        residues.pushBack(depth_term[i] * penalty_term[i] * 
          interpenetration_term[i]);
      } else {
        residues.pushBack(depth_term[i]);
      }
    }
  }

}  // namespace hand_fit
