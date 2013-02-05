//
//  camera.h
//
//  Created by Jonathan Tompson on 6/3/12.
//

#ifndef RENDERER_CAMERA_HEADER
#define RENDERER_CAMERA_HEADER

#include "math/math_types.h"

namespace renderer {
  class Renderer;

  class Camera {
  public:
    Camera(math::FloatQuat* eye_rot, math::Float3* eye_pos, int screen_width,
      int screen_height, float field_of_view, float near, float far);
    ~Camera();

    void updateView();
    void updateProjection();
    void rotateCamera(float theta_x, float theta_y);
    void moveCamera(math::Float3* dir_eye_space);

    // getter / setter methods
    inline math::Float4x4* view() { return &view_; }
    inline math::Float4x4* proj() { return &proj_; }
    inline math::Float2* near_far() { return &near_far_; }
    inline void near_far(math::Float2* set_val) { near_far_.set(set_val); }
    inline math::Float2* tangent_fov() { return &tangent_fov_; }
    inline math::Float2* cur_screen_size() { return &screen_size_; }
    inline void field_of_view(float set_val) { field_of_view_ = set_val; }
    inline void screen_width(float set_val) { screen_size_[0] = set_val; }
    inline void screen_height(float set_val) { screen_size_[1] = set_val; }
    inline math::Float3* eye_pos() { return &eye_pos_; }

  private:
    math::FloatQuat eye_rot_;
    math::FloatQuat eye_rot_inv_;
    math::Float3 eye_pos_;
    float x_axis_rot_;
    float y_axis_rot_;
    math::Float2 near_far_;
    math::Float2 tangent_fov_;
    math::Float2 screen_size_;
    float field_of_view_;

    math::Float4x4 view_;
    math::Float4x4 view_prev_frame_;
    math::Float4x4 proj_;
    math::Float4x4 proj_prev_frame_;  // Used for motion blur    
    math::Float4x4 view_inverse_;

    // Non-copyable, non-assignable.
    Camera(Camera&);
    Camera& operator=(const Camera&);
  };

};  // namespace renderer

#endif  // RENDERER_CAMERA_HEADER
