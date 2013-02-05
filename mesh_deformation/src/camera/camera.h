//
//  camera.h
//
//  Created by Jonathan Tompson on 6/3/12.
//

#ifndef RENDERER_CAMERA_HEADER
#define RENDERER_CAMERA_HEADER

#include "math/math_types.h"

namespace renderer {

  class Camera {
  public:
    Camera();
    ~Camera();

    void updateView();
    void updateProjection(int screen_width, int screen_height, 
                          float field_of_view, float view_plane_near,
                          float view_plane_far);
    void rotateCamera(float theta_x, float theta_y);
    void moveCamera(math::Float3* dir_eye_space);

    static void calcViewMat(math::Float3 up, math::Float3 forward);

    // getter setter methods
    inline math::Float4x4* view() { return &view_; }
    inline math::Float4x4* proj() { return &proj_; }

  private:
    math::FloatQuat eye_rot_;
    math::FloatQuat eye_rot_inv_;
    math::Float3 eye_pos_;
    float x_axis_rot_;
    float y_axis_rot_;

    math::Float4x4 view_;
    math::Float4x4 view_prev_frame_;
    math::Float4x4 proj_;
    math::Float4x4 proj_prev_frame_;  // Used for motion blur    

    // Non-copyable, non-assignable.
    Camera(Camera&);
    Camera& operator=(const Camera&);
  };

};  // namespace renderer

#endif  // RENDERER_CAMERA_HEADER
