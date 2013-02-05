//
//  camera.cpp
//
//  Created by Jonathan Tompson on 6/3/12.
//

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#ifdef _WIN32
#include <windows.h>
#endif
#include <GLUT/glut.h>
#endif
#include "camera/camera.h"
#include "math/math_types.h"

using math::Float3;
using math::FloatQuat;

namespace renderer {

  Camera::Camera() {
    eye_rot_.identity();
    eye_pos_.set(0.0f, 0.0f, 100.0f);  // Start off by looking down -z
    proj_.identity();
    x_axis_rot_ = 0;
    y_axis_rot_ = 0;
  }

  Camera::~Camera() {
    // Empty destructor
  }

  void Camera::updateView() {
    // Before updating view, record the old value
    view_prev_frame_.set(&view_);
    
    // Find the inverse rotation using quaternion
    math::FloatQuat::inverse(&eye_rot_inv_, &eye_rot_);  // Very fast
    eye_rot_inv_.quat2Mat4x4(&view_);
    
    view_[2] *= -1.0f;
    view_[6] *= -1.0f;
    view_[10] *= -1.0f;

    view_.multTranslation(eye_pos_[0], eye_pos_[1], eye_pos_[2]);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glLoadMatrixf(view_.m);

  }

  void Camera::updateProjection(int screen_width, int screen_height, 
                                float field_of_view, float view_plane_near, 
                                float view_plane_far) {
    // Before updating projection, record the old value
    proj_prev_frame_.set(&proj_);

    proj_.glProjection(view_plane_near, view_plane_far, field_of_view,
      static_cast<float>(screen_width), static_cast<float>(screen_height));
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glLoadMatrixf(proj_.m);

  }
  
  void Camera::rotateCamera(float theta_x, float theta_y) {
    y_axis_rot_ += theta_x;
    // Keep between -pi and +pi (allow wrap around rotations)
    y_axis_rot_ = (y_axis_rot_ > static_cast<float>(M_PI)) ? y_axis_rot_ - 
                  2.0f*static_cast<float>(M_PI) : y_axis_rot_;
    y_axis_rot_ = (y_axis_rot_ < -static_cast<float>(M_PI)) ? y_axis_rot_ + 
                  2.0f*static_cast<float>(M_PI) : y_axis_rot_;
    // Clamp between -pi_2 and +pi_2
    x_axis_rot_ += theta_y;
    x_axis_rot_ = (x_axis_rot_ > static_cast<float>(M_PI_2)) ? 
                  static_cast<float>(M_PI_2) : x_axis_rot_;
    x_axis_rot_ = (x_axis_rot_ < -static_cast<float>(M_PI_2)) ? 
                  -static_cast<float>(M_PI_2) : x_axis_rot_;

    // Rotate by y-axis first
    eye_rot_.identity();
    math::FloatQuat rot_mouse;
    rot_mouse.yAxisRotation(y_axis_rot_);
    math::FloatQuat rot_tmp;
    rot_tmp.mult(&eye_rot_, &rot_mouse);
    // Now rotate by x-axis
    rot_mouse.xAxisRotation(x_axis_rot_);
    eye_rot_.mult(&rot_tmp, &rot_mouse);
  }
  
  void Camera::moveCamera(Float3* dir_eye_space) {
    // First calcuate the direction in world coords
    Float3 dir_world_coords;
    dir_world_coords.affineTransformVec(&view_, dir_eye_space);
    eye_pos_.accum(dir_world_coords.m);
  }

}  // namespace renderer
