#include <string>
#include "renderer/camera/camera.h"
#include "renderer/open_gl_common.h"
#include "math/math_types.h"
#include "data_str/vector_managed.h"
#include "exceptions/wruntime_error.h"

using math::Float3;
using math::Float4x4;
using math::FloatQuat;
using std::wruntime_error;
using std::wstring;

namespace renderer {

  Camera::Camera(FloatQuat* eye_rot, Float3* eye_pos, int screen_width,
    int screen_height, float field_of_view, float near, float far) {
    proj_.identity();
    eye_rot_.set(eye_rot);
    math::FloatQuat::inverse(&eye_rot_inv_, &eye_rot_);
    eye_pos_.set(eye_pos);
    x_axis_rot_ = 0;
    y_axis_rot_ = 0;
    
    screen_size_.set(static_cast<float>(screen_width),
                     static_cast<float>(screen_height));
    field_of_view_ = field_of_view;
    if (near > 0 || far > 0) {
      throw wruntime_error(wstring(L"Camera::Camera() - ERROR: ") +
        wstring(L"near_far_[0] > 0 || near_far_[1] > 0! \n") +
        wstring(L"OpenGL convention is to look down the negative z axis!"));
    }
    near_far_[0] = near;
    near_far_[1] = far;
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

    view_.rightMultTranslation(eye_pos_[0], eye_pos_[1], eye_pos_[2]);
    Float4x4::affineRotationTranslationInverse(&view_inverse_, &view_);
  }

  void Camera::updateProjection() {
    // Before updating projection, record the old value
    proj_prev_frame_.set(&proj_);

    // Calculate tangent of the field of view --> Used for unprojecting view
    // space depth in shaders.
    float aspect = screen_size_[0] / screen_size_[1];
    float tanFovY = tanf(field_of_view_ / 2.0f);
    tangent_fov_[0] = tanFovY * aspect;
    tangent_fov_[1] = tanFovY;

    // Recall: OpenGL convention is to look down the negative Z axis,
    //         therefore, more negative values are actually further away.
    proj_.glProjection(-near_far_[0], -near_far_[1], field_of_view_,
      screen_size_[0], screen_size_[1]);
  }
  
  void Camera::rotateCamera(float theta_x, float theta_y) {
    y_axis_rot_ += theta_x;
    // Keep between -pi and +pi (allow wrap around rotations)
    y_axis_rot_ = (y_axis_rot_ > static_cast<float>(M_PI)) ?
      y_axis_rot_ - 2.0f*static_cast<float>(M_PI) : y_axis_rot_;
    y_axis_rot_ = (y_axis_rot_ < -static_cast<float>(M_PI)) ?
      y_axis_rot_ + 2.0f*static_cast<float>(M_PI) : y_axis_rot_;
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
    dir_world_coords.affineTransformVec(&view_inverse_, dir_eye_space);
    eye_pos_.accum(dir_world_coords.m);
  }

}  // namespace renderer