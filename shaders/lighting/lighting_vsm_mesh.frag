#version 150
#include "./shaders/lighting/lighting_include.frag"

in vec3 f_position_view;

uniform vec2 f_light_near_far;
uniform float f_vsm_depth_epsilon;

out vec2 vsm;  // View space depth and depth squared

void main(void) {
  // Rescale Depth into [0, 1]
  // Note, we need length(f_position_view) since we want distance from the
  // light and since the projection matrix might be perspective, this is not
  // necessarily just viewspace depth.
  float depth = linstep(-f_light_near_far.x, -f_light_near_far.y, 
    length(f_position_view)) + f_vsm_depth_epsilon;
  vec2 moments = ComputeMoments(depth);
  vsm = vec2(moments.x, moments.y);
  vsm = vsm;
}
