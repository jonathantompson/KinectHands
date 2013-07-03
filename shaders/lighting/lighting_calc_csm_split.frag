#version 150
#include "./shaders/lighting/lighting_include.frag"

in vec2 f_texture;

out int sm_split;

uniform sampler2D f_depth_normal_view_stencil;
uniform float f_vsm_split_depths[max_sm_count + 1];
uniform int f_vsm_count;

const int split_pow_lookup[32] = int[](0, 1, 1, 2, 2, 2, 2, 3,   // 1,  8
                                       3, 3, 3, 3, 3, 3, 3, 4,   // 9,  16
                                       4, 4, 4, 4, 4, 4, 4, 4,   // 17, 24
                                       4, 4, 4, 4, 4, 4, 4, 5);  // 25, 32

int GetSMSplit(float depth) {
  int split = 0;
  for(int i = 1; i <= f_vsm_count; i ++) {
    split += (depth < f_vsm_split_depths[i]) ? 1 : 0;
  }
  return split;
}

void main(){
  float depth = texture(f_depth_normal_view_stencil, f_texture).x;
  sm_split = GetSMSplit(depth);
}