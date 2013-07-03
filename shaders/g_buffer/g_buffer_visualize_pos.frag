#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;

void main(){
  float depth = texture(f_texture_sampler, f_texture).x;
  vec3 view_pos = GetPositionFromDepth(f_texture, depth);
  color = vec4(view_pos.x, view_pos.y, view_pos.z, 1.0);
}