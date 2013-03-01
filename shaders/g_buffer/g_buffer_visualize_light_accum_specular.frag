#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;

void main(){
  vec4 accum_sample = texture(f_texture_sampler, f_texture);
  float spec = accum_sample.w;
  color = vec4(spec, spec, spec, 1.0);
}