#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec2 f_texture;
uniform sampler2D f_texture_sampler;

out vec4 color;

void main(){
  vec4 accum_sample = texture(f_texture_sampler, f_texture);
  vec3 diffuse = accum_sample.xyz;
  color = vec4(diffuse, 1.0);
}