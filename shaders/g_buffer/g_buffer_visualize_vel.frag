#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;

void main(){
  vec2 vec = SampleGBufferVel(f_texture, f_texture_sampler);
  color = vec4(vec.x, vec.y, 0.0, 1.0);
}