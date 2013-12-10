#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_vsm;

void main(){
  vec2 moments = texture(f_vsm, f_texture).xy;
  color = vec4(moments.x, moments.x, moments.x, 1.0);
}
