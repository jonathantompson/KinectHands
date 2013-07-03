#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;

void main(){
  vec2 norm2D = vec2(texture(f_texture_sampler, f_texture).yz);
  color = vec4(TextureToNormal(norm2D), 1.0);
}