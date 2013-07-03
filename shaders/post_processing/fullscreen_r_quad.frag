#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;

void main(){
  float red = vec4(texture(f_texture_sampler, f_texture).xyz, 1.0).x;
  color = vec4(red, red, red, 1.0);
}