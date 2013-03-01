#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;

void main(){
  vec2 rgb = vec4(texture(f_texture_sampler, f_texture).xyz, 1.0).xyz;
  color = vec4(rgb, 1.0);
}