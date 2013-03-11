#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;

void main(){
  vec2 rg = vec4(texture(f_texture_sampler, f_texture).xyz, 1.0).xy;
  color = vec4(rg, 0.0, 1.0);
}