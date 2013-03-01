#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;

void main(){
  vec3 albedo = vec3(texture(f_texture_sampler, f_texture).xyz);
  color = vec4(albedo, 1.0);
}