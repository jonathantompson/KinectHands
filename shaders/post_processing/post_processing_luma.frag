#version 150

in vec2 f_texture;

out float luma;

uniform sampler2D f_src_rgb;
const vec3 luma_const = vec3(0.299, 0.587, 0.114);

void main(){
  vec3 rgb = texture(f_src_rgb, f_texture).xyz;
  luma = dot(rgb, luma_const);
}
