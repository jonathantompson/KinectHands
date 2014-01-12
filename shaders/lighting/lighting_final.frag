#version 150
#include "./shaders/lighting/lighting_include.frag"

in vec2 f_texture;

uniform sampler2D f_depth_normal_view_stencil;
uniform sampler2D f_albedo_spec_intensity;
uniform sampler2D f_accumulation_tex;
uniform sampler2D f_ambient_occ;
uniform float f_global_ambient;

out vec4 color; 

void main() {
  
  float stencil = SampleGBufferStencil(f_texture, f_depth_normal_view_stencil);
  vec3 albedo = SampleGBufferAlbedo(f_texture, f_albedo_spec_intensity);
  float ambient = texture(f_ambient_occ, f_texture).x;
  vec4 accumulation_tex_val = texture(f_accumulation_tex, f_texture);
  vec3 diffuse = accumulation_tex_val.xyz;
  float specular = accumulation_tex_val.w;

  // Now calculate the final color
  color = vec4(albedo * diffuse +
               albedo * (ambient * f_global_ambient + specular), 1.0);
  // If the stencil buffer is 0.0 --> just send down the albedo
  color = color * stencil + (1 - stencil) * vec4(albedo, 1.0);
  
}
