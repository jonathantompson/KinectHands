#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec2 f_texture;

out vec4 f_out_0;  // View space depth and normal
out vec4 f_out_1;  // albedo color and specular intensity
out vec4 f_out_2;  // shadow map split, spec power, velocity

const float large_z = 1e12;

uniform sampler2D f_texture_sampler;
uniform int f_strech_tex;
uniform float f_tex_aspect;
uniform float f_screen_aspect;
uniform vec3 f_clear_color;

void main(void) {
  vec3 clear_color;
  vec2 tex_coords = f_texture;
  if (f_strech_tex != 0) {
    clear_color = vec3(texture(f_texture_sampler, tex_coords).xyz);
  } else {
    if (f_tex_aspect >= f_screen_aspect) {
      float ratio = f_tex_aspect / f_screen_aspect;
      tex_coords.y = tex_coords.y * ratio - ((ratio - 1) * 0.5);
      if (tex_coords.y > 1.0 || tex_coords.y < 0) {
        clear_color = f_clear_color;
      } else {
        clear_color = vec3(texture(f_texture_sampler, tex_coords).xyz);
      }
    } else {
      float ratio = f_screen_aspect / f_tex_aspect;
      tex_coords.x = tex_coords.x * ratio - ((ratio - 1) * 0.5);
      if (tex_coords.x > 1.0 || tex_coords.x < 0) {
        clear_color = f_clear_color;
      } else {
        clear_color = vec3(texture(f_texture_sampler, tex_coords).xyz);
      }
    }
  }
  StoreGBuffer(f_out_0, f_out_1, f_out_2,  // output textures
               vec3(0.0, 0.0, 0),          // in vec3 position_view, 
               vec3(0.0, 0.0, -1.0),       // in vec3 norm, 
               clear_color,                // in vec3 diffuse, 
               0,                          // in float spec_intensity, 
               0,                          // in float spec_power, 
			   0,                          // in float lighting_stencil     
			   vec2(0,0));                 // in vec2 velocity                          
  f_out_0.x = -large_z;  // Force normalized depth to beyond the far plane
}