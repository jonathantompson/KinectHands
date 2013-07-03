#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec2 f_rgb_coord;
in vec3 f_normal_view;
in vec3 f_position_view;

out vec4 f_out_0;  // View space depth and normal
out vec4 f_out_1;  // albedo color and specular intensity
out vec4 f_out_2;  // shadow map split, spec power, velocity

uniform float f_spec_intensity;  // Default 1
uniform float f_spec_power;  // Default 32
uniform float f_lighting_stencil;
uniform vec3 f_const_albedo;
uniform sampler2D f_rgb_tex;

void main(void) {
  vec3 f_color = texture(f_rgb_tex, f_rgb_coord).xyz;
  StoreGBuffer(f_out_0, f_out_1, f_out_2,  // output textures
			   f_position_view,            // in vec3 position_view, 
			   f_normal_view,              // in vec3 norm, 
			   f_color,	                   // in vec3 diffuse, 
			   f_spec_intensity,           // in float spec_intensity, 
			   f_spec_power,               // in float spec_power, 
			   f_lighting_stencil,         // in float f_lighting_stencil 
			   vec2(0,0));                 // in vec2 vel
}
