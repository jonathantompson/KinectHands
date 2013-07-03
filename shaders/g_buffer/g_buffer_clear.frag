#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec2 f_texture;

out vec4 f_out_0;  // View space depth and normal
out vec4 f_out_1;  // albedo color and specular intensity
out vec4 f_out_2;  // shadow map split, spec power, velocity

const float large_z = 1e12;

uniform vec3 f_clear_color;

void main(void) {
  StoreGBuffer(f_out_0, f_out_1, f_out_2,  // output textures
               vec3(0.0, 0.0, 0),          // in vec3 position_view, 
               vec3(0.0, 0.0, -1.0),       // in vec3 norm, 
               f_clear_color,              // in vec3 diffuse, 
               0,                          // in float spec_intensity, 
               0,                          // in float spec_power, 
			   0,                          // in float lighting_stencil     
			   vec2(0,0));                 // in vec2 velocity                          
  f_out_0.x = -large_z;  // Force normalized depth to beyond the far plane
}