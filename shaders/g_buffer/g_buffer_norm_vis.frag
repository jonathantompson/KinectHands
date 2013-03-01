#version 330 core
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec3 color;

out vec4 f_out_0;  // View space depth and normal
out vec4 f_out_1;  // albedo color and specular intensity
out vec4 f_out_2;  // shadow map split, spec power, velocity

void main(void) {
  StoreGBuffer(f_out_0, f_out_1, f_out_2,  // output textures
			   vec3(0,0,0),                // in vec3 position_view, 
			   vec3(0,0,0),                // in vec3 norm, 
			   color,                      // in vec3 diffuse, 
			   0,                          // in float spec_intensity, 
			   0,                          // in float spec_power, 
			   0,                          // in float f_lighting_stencil 
			   vec2(0.0, 0.0));            // in vec2 vel
}
