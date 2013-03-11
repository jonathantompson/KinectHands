#version 150
#include "./shaders/lighting/lighting_include.frag"

in vec2 f_texture;

uniform sampler2D f_depth_normal_view_stencil;
uniform sampler2D f_ssao_buffer;
uniform vec2 f_texel_size;

out float ambient;

const int kernel_size = 4;

//The kernel distribution
// In matlab:
// x = -4:1:4;  sigma = 1; sigma_sq = sigma^2; 
// w = (1 / sqrt(2*pi*sigma_sq)) * exp(-(x.^2 / (2*sigma^2)))

// Sigma = 1:
//const float weights[2*kernel_size + 1] = float[](0.0001, 0.0044, 0.0540, 
//  0.2420, 0.3989, 0.2420, 0.0540, 0.0044, 0.0001);

// Sigma = 1.5:
const float weights[2*kernel_size + 1] = float[](0.0076, 0.0360, 0.1093, 
  0.2130, 0.2660, 0.2130, 0.1093, 0.0360, 0.0076);

void main(){
  ambient = 0.0;
  float sum = 0.0001;

  // Sample the center position
  float pos_center = texture(f_depth_normal_view_stencil, f_texture).x;

  for (int i = -kernel_size; i <= kernel_size; i++) {
    // Sample the new position
    vec2 cur_f_texture = f_texture + float(i) * f_texel_size;
    float pos_sample = texture(f_depth_normal_view_stencil, cur_f_texture).x;

	// Calculate the sample weight
    float g_coeff = weights[i+kernel_size];
	float z_diff = pos_center - pos_sample;
	float z_coeff = 1.0 / (0.01 + z_diff*z_diff);
	float final_coeff = g_coeff * z_coeff;
	sum += final_coeff;
	ambient += final_coeff *  texture(f_ssao_buffer, cur_f_texture).x;
  }

  ambient = clamp(ambient / sum, 0, 1);
}