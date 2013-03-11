#version 150
precision lowp float;
#include "./shaders/lighting/lighting_include.frag"

in vec2 f_texture;

uniform sampler2D f_depth_normal_view_stencil;
uniform sampler2D f_vector_noise;
uniform vec2 f_one_over_vector_noise_size;
uniform float f_ssao_scale;
uniform float f_ssao_bias;
uniform float f_ssao_intensity;
uniform float f_ssao_sample_radius;

out float ambient; 

// Generate a random 2D vector
vec2 getRandom(in vec2 tex_coord) {
  vec2 randCoords = f_screen_size * tex_coord * f_one_over_vector_noise_size;
  return texture(f_vector_noise, randCoords).xy * 2.0f - 1.0f;
}

// Calculate ambient occlusion pixel contribution from this offset
float doAmbientOcclusion(in vec2 tcoord, in vec2 tex0, in vec3 p, 
  in vec3 cnorm) {
  vec3 diff = SampleGBufferPos(tcoord + tex0, f_depth_normal_view_stencil) - p;
  float diff_length = length(diff);
  vec3 v = diff / diff_length;
  float d = diff_length * f_ssao_scale;
  return max(0.0, dot(cnorm, v) - f_ssao_bias) * (1.0 / (1.0 + d)) * 
    f_ssao_intensity;
}

const vec2 vec_4_samples[4] = vec2[](vec2(1,0), vec2(-1,0), vec2(0,1), vec2(0,-1));
const float k = 0.70710678;

void main() {
  vec3 p, n;
  SampleGBufferPosNorm(f_texture, f_depth_normal_view_stencil, p, n);

  const int iterations = 4;
  vec2 rand = getRandom(f_texture);
  float ao = 0.0;
  float rad = abs(f_ssao_sample_radius / p.z);

  // SSAO Calculation --> Loop can be unrolled (FASTER on older hardware!)
  //for (int j = 0; j < iterations; ++j) {
  //  vec2 coord1 = reflect(vec_4_samples[j], rand) * rad;
  //  vec2 coord2 = vec2(coord1.x * k - coord1.y * k, 
	 // coord1.x * k + coord1.y * k);
  //  ao += doAmbientOcclusion(f_texture, coord1*0.25, p, n);
  //  ao += doAmbientOcclusion(f_texture, coord2*0.5, p, n);
  //  ao += doAmbientOcclusion(f_texture, coord1*0.75, p, n);
  //  ao += doAmbientOcclusion(f_texture, coord2, p, n);
  //}
  
  // Loop can be unrolled (FASTER on older hardware!)
  vec2 coord1_0 = reflect(vec_4_samples[0], rand) * rad;
  vec2 coord2_0 = vec2(coord1_0.x * k - coord1_0.y * k, 
    coord1_0.x * k + coord1_0.y * k);
  ao += doAmbientOcclusion(f_texture, coord1_0*0.25, p, n);
  ao += doAmbientOcclusion(f_texture, coord2_0*0.5, p, n);
  ao += doAmbientOcclusion(f_texture, coord1_0*0.75, p, n);
  ao += doAmbientOcclusion(f_texture, coord2_0, p, n);

  vec2 coord1_1 = reflect(vec_4_samples[1], rand) * rad;
  vec2 coord2_1 = vec2(coord1_1.x * k - coord1_1.y * k, 
    coord1_1.x * k + coord1_1.y * k);
  ao += doAmbientOcclusion(f_texture, coord1_1*0.25, p, n);
  ao += doAmbientOcclusion(f_texture, coord2_1*0.5, p, n);
  ao += doAmbientOcclusion(f_texture, coord1_1*0.75, p, n);
  ao += doAmbientOcclusion(f_texture, coord2_1, p, n);

  vec2 coord1_2 = reflect(vec_4_samples[2], rand) * rad;
  vec2 coord2_2 = vec2(coord1_2.x * k - coord1_2.y * k, 
    coord1_2.x * k + coord1_2.y * k);
  ao += doAmbientOcclusion(f_texture, coord1_2*0.25, p, n);
  ao += doAmbientOcclusion(f_texture, coord2_2*0.5, p, n);
  ao += doAmbientOcclusion(f_texture, coord1_2*0.75, p, n);
  ao += doAmbientOcclusion(f_texture, coord2_2, p, n);

  vec2 coord1_3 = reflect(vec_4_samples[3], rand) * rad;
  vec2 coord2_3 = vec2(coord1_3.x * k - coord1_3.y * k, 
    coord1_3.x * k + coord1_3.y * k);
  ao += doAmbientOcclusion(f_texture, coord1_3*0.25, p, n);
  ao += doAmbientOcclusion(f_texture, coord2_3*0.5, p, n);
  ao += doAmbientOcclusion(f_texture, coord1_3*0.75, p, n);
  ao += doAmbientOcclusion(f_texture, coord2_3, p, n);

  ao /= (iterations * 4.0);
  ambient = clamp((1.0 - ao), 0, 1);
}
