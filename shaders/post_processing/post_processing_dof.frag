#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec2 f_texture;
in vec4 f_dof_bounds;

out vec4 final_color;

const int NUM_DOF_TAPS = 12;

uniform sampler2D f_depth_normal_view_stencil;
uniform sampler2D f_final_scene;
uniform vec2 f_disk_offsets[NUM_DOF_TAPS];

const float	MAX_COC = 10.0f;

float linstep(float v_min, float v_max, float v) {
  return clamp((v - v_min) / (v_max - v_min), 0, 1);
}

float GetBlurFactor(in float depth) {
  float blur_factor;
  if(depth < f_dof_bounds[1]) {
    blur_factor = 1.0f - linstep(f_dof_bounds[0], f_dof_bounds[1], depth);
  } else {
    blur_factor = linstep(f_dof_bounds[2], f_dof_bounds[3], depth);
  }
  return blur_factor;
}

void main(){
  // Start with center sample color
  vec4 color_sum = texture(f_final_scene, f_texture);
  float total_contribution = 1.0f;

  // Depth and blurriness values for center sample
  float center_depth = -SampleGBufferPosZ(f_texture, f_depth_normal_view_stencil);
  float center_blur = GetBlurFactor(center_depth);

  if (center_blur > 0.00001f) { // epsilon for roundoff errors
    // Compute CoC size based on blurriness
	float size_coc = center_blur * MAX_COC;

    // Run through all filter taps
	for (int i = 0; i < NUM_DOF_TAPS; i++) {
	  // Compute sample coordinates
	  vec2 tap_coord = f_texture + f_disk_offsets[i] * size_coc;

      // Fetch filter tap sample
      vec4 tap_color =  texture(f_final_scene, tap_coord);
      float tap_depth = -SampleGBufferPosZ(tap_coord, f_depth_normal_view_stencil);
      float tap_blur = GetBlurFactor(tap_depth);

      // Compute tap contribution based on depth and blurriness
      float tap_contribution = (tap_depth > center_depth) ? 1.0f : tap_blur;

      // Accumulate color and sample contribution
	  color_sum += tap_color * tap_contribution;
	  total_contribution += tap_contribution;
	}
  }

  // Normalize color sum
  final_color = color_sum / total_contribution;
}