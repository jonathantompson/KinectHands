#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec2 f_texture;

out vec4 final_color;
uniform sampler2D f_src_color;
uniform sampler2D f_depth_normal_view_stencil;
uniform sampler2D f_spec_power_vel;
uniform int f_max_samples;
uniform float f_time_scale;
uniform vec2 f_texel_size;
uniform float f_depth_cutoff;

void main(){
  // SampleGBufferVel
  // Sample the per-pixel screen-space velocity
  vec2 vel = SampleGBufferVel(f_texture, f_spec_power_vel);
  float depth = SampleGBufferPosZ(f_texture, f_depth_normal_view_stencil);
  // vel.y = -vel.y; // Flip y since there is an inversion from screen to texture coords.

  // Clamp to a max velocity.  
  vel *= f_time_scale;

  float speed = length(vel / f_texel_size);
  int n_samples = clamp(int(speed), 1, f_max_samples);
  float mult_total = 0;

  vec3 oResult = vec3(0, 0, 0);
  for (int i = 1; i <= n_samples; ++i) {
	vec2 offset = vel * (float(i) / float(n_samples) - 0.5);
	float cur_depth = SampleGBufferPosZ(f_texture + offset, 
	  f_depth_normal_view_stencil);
	float mult = clamp(1.0-(cur_depth - depth) / f_depth_cutoff, 0.01, 1.0);
	oResult += mult * texture(f_src_color, f_texture + offset).xyz;
	mult_total += mult;
  }
  final_color = vec4(oResult.xyz / mult_total, 1.0);
}