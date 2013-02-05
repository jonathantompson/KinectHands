#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D kinect_depth;
uniform sampler2D synth_depth;

const float epsilon = 0.00001;
const int sample_rad = 5;
const int sample_rad_sq = sample_rad * sample_rad;
uniform float max_depth;

void main(){
  //float depth_integ = 0;
  //float depth_kinect = texture(kinect_depth, f_texture).x;
  //float depth_synth, depth_difference;
  //for (int v = -sample_rad; v <= sample_rad; v++) {
  //  for (int u = -sample_rad; u <= sample_rad; u++) {
  //    depth_synth = texture(synth_depth, f_texture + vec2(u, v)*texel_dim).x;
  //    depth_difference = min(abs(depth_kinect - depth_synth), max_depth);
  //    depth_difference *= depth_difference;  // D^2
  //    depth_integ += depth_difference;
  //  }
  //}
  //depth_integ /= sample_rad_sq;
  //depth_integ = depth_difference * depth_difference;

  float depth_kinect = texture(kinect_depth, f_texture).x;
  float depth_synth = texture(synth_depth, f_texture).x;
  float depth_integ = min(abs(depth_kinect - depth_synth), max_depth);
  // depth_integ = depth_integ * depth_integ;

  float d_intersection = 0;
  if (depth_kinect > epsilon && depth_synth > epsilon) {
    d_intersection = 1;
  }

  float d_union = 0; 
  if (depth_kinect > epsilon || depth_synth > epsilon) {
    d_union = 1;
  }

  color = vec4(depth_integ, d_union, d_intersection, 0.0);    
}