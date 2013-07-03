// *********************
// lighting_include.frag
// *********************
#include "./shaders/g_buffer/g_buffer_include.frag"

uniform vec2 f_screen_size;

const int max_sm_count = 4;

struct PointLight {
  vec3 diffuse_color;
  vec3 pos_view;
  float diffuse_intensity;
  vec2 near_far;
  float spec_intensity;	
};

struct DirLight {
  vec3 diffuse_color;
  vec3 dir_view;
  float diffuse_intensity;
  float spec_intensity;	
};

struct SpotLight {
  vec3 diffuse_color;
  vec3 pos_view;
  vec3 dir_view;
  float outer_angle_cosine;
  float inner_minus_outer_cosine;
  float diffuse_intensity;
  vec2 near_far;
  float spec_intensity;	
};

vec2 CalcTexCoord() {
  return gl_FragCoord.xy / f_screen_size;
}

float linstep(in float v_min, in float v_max, in float v) {
  return clamp((v - v_min) / (v_max - v_min), 0, 1);
}

float lerp(float a, float b, float s) {
    return a + (b - a) * s;       
}

vec2 ComputeMoments(in float depth) {
  float dx = dFdx(depth);  
  float dy = dFdy(depth);  
	
  // Compute first few moments of depth
  vec2 moments;
  moments.x = depth;
  moments.y = depth * depth  + 0.25 * (dx*dx + dy*dy);
    
  // New encoding:
  // http://www.gamedev.net/topic/485290-extending-vsm---cheap-pcss--alternate-representation/
  // Store 4*(D - D^2) --> For better linearity

  return moments;
}

// Light bleeding reduction
float LBR(in float p, in float lbr_amount) {
    return linstep(lbr_amount, 1, p);
}


// Computes Chebyshev's Inequality
// Returns an upper bound given the first two moments and mean
float ChebyshevUpperBound(in vec2 moments, in float mean, 
  in float min_variance, in float lbr_amount) {
  // Standard shadow map comparison
  // float p = (mean <= moments.x);

  float p = 0.0;
  if (mean <= moments.x) {
    p = 1.0;
  }
  
  // Compute variance
  float variance = moments.y - (moments.x * moments.x);
  variance = max(variance, min_variance);
    
  // Compute probabilistic upper bound
  float d     = mean - moments.x;
  float p_max = variance / (variance + d*d);
  
  // Light bleeding reduction:
  float p_ret = smoothstep(lbr_amount, 1.0, max(p, p_max));  
  // float p_ret = LBR(p_max, lbr_amount);  // Light bleeding reduction

  return p_ret;
  // return max(p, p_max);
}