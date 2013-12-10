#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"
// From here: 
// http://developer.download.nvidia.com/assets/gamedev/files/sdk/11/FXAA_WhitePaper.pdf

in vec2 f_texture;

out vec4 final_color;
uniform sampler2D f_src_rgb;
uniform sampler2D f_src_luma;

uniform vec2 f_texel_size;

vec3 luma = vec3(0.299, 0.587, 0.114);

void main(){

  // The parameters are hardcoded for now, but could be
  // made into uniforms to control fromt he program.
  float FXAA_SPAN_MAX = 8.0;
  float FXAA_REDUCE_MUL = 1.0/8.0;
  float FXAA_REDUCE_MIN = (1.0/128.0);

  float lumaNW = texture(f_src_luma, f_texture.xy + (vec2(-0.5, -0.5) * f_texel_size)).x;
  float lumaNE = texture(f_src_luma, f_texture.xy + (vec2(+0.5, -0.5) * f_texel_size)).x;
  float lumaSW = texture(f_src_luma, f_texture.xy + (vec2(-0.5, +0.5) * f_texel_size)).x;
  float lumaSE = texture(f_src_luma, f_texture.xy + (vec2(+0.5, +0.5) * f_texel_size)).x;
  float lumaM  = texture(f_src_luma, f_texture.xy).x;
	
  float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
  float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));
	
  vec2 dir;
  dir.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
  dir.y =  ((lumaNW + lumaSW) - (lumaNE + lumaSE));
	
  float dirReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * (0.25 * FXAA_REDUCE_MUL), FXAA_REDUCE_MIN);
	  
  float rcpDirMin = 1.0/(min(abs(dir.x), abs(dir.y)) + dirReduce);
	
  dir = min(vec2(FXAA_SPAN_MAX,  FXAA_SPAN_MAX), 
        max(vec2(-FXAA_SPAN_MAX, -FXAA_SPAN_MAX), dir * rcpDirMin)) * f_texel_size;
		
  vec3 rgbA = (1.0/2.0) * (
              texture(f_src_rgb, f_texture.xy + dir * (1.0/3.0 - 0.5)).xyz +
              texture(f_src_rgb, f_texture.xy + dir * (2.0/3.0 - 0.5)).xyz);
  vec3 rgbB = rgbA * (1.0/2.0) + (1.0/4.0) * (
              texture(f_src_rgb, f_texture.xy + dir * (0.0/3.0 - 0.5)).xyz +
              texture(f_src_rgb, f_texture.xy + dir * (3.0/3.0 - 0.5)).xyz);
  float lumaB = dot(rgbB, luma);

  if((lumaB < lumaMin) || (lumaB > lumaMax)){
    final_color.xyz=rgbA;
  } else {
    final_color.xyz=rgbB;
  }
  final_color.a = 1.0;
}
