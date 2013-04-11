#version 150
#extension GL_EXT_texture_array : enable
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec2 f_texture;

out vec4 color;

uniform sampler2DArray f_vsm_array;
uniform int sm_index;

void main(){
  
  vec2 moments = texture2DArray(f_vsm_array, 
    vec3(f_texture, sm_index)).xy;

  color = vec4(moments.x, moments.x, moments.x, 1.0);
}