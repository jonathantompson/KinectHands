#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"

in vec3 v_pos;
in vec2 v_tex_coord;
in vec3 v_nor;
in ivec4 v_bonei;
in vec4 v_bonew;

uniform mat4 pvw_mat;
uniform mat4 vw_mat;
uniform mat4x4 bone_trans[MAX_BONE_COUNT];

out vec3 f_position_view;

void main(void) {
  // Linear blend skinning:
  mat4 bone_transform = bone_trans[v_bonei[0]] * v_bonew[0];
  bone_transform     += bone_trans[v_bonei[1]] * v_bonew[1];
  bone_transform     += bone_trans[v_bonei[2]] * v_bonew[2];
  bone_transform     += bone_trans[v_bonei[3]] * v_bonew[3];

  vec4 pos = bone_transform * vec4(v_pos, 1.0);
  vec3 nor = (bone_transform * vec4(v_nor, 0.0)).xyz;

  // Multiply the mvp matrix by the vertex to obtain our final vertex position
  gl_Position = pvw_mat * pos;

  f_position_view = (vw_mat * pos).xyz;
}
