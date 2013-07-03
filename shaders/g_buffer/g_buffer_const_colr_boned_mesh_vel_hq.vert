#version 150
#include "./shaders/g_buffer/g_buffer_include.frag"
// Note, uses the same fragment shader as g_buffer_text_mesh.vert

in vec3 v_pos;
in vec3 v_nor;
in ivec4 v_bonei;
in vec4 v_bonew;

uniform mat4 pvw_mat;
uniform mat4 vw_mat;
uniform mat4 normal_mat;
uniform mat4x4 bone_trans[MAX_BONE_COUNT];
uniform mat4 pvw_mat_prev_frame;
uniform mat4x4 bone_trans_prev_frame[MAX_BONE_COUNT];

out vec3 f_color;
out vec3 f_normal_view;
out vec3 f_position_view;
out vec4 f_gl_position;
out vec4 f_gl_position_prev;

void main(void) {
  // Linear blend skinning:
  mat4 bone_transform = bone_trans[v_bonei[0]] * v_bonew[0];
  bone_transform     += bone_trans[v_bonei[1]] * v_bonew[1];
  bone_transform     += bone_trans[v_bonei[2]] * v_bonew[2];
  bone_transform     += bone_trans[v_bonei[3]] * v_bonew[3];

  vec4 pos = bone_transform * vec4(v_pos, 1.0);
  vec3 nor = (bone_transform * vec4(v_nor, 0.0)).xyz;

  bone_transform  = bone_trans_prev_frame[v_bonei[0]] * v_bonew[0];
  bone_transform += bone_trans_prev_frame[v_bonei[1]] * v_bonew[1];
  bone_transform += bone_trans_prev_frame[v_bonei[2]] * v_bonew[2];
  bone_transform += bone_trans_prev_frame[v_bonei[3]] * v_bonew[3];

  vec4 pos_prev_frame = bone_transform * vec4(v_pos, 1.0);

  // Multiply the mvp matrix by the vertex to obtain our final vertex position
  gl_Position = pvw_mat * pos;

  f_normal_view = (normal_mat * vec4(nor, 0.0)).xyz;
  f_position_view = (vw_mat * pos).xyz;
  f_gl_position = gl_Position;
  f_gl_position_prev = (pvw_mat_prev_frame * pos_prev_frame);
}
