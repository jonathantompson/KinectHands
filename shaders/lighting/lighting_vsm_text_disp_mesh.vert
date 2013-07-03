#version 410 core

in vec3 v_pos;
in vec2 v_tex_coord;
in vec3 v_nor;
in vec3 v_tangent;
in vec3 v_bitangent;  // We could calculate this in shader, but this saves comp

uniform mat4 vw_mat;
uniform mat4 normal_mat;

// Control shader input (vertex shader output):
out vec2 f_tex_coord_cs_in;  
out vec3 f_normal_view_cs_in;
out vec3 f_position_view_cs_in;

void main(void) {
  // We don't perform pvw at this point, since we will do this after new
  // geometry is created by the TESS_EVALUATION shader
  f_normal_view_cs_in = (normal_mat * vec4(v_nor, 0.0)).xyz;
  f_tex_coord_cs_in = v_tex_coord;
  f_position_view_cs_in = (vw_mat * vec4(v_pos, 1.0)).xyz;
}
