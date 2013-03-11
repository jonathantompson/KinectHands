#version 410 core

in vec3 v_pos;
in vec2 v_tex_coord;
in vec3 v_nor;
in vec3 v_tangent;

uniform mat4 vw_mat;
uniform mat4 normal_mat;

// Control shader input (vertex shader output):
out vec2 tex_coord_cs_in;  
out vec3 normal_view_cs_in;
out vec3 position_view_cs_in;
out vec3 tangent_view_cs_in;

void main(void) {
  // We don't perform pvw at this point, since we will do this after new
  // geometry is created by the TESS_EVALUATION shader
  normal_view_cs_in = (normal_mat * vec4(v_nor, 0.0)).xyz;
  tangent_view_cs_in = (normal_mat * vec4(v_tangent, 0.0)).xyz;
  tex_coord_cs_in = v_tex_coord;
  position_view_cs_in = (vw_mat * vec4(v_pos, 1.0)).xyz;
}
