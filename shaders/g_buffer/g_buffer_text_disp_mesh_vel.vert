#version 410 core

in vec3 v_pos;
in vec2 v_tex_coord;
in vec3 v_nor;
in vec3 v_tangent;

uniform mat4 vw_mat;
uniform mat4 normal_mat;

// For velocity blur only
// Velocity is not by movement of tessellated geometry, but by movement of
// coarse control points.  I think this is OK.
uniform mat4 pvw_mat;
uniform mat4 pvw_mat_prev_frame;

// Control shader input (vertex shader output):
out vec2 tex_coord_cs_in;  
out vec3 normal_view_cs_in;
out vec3 position_view_cs_in;
out vec3 tangent_view_cs_in;

// Velocity blur only:
out vec4 f_gl_position_cs_in;
out vec4 f_gl_position_prev_cs_in;

void main(void) {
  // We don't perform pvw at this point, since we will do this after new
  // geometry is created by the TESS_EVALUATION shader
  normal_view_cs_in = (normal_mat * vec4(v_nor, 0.0)).xyz;
  tangent_view_cs_in = (normal_mat * vec4(v_tangent, 0.0)).xyz;
  tex_coord_cs_in = v_tex_coord;
  position_view_cs_in = (vw_mat * vec4(v_pos, 1.0)).xyz;

  f_gl_position_cs_in = pvw_mat * vec4(v_pos, 1.0);
  f_gl_position_prev_cs_in = (pvw_mat_prev_frame * vec4(v_pos, 1.0));
}
