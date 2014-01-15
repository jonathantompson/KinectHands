#version 150

in vec3 v_pos;
in vec3 v_col;

uniform mat4 pvw_mat;
uniform mat4 vw_mat;
uniform mat4 pvw_mat_prev_frame;

out vec3 f_color;
out vec3 f_normal_view;
out vec3 f_position_view;
out vec4 f_gl_position;
out vec4 f_gl_position_prev;

void main(void) {
  // Multiply the mvp matrix by the vertex to obtain our final vertex position
  gl_Position = pvw_mat * vec4(v_pos, 1.0);

  f_position_view = (vw_mat * vec4(v_pos, 1.0)).xyz;
  f_color = v_col;
  f_normal_view = vec3(0,0,1);  // Always point towards the camera
  f_gl_position = gl_Position;
  f_gl_position_prev = (pvw_mat_prev_frame * vec4(v_pos, 1.0));
}
