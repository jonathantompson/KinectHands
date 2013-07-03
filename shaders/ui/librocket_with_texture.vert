#version 150

uniform mat4 vw_mat;

in vec2 v_pos;
in vec4 v_col;
in vec2	v_tex_coord;

out vec4 f_color;
smooth out vec2 f_texture;

void main(void) {	
	f_color = v_col;
	f_texture = v_tex_coord;
	gl_Position = vw_mat * vec4(v_pos, 0.0, 1.0);
}
