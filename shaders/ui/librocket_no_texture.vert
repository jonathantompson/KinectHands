#version 150

uniform mat4 vw_mat;

in vec2 v_pos;
in vec4 v_col;

out vec4 fColor;

void main(void) {	
	fColor = v_col;
	gl_Position = vw_mat * vec4(v_pos, 0.0, 1.0);
}
