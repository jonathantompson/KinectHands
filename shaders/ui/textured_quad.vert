#version 150

in vec3 v_pos;

uniform mat4 pvw_mat;

out vec2 f_texture;

void main(){
	gl_Position =  pvw_mat * vec4(v_pos, 1);
	f_texture = (v_pos.xy + vec2(1,1)) * 0.5;  // TO DO: fix this to * 0.5
}

