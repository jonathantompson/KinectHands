#version 150

in vec3 v_pos;

out vec2 f_texture;

void main(){
	gl_Position =  vec4(v_pos, 1);
	f_texture = (v_pos.xy + vec2(1,1)) * 0.5;  // 0 to 1
}

