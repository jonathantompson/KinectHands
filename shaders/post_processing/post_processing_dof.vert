#version 150

in vec3 v_pos;

uniform vec4 v_dof_bounds;

uniform sampler2D v_depth_normal_view;
uniform float v_camera_far;
uniform float v_camera_mid;

out vec2 f_texture;
out vec4 f_dof_bounds;

void main(){
	gl_Position =  vec4(v_pos, 1);
	f_texture = (v_pos.xy + vec2(1,1)) * 0.5;
	
	// Perform some calculations of values that are constant for all pixels
	// Doing it here saves time in the pixel shader (only 6 texture lookups
	// --> more or less negletable).
    float pos_z = -texture(v_depth_normal_view, vec2(0.5f, 0.5f)).x;
	float f_focal_distance = pos_z;
	if (f_focal_distance > -v_camera_far) {
	  f_focal_distance = -v_camera_mid;
	}
	f_dof_bounds = f_focal_distance * v_dof_bounds;
}

