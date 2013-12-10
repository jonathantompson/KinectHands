#version 150
#extension GL_EXT_gpu_shader4 : require

in vec2 f_texture;

const vec3 split_colors[9] = vec3[](vec3(1.0, 0.0, 0.0), 
                                    vec3(0.0, 1.0, 0.0), 
								    vec3(0.0, 0.0, 1.0), 
								    vec3(1.0, 1.0, 0.0), 
								    vec3(1.0, 0.0, 1.0), 
								    vec3(0.0, 1.0, 1.0), 
								    vec3(1.0, 1.0, 1.0), 
								    vec3(0.5, 1.0, 0.0), 
								    vec3(0.0, 0.5, 1.0));
uniform isampler2D f_vsm_splits;
uniform float f_vsm_split_alpha;

out vec4 color;

void main(){
  int split = texture(f_vsm_splits, f_texture).x;
  color = vec4(split_colors[split], f_vsm_split_alpha);
}
