#version 150
#extension GL_EXT_texture_array : enable

in vec2 f_texture;

out vec4 color;

uniform sampler2DArray f_texture_sampler;
uniform int f_radius;
uniform vec2 f_texel_size;
uniform float f_array_index;

void main(){
  color = vec4(0.0, 0.0, 0.0, 1.0);
  for (int i = -f_radius; i <= f_radius; i++) {
    vec2 cur_tex_coord = f_texture + i * f_texel_size;
    color += vec4(texture2DArray(f_texture_sampler, 
	  vec3(cur_tex_coord, f_array_index)).xyz, 1.0);
  }
  color = color / (2.0*f_radius+1.0);
}