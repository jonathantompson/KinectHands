#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;
uniform int f_radius;
uniform vec2 f_texel_size;

void main(){
  color = vec4(0.0, 0.0, 0.0, 0.0);
  for (int i = -f_radius; i <= f_radius; i++) {
    vec2 cur_tex_coord = f_texture + i * f_texel_size;
    color += texture(f_texture_sampler, cur_tex_coord).xyzw;
  }
  color = color / (2.0*f_radius+1.0);
}