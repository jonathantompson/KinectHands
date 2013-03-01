#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;

void main(){
  float lighting_stencil = texture(f_texture_sampler, f_texture).w;
  color = vec4(lighting_stencil, lighting_stencil, lighting_stencil, 1.0);
}
