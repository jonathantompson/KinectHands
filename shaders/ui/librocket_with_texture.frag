#version 150

uniform sampler2D f_texture_sampler;

in vec4 f_color;
smooth in vec2 f_texture;

out vec4 out_Color0;

void main(void) {	
	vec4 tex_color = texture(f_texture_sampler, f_texture.xy);
	out_Color0 = tex_color * vec4(f_color.rgba);
}
