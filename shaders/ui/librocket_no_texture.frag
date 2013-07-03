#version 150

in vec4 fColor;
out vec4 out_Color0;
void main(void) {	
	out_Color0 = vec4(fColor.rgba);
}
