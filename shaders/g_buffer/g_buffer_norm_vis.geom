#version 330 core

layout(triangles) in;
layout(line_strip) out;
layout(max_vertices = 6) out;

in vec3 f_normal_view[];
in vec3 f_position_view[];

out vec3 color;

uniform sampler2D f_depth_normal_view_stencil;
uniform mat4 p_mat;
uniform vec2 pixel_size;
uniform float normal_length;

void main(){
  const vec3 red = vec3(1.0, 0.25, 0.25);
  const vec3 blue = vec3(0.25, 0.25, 1.0);

  for (int i = 0; i < 3; i++) {
    color = red;
    gl_Position = p_mat * vec4(f_position_view[i], 1.0);
    EmitVertex();
    color = blue;
    vec3 p2 = f_position_view[i] + normal_length * normalize(f_normal_view[i]);
    gl_Position = p_mat * vec4(p2, 1.0); 
    EmitVertex();
    EndPrimitive();
  }
}