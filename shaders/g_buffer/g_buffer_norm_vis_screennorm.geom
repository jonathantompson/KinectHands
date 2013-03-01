#version 330 core
#include "./shaders/g_buffer/g_buffer_include.frag"

#define pts_rad 3  // in u and v
#define pts_spacing 50  // pixels between visualized normals
#define n_vert_total 98  // 2 * (pts_rad * 2 + 1)^2

layout(points) in;
layout(line_strip) out;
layout(max_vertices = n_vert_total) out;

in vec3 g_pos[];

out vec3 color;

uniform sampler2D f_depth_normal_view_stencil;
uniform mat4 p_mat;
uniform vec2 pixel_size;

void main(){
  const vec3 red = vec3(1.0, 0.3, 0.3);
  const vec3 blue = vec3(0.3, 0.3, 1.0);

  // Emmit a line at each sample point:
  for (int v = -pts_rad; v <= pts_rad; v++) {
    for (int u = -pts_rad; u <= pts_rad; u++) {
	  // Get the current texture point
      vec2 tcoord = vec2(0.5, 0.5) + (pixel_size * vec2(u, v) * pts_spacing);

      // sample the screen space position and normal at the current point
      vec3 p, n;
      SampleGBufferPosNorm(tcoord, f_depth_normal_view_stencil, p, n);

      color = red;
      // this is a little lazy, we should be able to get ndc from tex position:
      gl_Position = p_mat * vec4(p, 1); 
      EmitVertex();

      color = blue;
      vec3 p2 = p + 0.25 * n;
      gl_Position = p_mat * vec4(p2, 1); 
      EmitVertex();

      EndPrimitive();
	}
  }
}