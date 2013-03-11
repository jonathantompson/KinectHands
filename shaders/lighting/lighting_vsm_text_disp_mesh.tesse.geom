#version 410 core

// Note: Textured Mesh AND Texture Displaced Mesh share a fragment
// shader!

layout(triangles, equal_spacing, cw) in;

uniform mat4 p_mat;
uniform sampler2D te_disp_tex;
uniform float te_disp_factor;
uniform vec2 te_disp_texel_size;

// Evaluation shader input (control shader output):
in vec3 f_position_view_es_in[];
in vec2 f_tex_coord_es_in[];
in vec3 f_normal_view_es_in[];

// Fragment shader input (evaluation shader output):
#define f_position_view_geom f_position_view  // Send straight to frag
out vec3 f_position_view_geom;

vec2 interpolate2D(vec2 v0, vec2 v1, vec2 v2) {
  return vec2(gl_TessCoord.x) * v0 + vec2(gl_TessCoord.y) * v1 + 
    vec2(gl_TessCoord.z) * v2;
}

vec3 interpolate3D(vec3 v0, vec3 v1, vec3 v2) {
  return vec3(gl_TessCoord.x) * v0 + vec3(gl_TessCoord.y) * v1 + 
	vec3(gl_TessCoord.z) * v2;
}

// To get around shimmering:
// http://sebastiansylvan.wordpress.com/2010/04/18/the-problem-with-tessellation-in-directx-11/

void main() {
  // Interpolate the attributes of the output vertex using the barycentric 
  // coordinates
  f_position_view_geom = interpolate3D(f_position_view_es_in[0],
    f_position_view_es_in[1], f_position_view_es_in[2]);

  // Displace the vertex along the normal
  float displacement = texture(te_disp_tex, f_rgb_coord_geom.xy).x;
  f_position_view_geom += (f_normal_view * displacement * te_disp_factor);
  gl_Position = p_mat * vec4(f_position_view_geom, 1.0);
}
