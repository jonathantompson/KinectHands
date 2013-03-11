#version 410 core

// Note: Textured Mesh AND Texture Displaced Mesh share a fragment
// shader!

layout(triangles, equal_spacing, ccw) in;

uniform mat4 p_mat;
uniform sampler2D te_disp_tex;
uniform float te_disp_factor;
uniform vec2 te_disp_texel_size;

// Evaluation shader input (control shader output):
in vec3 position_view_es_in[];
in vec2 tex_coord_es_in[];
in vec3 normal_view_es_in[];
in vec3 tangent_view_es_in[];
in vec2 tangent_scale_es_in[];

// Velocity blur only:
in vec4 f_gl_position_es_in[];
in vec4 f_gl_position_prev_es_in[];

// Fragment shader input (evaluation shader output):
out vec3 f_position_view;
out vec2 f_rgb_coord;
out vec3 f_normal_view;

// Velocity blur only:
out vec4 f_gl_position;
out vec4 f_gl_position_prev;

vec2 interpolate2D(vec2 v0, vec2 v1, vec2 v2) {
  return vec2(gl_TessCoord.x) * v0 + vec2(gl_TessCoord.y) * v1 + 
    vec2(gl_TessCoord.z) * v2;
}
vec3 interpolate3D(vec3 v0, vec3 v1, vec3 v2) {
  return vec3(gl_TessCoord.x) * v0 + vec3(gl_TessCoord.y) * v1 + 
	vec3(gl_TessCoord.z) * v2;
}
vec4 interpolate4D(vec4 v0, vec4 v1, vec4 v2) {
  return vec4(gl_TessCoord.x) * v0 + vec4(gl_TessCoord.y) * v1 + 
	vec4(gl_TessCoord.z) * v2;
}

void main() {
  // Interpolate the attributes of the output vertex using the barycentric 
  // coordinates
  f_rgb_coord = interpolate2D(tex_coord_es_in[0], tex_coord_es_in[1],
    tex_coord_es_in[2]);
  f_position_view = interpolate3D(position_view_es_in[0],
    position_view_es_in[1], position_view_es_in[2]);
  f_gl_position = interpolate4D(f_gl_position_es_in[0],
    f_gl_position_es_in[1], f_gl_position_es_in[2]);
  f_gl_position_prev = interpolate4D(f_gl_position_prev_es_in[0],
    f_gl_position_prev_es_in[1], f_gl_position_prev_es_in[2]);

  f_normal_view = normalize(interpolate3D(normal_view_es_in[0], 
    normal_view_es_in[1], normal_view_es_in[2]));

  // Displace the vertex along the normal
  float displacement = texture(te_disp_tex, f_rgb_coord.xy).x;
  f_position_view += (f_normal_view * (displacement - 0.5)* te_disp_factor);
  gl_Position = p_mat * vec4(f_position_view, 1.0);

  // Calculate derivative of height Vs. u, v using central differencing
  //     V2
  // U2       U
  //     V
  float fDeltaU = te_disp_texel_size.x;
  float fDeltaV = te_disp_texel_size.y;
  float fNeighborU =  texture(te_disp_tex, f_rgb_coord.xy + 
    vec2(fDeltaU, 0)).x * te_disp_factor;
  float fNeighborV =  texture(te_disp_tex, f_rgb_coord.xy + 
    vec2(0, fDeltaV)).x * te_disp_factor;
  float fNeighborU2 = texture(te_disp_tex, f_rgb_coord.xy - 
    vec2(fDeltaU, 0)).x * te_disp_factor;
  float fNeighborV2 = texture(te_disp_tex, f_rgb_coord.xy - 
    vec2(0, fDeltaV)).x * te_disp_factor;

  // Change in view space per texel step
  float fDistanceU = tangent_scale_es_in[0].x * fDeltaU;
  float fDistanceV = tangent_scale_es_in[0].y * fDeltaV;

  float fDhDu = (fNeighborU - fNeighborU2); 
  float fDhDv = (fNeighborV - fNeighborV2); 
  vec3 vTangentTS = normalize(vec3(fDistanceU, 0,          fDhDu/2)); 
  vec3 vBinormalTS = normalize(vec3(0,          fDistanceV, fDhDv/2)); 
  vec3 vNormalTS = cross(vTangentTS, vBinormalTS); 

  // norm is in tangent space: We need to put this into view space.
  // 1. Create tangent basis:
  vec3 tangent_view = interpolate3D(tangent_view_es_in[0], 
    tangent_view_es_in[1], tangent_view_es_in[2]);
  vec3 bitangent_view = cross(f_normal_view, tangent_view);
  
  // 2. Gram-Schmidt orthogonalization.
  // NOTE: Works only if the vectors are close to orthogonal.
  // N' = |N|
  // T' = |T - (N' dot T) N'|
  // B' = |B - (N' dot B) N' - (T' dot B) T'|
  // f_normal_view = normalize(f_normal_view, epsilon);  // Already done
  tangent_view -= f_normal_view * dot(f_normal_view, tangent_view);
  tangent_view = normalize(tangent_view);
  bitangent_view -= f_normal_view * dot(f_normal_view, bitangent_view);
  bitangent_view -= tangent_view * dot(tangent_view, bitangent_view);
  bitangent_view = normalize(bitangent_view);

  // 3. Transform tangent space normal into viewspace
  mat3 TBN = mat3(tangent_view, bitangent_view, f_normal_view);
  f_normal_view = TBN * vNormalTS;
  f_normal_view = normalize(f_normal_view);
}
