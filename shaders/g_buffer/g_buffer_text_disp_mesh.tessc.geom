#version 410 core
// define the number of CPs in the output patch
layout (vertices = 3) out;

// Control shader input (vertex shader output):
in vec2 tex_coord_cs_in[];
in vec3 normal_view_cs_in[];
in vec3 position_view_cs_in[];
in vec3 tangent_view_cs_in[];

// Evaluation shader input (control shader output):
out vec2 tex_coord_es_in[];
out vec3 normal_view_es_in[];
out vec3 position_view_es_in[];
out vec3 tangent_view_es_in[];
out vec2 tangent_scale_es_in[];

uniform mat4 p_mat;
//uniform float tc_lod_factor;
//uniform float tc_max_tess_level;
//uniform vec2 tc_screen_size;
uniform float tc_tess_factor;

//vec4 Project(vec4 vertex){
//    vec4 result = p_mat * vertex;
//    result /= result.w;
//    return result;
//}

//vec2 ScreenSpace(vec4 vertex) {
//  return (clamp(vertex.xy, -1.3, 1.3) + 1) * 0.5 * tc_screen_size;
//}

//float GetTessLevel(vec2 ssA, vec2 ssB) {
//  return clamp(distance(ssA, ssB)/tc_lod_factor, 1.0, tc_max_tess_level);
//}

//bool Offscreen(vec4 vertex){
//  if (vertex.z < -1.0) {
//    return true;
//  }   
//  return any(lessThan(vertex.xy, vec2(-1.7)) || 
//             greaterThan(vertex.xy, vec2(1.7)));
//}

void main() {
  // Set the control points of the output patch
  tex_coord_es_in[gl_InvocationID] = tex_coord_cs_in[gl_InvocationID];
  normal_view_es_in[gl_InvocationID] = normal_view_cs_in[gl_InvocationID];
  position_view_es_in[gl_InvocationID] = position_view_cs_in[gl_InvocationID];
  tangent_view_es_in[gl_InvocationID] = tangent_view_cs_in[gl_InvocationID];

  if (gl_InvocationID == 0){
    /*
    vec4 v0 = Project(vec4(position_view_cs_in[0], 1.0));
    vec4 v1 = Project(vec4(position_view_cs_in[1], 1.0));
    vec4 v2 = Project(vec4(position_view_cs_in[2], 1.0));

    if (all(bvec3(Offscreen(v0), Offscreen(v1), Offscreen(v2)))) {
      gl_TessLevelInner[0] = 0;
      gl_TessLevelOuter[0] = 0;
      gl_TessLevelOuter[1] = 0;
      gl_TessLevelOuter[2] = 0;
    } else{
      vec2 ss0 = ScreenSpace(v0);
      vec2 ss1 = ScreenSpace(v1);
      vec2 ss2 = ScreenSpace(v2);

      // For a good picture for what the tessellation levels mean, see:
      // http://ogldev.atspace.co.uk/www/tutorial30/tutorial30.html
      // The TessLevelOuter edge is between the verticies that is oposite the
      // vertex corresponding it's index.

      // gl_TessLevelOuter[0] between V1 and V2:
      gl_TessLevelOuter[0] = GetTessLevel(ss1, ss2);
      // gl_TessLevelOuter[1] between V0 and V2:
      gl_TessLevelOuter[1] = GetTessLevel(ss0, ss2);
      // gl_TessLevelOuter[2] between V0 and V1:
      gl_TessLevelOuter[2] = GetTessLevel(ss0, ss1);
      gl_TessLevelInner[0] = max(gl_TessLevelOuter[0], 
                                 max(gl_TessLevelOuter[1], 
                                     gl_TessLevelOuter[2]));
    }
	*/
    gl_TessLevelOuter[0] = tc_tess_factor;
    gl_TessLevelOuter[1] = tc_tess_factor;
    gl_TessLevelOuter[2] = tc_tess_factor;
    gl_TessLevelInner[0] = tc_tess_factor;

	// Calculate the viewspace --> UV space scale
    vec3 vec_1 = position_view_cs_in[1] - position_view_cs_in[0];
    vec3 vec_2 = position_view_cs_in[2] - position_view_cs_in[0];
    vec2 duv_1 = tex_coord_cs_in[1] - tex_coord_cs_in[0];
    vec2 duv_2 = tex_coord_cs_in[2] - tex_coord_cs_in[0];
	vec3 bitangent = normalize(cross(tangent_view_cs_in[0], 
	  normal_view_es_in[0]));
    if (duv_1.x > duv_2.x) {  
      // vec2 is potentially orthogonal to tangent --> use vec1
      float vec_dot_tan = dot(vec_1, normalize(tangent_view_cs_in[0]));
      tangent_scale_es_in[gl_InvocationID].x = abs(vec_dot_tan) / abs(duv_1.x);
    } else {
      // vec1 is potentially orthogonal to tangent --> use vec2
      float vec_dot_tan = dot(vec_2, normalize(tangent_view_cs_in[0]));
      tangent_scale_es_in[gl_InvocationID].x = abs(vec_dot_tan) / abs(duv_2.x);
    }
    if (duv_1.y > duv_2.y) {  
      // vec2 is potentially orthogonal to tangent --> use vec1
	  float vec_dot_bitan = dot(vec_1, bitangent);
      tangent_scale_es_in[gl_InvocationID].y = abs(vec_dot_bitan) / abs(duv_1.y);
    } else {
      // vec1 is potentially orthogonal to tangent --> use vec2
	  float vec_dot_bitan = dot(vec_2, bitangent);
      tangent_scale_es_in[gl_InvocationID].y = abs(vec_dot_bitan) / abs(duv_2.y);
    }
  }
}
