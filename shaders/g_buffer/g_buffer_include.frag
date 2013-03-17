// *********************
// g_buffer_include.frag
// *********************

const int MAX_BONE_COUNT = 64;
uniform vec2 f_inv_focal_length;

// Calculate f_inv_focal_length using:
// const float fovy = fov_y_deg * 3.14159265 / 180.0;
// const float f_inv_focal_length.x = tan(fovy * 0.5) * width / height;
// const float f_inv_focal_length.y = tan(fovy * 0.5);

// GBuffer normal encoding:
// Some references:
// http://aras-p.info/texts/CompactNormalStorage.html
// http://www.slideshare.net/guest11b095/a-bit-more-deferred-cry-engine3 pg 13
// http://developer.amd.com/gpu_assets/01GDC09AD3DDStalkerClearSky210309.ppt
// On top of these, I've chosen to encode the sign of z in the sign of x.  
// Therefore, encoding is completely lossless and uses only 2 floats!

vec2 NormalToTexture(vec3 norm) {
  norm = normalize(norm);
  vec2 return_val = 0.5 * (norm.xy + vec2(1.0, 1.0));
  return_val.x *= (norm.z < 0.0 ? -1.0 : 1.0);
  return return_val;
}

vec3 TextureToNormal(vec2 enc) {
  vec3 norm;
  norm.xy = (2.0 * abs(enc)) - vec2(1.0, 1.0);
  norm.z = (enc.x < 0.0 ? -1.0 : 1.0) * 
           sqrt(abs(1.0 - (norm.x * norm.x) - (norm.y * norm.y)));
  return norm;
}

vec3 uv_to_eye(vec2 uv, float eye_z) {
   uv = (uv * vec2(-2.0, -2.0) - vec2(-1.0, -1.0));
   return vec3(uv * f_inv_focal_length * eye_z, eye_z);
}

// Reconstruction of view position from depth
// http://www.horde3d.org/forums/viewtopic.php?f=1&t=569
vec3 GetPositionFromDepth(in vec2 tex_coord, in float posz) {
  return uv_to_eye(tex_coord, posz);
}

// StoreGBuffer - Correctly format the GBuffer data into the render targets
// Specular power stored as log2(X)/10.5 (as described in 
// http://www.scribd.com/doc/6522844/Deferred-Rendering-in-Kill-Zone)
void StoreGBuffer(out vec4 depth_normal_view_stencil, 
                  out vec4 albedo_spec_intensity,
                  out vec4 spec_power_vel,
                  in vec3 position_view, 
                  in vec3 norm, 
                  in vec3 diffuse, 
                  in float spec_intensity, 
                  in float spec_power, 
				  in float lighting_stencil,
				  in vec2 velocity) {
  vec2 norm2D = NormalToTexture(norm.xyz);
  depth_normal_view_stencil = vec4(position_view.z, norm2D.x, norm2D.y, 
    lighting_stencil);
  albedo_spec_intensity = vec4(diffuse, spec_intensity);
  spec_power_vel = vec4(log2(max(spec_power, 0.00001))/10.5, 
                        velocity.x, velocity.y, 0.0);
}

// SampleGBuffer just position, normal and stencil
void SampleGBufferPosNormStencil(in vec2 tex_coord,
                                 in sampler2D depth_normal_view_stencil, 
                                 out vec3 position_view, 
                                 out vec3 norm,
								 out float lighting_stencil) {
  // Sample textures
  vec4 val_depth_normal_view = texture(depth_normal_view_stencil, tex_coord);

  // Depth --> View space position
  float posz = val_depth_normal_view.x;
  position_view = GetPositionFromDepth(tex_coord, posz);

  // Normal
  vec2 norm2D = val_depth_normal_view.yz;
  norm = TextureToNormal(norm2D);

  lighting_stencil = val_depth_normal_view.w;
}

// SampleGBuffer just stencil
float SampleGBufferStencil(in vec2 tex_coord,
                          in sampler2D depth_normal_view_stencil) {
  // Sample textures
  return texture(depth_normal_view_stencil, tex_coord).w;
}

// SampleGBuffer just position, normal
void SampleGBufferPosNorm(in vec2 tex_coord,
                          in sampler2D depth_normal_view_stencil, 
                          out vec3 position_view, 
                          out vec3 norm) {
  // Sample textures
  vec3 val_depth_normal_view = texture(depth_normal_view_stencil, tex_coord).xyz;

  // Depth --> View space position
  float posz = val_depth_normal_view.x;
  position_view = GetPositionFromDepth(tex_coord, posz);

  // Normal
  vec2 norm2D = val_depth_normal_view.yz;
  norm = TextureToNormal(norm2D);
}

// SampleGBuffer just position
vec3 SampleGBufferPos(in vec2 tex_coord,
                      in sampler2D depth_normal_view_stencil) {
  // Depth --> View space position
  float posz = texture(depth_normal_view_stencil, tex_coord).x;
  return GetPositionFromDepth(tex_coord, posz);
}

vec3 SampleGBufferNor(in vec2 tex_coord,
                      in sampler2D depth_normal_view_stencil) {
  return TextureToNormal(texture(depth_normal_view_stencil, tex_coord).yz);
}

vec2 SampleGBufferVel(in vec2 tex_coord,
                      in sampler2D spec_power_vel) {
  return texture(spec_power_vel, tex_coord).yz;
}

float SampleGBufferPosZ(in vec2 tex_coord,
                       in sampler2D depth_normal_view_stencil) {
  // Depth
  return texture(depth_normal_view_stencil, tex_coord).x;
}

void SampleGBufferPoszNor(in vec2 tex_coord,
                          in sampler2D depth_normal_view_stencil,
						  out float posz,
						  out vec3 nor) {
  // Sample textures
  vec3 val_depth_normal_view = texture(depth_normal_view_stencil, tex_coord).xyz;

  posz = val_depth_normal_view.x;
  nor = TextureToNormal(val_depth_normal_view.yz);
}

// SampleGBuffer
void SampleGBufferAlbedoSpecular(in vec2 tex_coord,
                                 in sampler2D albedo_spec_intensity,
                                 in sampler2D spec_power_vel,
                                 out vec3 albedo, 
                                 out float spec_intensity, 
                                 out float spec_power) {
  // Sample textures
  vec4 val_albedo_spec_intensity = texture(albedo_spec_intensity, tex_coord);
  vec4 val_spec_power_vel = texture(spec_power_vel, tex_coord);
  
  // Albedo
  albedo = val_albedo_spec_intensity.xyz;

  // Specular intensity and power
  spec_intensity = val_albedo_spec_intensity.w;
  spec_power = pow(2, val_spec_power_vel.x * 10.5);
}

// SampleGBuffer
vec3 SampleGBufferAlbedo(in vec2 tex_coord,
                         in sampler2D albedo_spec_intensity) {
  // Sample textures
  vec4 val_albedo_spec_intensity = texture(albedo_spec_intensity, tex_coord);
 
  // Albedo
  return val_albedo_spec_intensity.xyz;
}

// SampleGBuffer
void SampleGBuffer(in vec2 tex_coord,
                   in sampler2D depth_normal_view_stencil, 
                   in sampler2D albedo_spec_intensity,
                   in sampler2D spec_power_vel,
                   out vec3 position_view, 
                   out vec3 norm, 
                   out vec3 albedo, 
                   out float spec_intensity, 
                   out float spec_power, 
				   out float lighting_stencil) {
  SampleGBufferPosNormStencil(tex_coord, depth_normal_view_stencil, 
    position_view, norm, lighting_stencil);
  SampleGBufferAlbedoSpecular(tex_coord, albedo_spec_intensity, 
    spec_power_vel, albedo, spec_intensity, spec_power);
}

