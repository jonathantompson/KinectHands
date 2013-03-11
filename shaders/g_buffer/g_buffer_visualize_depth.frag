#version 150

in vec2 f_texture;

out vec4 color;

uniform sampler2D f_texture_sampler;
uniform vec2 f_camera_near_far;

void main(){
  float posz = texture(f_texture_sampler, f_texture).x;
  float normalized_depth = (posz - f_camera_near_far.x) / 
    (f_camera_near_far.y - f_camera_near_far.x);

  color = vec4(normalized_depth, normalized_depth, normalized_depth, 1.0);
}