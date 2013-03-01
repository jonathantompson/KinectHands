#version 330 core

in vec3 v_pos;
out vec3 g_pos;

void main(){
  g_pos = v_pos;  // We have to have SOMETHING otherwise NVIDIA drivers will
                  // optimize out the entire pipeline.
}

