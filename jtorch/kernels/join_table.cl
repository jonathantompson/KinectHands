__kernel void JoinTable(
  const __global  float* input,  // 0
  __global  float* output,       // 1 
  const int output_offset) {     // 2

  const int x_in = get_global_id(0);
  const int y_in = get_global_id(1);
  const int z_in = get_global_id(2);

  const int width = get_global_size(0);
  const int height = get_global_size(1);

  // Calculate index into input array
  const int i_in = x_in + width * (y_in + height * z_in);

  output[i_in + output_offset] = input[i_in];
}
