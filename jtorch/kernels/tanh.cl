__kernel void TanH(const __global  float* input, __global  float* output) {

  const int width = get_global_size(0);
  const int height = get_global_size(1);

  const int x_out = get_global_id(0);
  const int y_out = get_global_id(1);
  const int f_out = get_global_id(2);

  const int index = x_out + width * (y_out + height * f_out);

  output[index] = tanh(input[index]);
}
