__kernel void Linear(
  const __global  float* input,  // 0
  __global  float* output,       // 1 
  __constant float* weights,     // 2
  __constant float* biases,      // 3
  const int input_size) {        // 4

  const int x_out = get_global_id(0);

  // Initilize the output to the bias
  float sum = biases[x_out];

  // Get a pointer to the current weight vector
  __constant float* pweights = &weights[x_out * input_size];

  // Perform the linear accumulation
  for (int i = 0; i < input_size; i++) {
    sum += input[i] * pweights[i];
  }

  output[x_out] = sum;
}
