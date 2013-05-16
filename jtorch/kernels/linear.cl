// This could be much better: http://www.bealto.com/gpu-gemv_v2.html
// Although it's OK if the number of outputs is high (so we can have many
// concurrent threads).
__kernel void MatVecMultSimple(
  // Y = A * X (matrix-vector mulitply)
  __global const float* A,  // 0  --> Size M (rows) x N (cols) stored column major
  __global const float* X,  // 1  --> Size N
  __global  float* Y,       // 2  --> Size M
  const int M,              // 3
  const int N) {            // 4

  const int i = get_global_id(0);  // row index

  float sum = 0;
  // Perform the linear accumulation
  for (int k = 0; k < N; k++) {
    sum += A[i + M * k] * X[k];
  }

  Y[i] = sum;
}


__kernel void Accum (
  // output = bias
  __global  float* output,          // 0
  const __global float* biases) {   // 1

  const int x_out = get_global_id(0);

  output[x_out] += biases[x_out];
}
