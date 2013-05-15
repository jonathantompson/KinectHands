__kernel void SpatialConvolutionMap(const __global  float* input,
  __global  float* pOutput, __constant float* weights, __constant float* biases,
  const int input_w, const int input_h, const int filt_width) {
	/*
	// Work on CPU
	if (get_global_id(0) == 0 && get_global_id(1) == 0) {
		printf("hello");
	}
	*/

    const int nWidth = get_global_size(0);

    const int xOut = get_global_id(0);
    const int yOut = get_global_id(1);

    const int xInTopLeft = xOut;
    const int yInTopLeft = yOut;

    float sum = 0;
    for (int r = 0; r < nFilterWidth; r++)
    {
        const int idxFtmp = r * nFilterWidth;

        const int yIn = yInTopLeft + r;
        const int idxIntmp = yIn * nInWidth + xInTopLeft;

        for (int c = 0; c < nFilterWidth; c++)
        {
            const int idxF  = idxFtmp  + c;
            const int idxIn = idxIntmp + c;
            sum += pFilter[idxF]*pInput[idxIn];
        }
    }
    const int idxOut = yOut * nWidth + xOut;
    pOutput[idxOut] = sum;
}
