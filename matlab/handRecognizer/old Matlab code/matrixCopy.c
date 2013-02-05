#include "mex.h"

void matrixCopy(double ret[], double A[], int width, int height) {
  int x;
  int y;
  // Matlab stores matrices column major!
  for (x = 0; x < width; x++) {  
    for (y = 0; y < height; y++) {
      int index = y + x*height;
      ret[index] = A[index];
      printf("ret[%d] = %f\n", index, A[index]);
    }
  }
}

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  double *x,*y;
  size_t mrows,ncols;
  int width;
  int height;
  
  /* Check for proper number of arguments. */
  if(nrhs!=1) {
    mexErrMsgIdAndTxt( "MATLAB:timestwo:invalidNumInputs",
            "One input required.");
  } 
  if(nlhs!=1) {
    mexErrMsgIdAndTxt( "MATLAB:timestwo:invalidNumOutput",
            "One output required.");
  }
  
  /* The input must be a noncomplex scalar double.*/
  mrows = mxGetM(prhs[0]);
  ncols = mxGetN(prhs[0]);

  width = (int)ncols;
  height = (int)mrows;
  if( !mxIsDouble(prhs[0]) || mxIsComplex(prhs[0])) {
    mexErrMsgIdAndTxt( "MATLAB:timestwo:inputNotRealScalarDouble",
            "Input must be a noncomplex double.");
  }
  
  /* Create matrix for the return argument. */
  plhs[0] = mxCreateDoubleMatrix((mwSize)mrows, (mwSize)ncols, mxREAL);
  
  /* Assign pointers to each input and output. */
  x = mxGetPr(prhs[0]);
  y = mxGetPr(plhs[0]);
  
  /* Call the timestwo subroutine. */
  matrixCopy(y, x, width, height);
}
