/*==========================================================
 * evaluateDecisionTree.c
 *
 * Inputs: RHS[0] - Tree Coefficients: WL[u,1] - int32
 *         RHS[1] - Tree Coefficients: WL[v,1] - int32
 *         RHS[2] - Tree Coefficients: WL[t,1] - single (float)
 *         RHS[3] - Tree histograms: hist(2^h-1, 2) - int32
 *         RHS[4] - image_data(width*height, n) - uint16
 *         RHS[5] - [Image Width, Image Height] - int32
 *
 * Outputs: LHS[0] - label_data(width*height, n) - uint16
 *
 *========================================================*/

// Matlab types --> C types conversion
// http://www.mathworks.com.au/help/techdoc/apiref/mxclassid.html

#include "mex.h"
#include <math.h>
#include <string.h>  // memset
#include <time.h>  // time

#define EPSILON 0.00000001f
// #define VERBOSE
// #define GO_LEFT_OUTSIDE  // Step down the tree to the left when offset UV is outside the image
#define printf mexPrintf
#define printf_flush mexEvalString("drawnow;")

// Microsoft doesn't include log2 in math.h (not part of C90 standard, but
// part of C99 standard).
#if defined(_WIN32) || defined(WIN32)
float log2(float val) {
  return ((float)log(val) / (float)log(2));
}
#endif

int calcTreeSize(int height) { return 1<<height; }
#if defined(_WIN32) || defined(WIN32)
int calcTreeHeight(int size) { return (int)log2((float)size); }
#else
int calcTreeHeight(int size) { return log2(size); }
#endif

int getLChild(int i) {
  return 2*i + 1;
}

int getRChild(int i) {
  return 2*i + 2;
}

int getParent(int i) {
  return (i-1) / 2;
}

// The computational routine
void evaluateDecisionTree(short* label_data,         // output
                          int width,                 // input
                          int height,                // input
                          int* dt_coeffs0,           // input
                          int* dt_coeffs1,           // input
                          float* dt_coeffs2,         // input
                          int* dt_hist,            // input
                          int tree_size,             // input
                          short* image_data) {       // input
  // For no good reason the Microsoft compiler requires ALL C variable
  // declarations to happen at the START of the function.  Very annoying...
  int tree_height;
  int u;
  int v;
  bool isLeaf;
  int cur_node, cur_height;
  int index, index_offset;
  float relative_depth;
  int u_offset;
  int v_offset;
  
  tree_height = calcTreeHeight(tree_size);
  
#ifdef VERBOSE
  printf("\evaluateDecisionTree INPUTS:\n");
  printf("   tree_height = %d, tree_size = %d\n", tree_height, tree_size);
  printf("   width = %d\n", width);
  printf("   height = %d\n", height);
  printf_flush;
#endif
  
  for (v = 0; v < height; v++) {
    for (u = 0; u < width; u++) {
      index = v*width + u;
      cur_node = 0;
      cur_height = 1;
      while (true) {       
        isLeaf = (cur_height == tree_height) || 
                 dt_hist[getLChild(cur_node)*2] == -1;
        if (isLeaf) {
          if (dt_hist[cur_node*2] > dt_hist[cur_node*2+1]) {
            label_data[index] = 0;
          } else {
            label_data[index] = 1;
          }
          break;
        }
        
        // Otherwise evaluate the current weak learner
        u_offset = u + dt_coeffs1[cur_node];
        v_offset = v + dt_coeffs0[cur_node];
        
        if (u_offset >= 0 && u_offset < width && v_offset >= 0 && v_offset < height) {
          index_offset = v_offset * width + u_offset;
          relative_depth = (float)(image_data[index_offset] - image_data[index]) / ((float)image_data[index]);
          if (relative_depth >= dt_coeffs2[cur_node]) {
            // Go left
            cur_node = getLChild(cur_node);
          } else {
            // Go right
            cur_node = getRChild(cur_node);
          }
        } else {
          #ifdef GO_LEFT_OUTSIDE
          // Go left --> The offset pixel is off the image
          cur_node = getLChild(cur_node);
          #else
          // Go right --> The offset pixel is off the image
          cur_node = getRChild(cur_node);
          #endif
        }
        cur_height++;
      }
    }
  }
  
}

// The gateway function
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]) {
  // For no good reason the Microsoft compiler requires ALL C variable
  // declarations to happen at the START of the function.  Very annoying...
  size_t mrows, ncols;
  int tree_size;
  int width;
  int height;
  mwSize label_output_size[2];
  
  // Check for proper number of arguments.
  if (nrhs != 6) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidNumInputs",
            "6 inputs required.");
  }
  if (nlhs != 1) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidNumOutputs",
            "1 outputs required.");
  }
  
  // Check the types of the inputs
  if (!mxIsClass(prhs[0], "int32") || mxIsComplex(prhs[0])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[0] is not real int32");
  }
  if (!mxIsClass(prhs[1], "int32") || mxIsComplex(prhs[0])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[1] is not real int32");
  }  
  if (!mxIsClass(prhs[2], "single") || mxIsComplex(prhs[0])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[2] is not real single");
  }  
  if (!mxIsClass(prhs[3], "int32") || mxIsComplex(prhs[0])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[3] is not real int32");
  }  
  if (!mxIsClass(prhs[4], "uint16") || mxIsComplex(prhs[0])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[4] is not real uint16");
  }    
  if (!mxIsClass(prhs[5], "int32") || mxIsComplex(prhs[5])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[5] is not real int32");
  }
  
  // Check the size of the inputs
  // TO DO: Do this
  
  // Allocate the LHS values:
  mrows = mxGetM(prhs[0]);
  tree_size = (int)mrows;
  width = ((int*)mxGetData(prhs[5]))[0];
  height = ((int*)mxGetData(prhs[5]))[1];
  label_output_size[0] = (int)mxGetM(prhs[4]);
  label_output_size[1] = (int)mxGetN(prhs[4]);
  plhs[0] = mxCreateNumericArray((mwSize)2,  // mwSize ndim
          (mwSize*)label_output_size,  // const mwSize *dims
          mxUINT16_CLASS,  // mxClassID classid
          mxREAL);  // mxComplexity ComplexFlag
  
  evaluateDecisionTree((short*)mxGetData(plhs[0]), width, height,
    (int*)mxGetData(prhs[0]), (int*)mxGetData(prhs[1]),
    (float*)mxGetData(prhs[2]), (int*)mxGetData(prhs[3]),
    tree_size, (short*)mxGetData(prhs[4]));
}
