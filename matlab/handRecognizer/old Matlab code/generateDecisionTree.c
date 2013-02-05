/*==========================================================
 * generateDecisionTree.c
 *
 * A decision tree implmentation for use with Kinect data:
 * The class of weak learners are specified by [u,v] offsets and a
 * threshold, and are a function of the relative pixel depth between pairs
 * of pixels.
 *
 * Only 2 label types are supported for now, a zero and a one.
 *
 * Inputs: RHS[0] - Training data: image_data(width*height, num_images) - uint16
 *         RHS[1] - Training data: label_data(width*height, num_images) - uint16
 *         RHS[2] - Weak Learner Coefficients: WL[u,1] - u offset - int32
 *         RHS[3] - Weak Learner Coefficients: WL[v,1] - v offset - int32
 *         RHS[4] - Weak Learner Coefficients: WL[t,1] - thresholds - single (float)
 *         RHS[5] - Num Weak Lerner Samples per Node - int32
 *         RHS[6] - tree height - int32
 *         RHS[7] - [Image Width, Image Height] - int32
 *         RHS[8] - random seed - int32
 *
 * Outputs: LHS[0] - Tree Coefficients: WL[u,1] - int32
 *          LHS[1] - Tree Coefficients: WL[v,1] - int32
 *          LHS[2] - Tree Coefficients: WL[t,1] - single (float)
 *          LHS[3] - Tree histograms: hist(2^h-1, 2) - int32
 *
 *========================================================*/

// Matlab types --> C types conversion
// http://www.mathworks.com.au/help/techdoc/apiref/mxclassid.html

#include "mex.h"
#include <math.h>
#include <string.h>  // memset
#include <time.h>  // time

#define EPSILON 0.00000001f

#define VERBOSE  // Print perbose messages during tree generation
// #define GO_LEFT_OUTSIDE  // Step down the tree to the left when offset UV is outside the image

#define printf mexPrintf
#define printf_flush mexEvalString("drawnow;")

int calcTreeSize(int height) { return 1<<height; }

int permuteWLCoeffs(int** wlset_coeffs0,   // output
        int** wlset_coeffs1,   // output
        float** wlset_coeffs2, // output
        int* wl_coeffs_sizes,  // input
        int* wl_coeffs0,       // input
        int* wl_coeffs1,       // input
        float* wl_coeffs2) {   // input
  // For no good reason the Microsoft compiler requires ALL C variable
  // declarations to happen at the START of the function.  Very annoying...
  int num_wl_permutations = 1;
  int i;
  int cur_indices[3];
  int cur_perm = 0;
  
  for (i = 0; i < 3; i++) {
    num_wl_permutations *= wl_coeffs_sizes[i];
  }
  
  *wlset_coeffs0 = (int*)malloc(sizeof(int) * num_wl_permutations);
  *wlset_coeffs1 = (int*)malloc(sizeof(int) * num_wl_permutations);
  *wlset_coeffs2 = (float*)malloc(sizeof(float) * num_wl_permutations);
  
  cur_indices[0] = 0;
  cur_indices[1] = 0;
  cur_indices[2] = 0;
  
  while (cur_perm < num_wl_permutations) {
    (*wlset_coeffs0)[cur_perm] = wl_coeffs0[cur_indices[0]];
    (*wlset_coeffs1)[cur_perm] = wl_coeffs1[cur_indices[1]];
    (*wlset_coeffs2)[cur_perm] = wl_coeffs2[cur_indices[2]];
    
    cur_indices[0]++;
    for (i = 0; i < 3; i++) {
      if (cur_indices[i] >= wl_coeffs_sizes[i]) {
        cur_indices[i] = 0;
        if (i < 2) {
          cur_indices[i + 1]++;
        }
      } else {
        break;
      }
    }
    cur_perm++;
  }
  
  return num_wl_permutations;
}

// Microsoft doesn't include log2 in math.h (not part of C90 standard, but
// part of C99 standard).
#if defined(_WIN32) || defined(WIN32)
float log2(float val) {
  return ((float)log(val) / (float)log(2));
}
#endif

float calcEntropy(float* prob) {
  return -(prob[0]*log2(prob[0]+EPSILON) + prob[1]*log2(prob[1]+EPSILON));
}

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
void generateDecisionTree(int* dt_coeffs0,           // output
                          int* dt_coeffs1,           // output
                          float* dt_coeffs2,         // output
                          int* dt_hist,              // output
                          int num_images,            // input
                          short* image_data,         // input
                          short* label_data,         // input
                          int* wl_coeffs0,           // input
                          int* wl_coeffs1,           // input
                          float* wl_coeffs2,         // input
                          int* wl_coeffs_sizes,      // input
                          int num_samples_per_node,  // input
                          int tree_height,           // input
                          int width,                 // input
                          int height,                // input
                          int random_seed) {         // input
  // For no good reason the Microsoft compiler requires ALL C variable
  // declarations to happen at the START of the function.  Very annoying...
  int tree_size;
  int stack_size;
  int* stack;
  int* stack_heights;
  int* stack_occupancy_lengths;
  int cur_stack_ptr;
  int occupancy_length;
  int* occupancy;
  int* occupancy_left;
  int* occupancy_right;
  int u;
  int v;
  int index;
  int i;
  int* wlset_coeffs0;  // u offsets
  int* wlset_coeffs1;  // v offsets
  float* wlset_coeffs2;  // thresholds
  int num_wl_permutations;
  float* entropy;
  int hist_root[2] = {0, 0};
  float p_root[2];
  float sum_hist;
  int num_nodes_finished = 0;
  int cur_node, cur_height;
  float max_gain, cur_gain;
  int index_best_wl, index_cur_wl;
  int num_attempts;
  int cur_u_offset, cur_v_offset;
  int u_offset, v_offset;
  int index_offset;
  float cur_threshold;
  int hist_left[2] = {0, 0};
  int hist_right[2] = {0, 0};
  int hist_left_best[2] = {0, 0};
  int hist_right_best[2] = {0, 0};
  float p_left[2] = {0, 0};
  float p_right[2] = {0, 0};
  int* cur_occupancy_im;
  int* cur_occupancy_im_left;
  int* cur_occupancy_im_right;
  short* cur_image_data;
  short* cur_label_data;
  float relative_depth;
  float entropy_parent, entropy_left, entropy_right;
  float entropy_left_best, entropy_right_best;
  float sum_hist_left, sum_hist_right;
  int l_child_node, r_child_node;
  int repeat_attempts;
  int temp;
  const int max_repeat_attempts = 3;
  
  srand(random_seed);
  tree_size = calcTreeSize(tree_height);
  
  #ifdef VERBOSE
  printf("\ngenerateDecisionTree INPUTS:\n");
  printf("   num_images = %d\n", num_images);
  printf("   wl_coeffs_sizes = [%d, %d, %d]\n", wl_coeffs_sizes[0], wl_coeffs_sizes[1], wl_coeffs_sizes[2]);
  printf("   num_samples_per_node = %d\n", num_samples_per_node);
  printf("   tree_height = %d, tree_size = %d\n", tree_height, tree_size);
  printf("   width = %d\n", width);
  printf("   height = %d\n", height);
  printf("   seed = %d\n", random_seed);
  printf_flush;
  #endif
          
  // Create the stack structures
  stack_size = 2 * tree_height;
  stack = malloc(sizeof(int) * stack_size);
  stack_heights = malloc(sizeof(int) * stack_size);
  stack_occupancy_lengths = malloc(sizeof(int) * stack_size);
  cur_stack_ptr = 0;
  stack[cur_stack_ptr] = 0;
  stack_heights[cur_stack_ptr] = 1;
  stack_occupancy_lengths[cur_stack_ptr] = num_images * width * height;
  
  // Create the occupancy structures.  Since we're desending the tree DFS we
  // need to store the occupancy list only for each level of the stack.  The
  // occupancy list takes up the most memory of all the data structures,
  // It's the size of the (input data * 2 * height), so we're going to pack it
  // tightly into 32 bit ints.
  #ifdef VERBOSE
  printf("initializing occupancy list...\n");
  printf_flush;
  #endif
          occupancy_length = (int)ceil((float)(width*height) / 32.0f);
  if (sizeof(unsigned int) != 4) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:unexpectedTypeSize",
            "unsigned int is not 4 bytes!");
  }
  occupancy = malloc(sizeof(unsigned int) * stack_size *
          num_images * occupancy_length);
  occupancy_left = malloc(sizeof(unsigned int) * num_images *
          occupancy_length);
  occupancy_right = malloc(sizeof(unsigned int) * num_images *
          occupancy_length);
  
  memset(occupancy, 0, sizeof(unsigned int) * stack_size * num_images * occupancy_length);
  // Occupancy for all the images in the first stack point will be the same
  // Calculate the occupancy for one image and then duplicate it.
  for (i = 0; i < occupancy_length; i++) {
    occupancy[i] = 0xffffffff;
  }
  for (i = 1; i < num_images; i++) {
    memcpy(&occupancy[i * occupancy_length], &occupancy[0],
            sizeof(unsigned int) * occupancy_length);
  }
  
  // Initalize the histograms to be (-1, -1) --> Indicates node doesn't exist
  for (i = 0; i < tree_size * 2; i++) {
    dt_hist[i] = -1;
  }
  
  #ifdef VERBOSE
  printf("permuting WL coefficients...\n");
  printf_flush;
  #endif
  num_wl_permutations = permuteWLCoeffs(&wlset_coeffs0, &wlset_coeffs1,
    &wlset_coeffs2, wl_coeffs_sizes, wl_coeffs0, wl_coeffs1, wl_coeffs2);
  if (num_samples_per_node > num_wl_permutations) {
    num_samples_per_node = num_wl_permutations;
  }
  if (tree_height > num_wl_permutations) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:unexpectedData",
            "The weak learner set is smaller than the height of the tree!");
  }
  entropy = malloc(sizeof(float) * stack_size);
  
  // Calculate the entropy of the root and at the same time pack the occupancy
  // image.
  #ifdef VERBOSE
  printf("calculating root entropy...\n");
  printf_flush;
  #endif
  for (i = 0; i < num_images * width * height; i++) {
    switch (label_data[i]) {
      case 0:
        hist_root[0]++;
        break;
      case 1:
        hist_root[1]++;
        break;
      default:
        mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:unexpectedData",
                "Unrecognized pixel type");
        break;
    }
          }
  
  sum_hist = (float)(hist_root[0] + hist_root[1]);
  p_root[0] = (float)(hist_root[0]) / sum_hist;
  p_root[1] = (float)(hist_root[1]) / sum_hist;
  entropy[0] = calcEntropy(p_root);
  dt_hist[0] = hist_root[0];
  dt_hist[1] = hist_root[1];
  
  #ifdef VERBOSE
  printf("  hist_root = <%d, %d>\n", hist_root[0], hist_root[1]);
  printf("  entropy[0] = %+.4f\n", entropy[0]);
  printf_flush;
  #endif
          
  //***************************************************
  //****************** MAIN LOOP **********************
  //***************************************************
  repeat_attempts = 0;
  while (cur_stack_ptr >= 0) {
    printf("Processing tree node %d of %d\n", num_nodes_finished+1, tree_size-1);
    printf_flush;
    
    // Get the next node off the stack
    cur_node = stack[cur_stack_ptr];
    cur_height = stack_heights[cur_stack_ptr];
    
    // Now, from the list of still avaliable WL permutations choose a maximal WL
    // from a random subset
    max_gain = -1.0f;
    index_best_wl = -1;
    num_attempts = 0;
    while (num_attempts < num_samples_per_node) {
      // pick a random weak learner
      index_cur_wl = rand() % num_wl_permutations;
      // We picked a WL that hasn't been used yet --> Success
      num_attempts++;
      cur_u_offset = wlset_coeffs0[index_cur_wl];
      cur_v_offset = wlset_coeffs1[index_cur_wl];
      cur_threshold = wlset_coeffs2[index_cur_wl];
      
      // For all of the data points still alive, bin all of the data points
      // using the weak lerner
      hist_left[0] = 0;
      hist_left[1] = 0;
      hist_right[0] = 0;
      hist_right[1] = 0;
      
      // Optimizations:
      // Optim1: Create a lookup table for the divide and mod operations
      //        --> OK, ~7% faster, but only pre-calculating mod operations
      // Optim2: Try iterating through by image in the inner loop to avoid
      //        lots of branching conditionals for the offsets
      //        --> BAD ~6x slower
      // Optim3: Try handling zero values differently
      // Optim4: Only perform occupancy once the good weak lerner is found 
      //        --> OK.  Only faster when using lots of weak learners.
      temp = 0;  // TEMP CODE
      for (i = 0; i < num_images; i++) {
        // Precompute index zero pointers to save time
        cur_occupancy_im = &occupancy[(cur_stack_ptr * num_images + i) * occupancy_length];
        cur_image_data = &image_data[i * width * height];
        cur_label_data = &label_data[i * width * height];
        
        index = 0;
        for (v = 0; v < height; v++) {
          for (u = 0; u < width; u++) {
            if (((cur_occupancy_im[index / 32] >> (index % 32)) & 1) == 1) {
              u_offset = u + cur_u_offset;
              v_offset = v + cur_v_offset;
              if (u_offset >= 0 && u_offset < width && v_offset >= 0 && v_offset < height) {
                index_offset = v_offset * width + u_offset;
                relative_depth = (float)(cur_image_data[index_offset] - cur_image_data[index]) / ((float)cur_image_data[index]);
                if (relative_depth >= cur_threshold) {
                  // Go left
                  hist_left[cur_label_data[index]]++;
                } else {
                  // Go right
                  hist_right[cur_label_data[index]]++;
                }
              } else {
                #ifdef GO_LEFT_OUTSIDE
                // Go left --> The offset pixel is off the image
                hist_left[cur_label_data[index]]++;
                #else
                // Go right --> The offset pixel is off the image
                hist_right[cur_label_data[index]]++;
                #endif
              }
              temp++;
            } // if (((cur_occupancy_im[index / 32] >> (index%32)) & 1) == 1)
            index++;
          }  // for (u = 0; u < width; u++)
        }  // for (v = 0; v < height; v++)
      }  // for (i = 0; i < num_images; i++)
      
      if (temp != stack_occupancy_lengths[cur_stack_ptr]) {
        mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:unexpectedData",
                "occupancy list is messed up, didn't count as many pixels as was stated on the stack!");
      }
      
      // Check that the histograms add up to our parents population, 
      // otherwise something went very wrong
      if ((hist_left[0] + hist_left[1] + hist_right[0] + hist_right[1]) != 
              stack_occupancy_lengths[cur_stack_ptr]) {
        mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:unexpectedData",
                "children populations do not equal the parent population!");
      }
      
      // Normalize the histograms and calculate entropy
      // Only continue on if we made some sort of split.
      if ((hist_left[0] + hist_left[1]) != 0 && (hist_right[0] + hist_right[1]) != 0) {
        sum_hist_left = (float)(hist_left[0] + hist_left[1]);
        if (sum_hist_left > 0) {
          p_left[0] = ((float)hist_left[0]) / sum_hist_left;
          p_left[1] = ((float)hist_left[1]) / sum_hist_left;
        } else {
          p_left[0] = 0.0f;
          p_left[1] = 0.0f;
        }
        sum_hist_right = (float)(hist_right[0] + hist_right[1]);
        if (sum_hist_right > 0) {
          p_right[0] = ((float)hist_right[0]) / sum_hist_right;
          p_right[1] = ((float)hist_right[1]) / sum_hist_right;
        } else {
          p_right[0] = 0.0f;
          p_right[1] = 0.0f;
        }

        // from the left and right histograms calculate the entropy
        entropy_parent = entropy[cur_stack_ptr];
        entropy_left = calcEntropy(p_left);
        entropy_right = calcEntropy(p_right);

        // Calculate the normalized information gain (using shannon entropy)
        // From Murphy's matlab code (thanks Papa Murphy!):
        // gain = h0 - (h1 * s1/s0 + h2 * s2/s0);
        cur_gain = entropy_parent - (entropy_left * sum_hist_left/stack_occupancy_lengths[cur_stack_ptr] +
                                     entropy_right * sum_hist_right/stack_occupancy_lengths[cur_stack_ptr]);

        if (cur_gain > max_gain) {
          max_gain = cur_gain;
          index_best_wl = index_cur_wl;
          dt_coeffs0[cur_node] = cur_u_offset;
          dt_coeffs1[cur_node] = cur_v_offset;
          dt_coeffs2[cur_node] = cur_threshold;
          hist_left_best[0] = hist_left[0];
          hist_left_best[1] = hist_left[1];
          hist_right_best[0] = hist_right[0];
          hist_right_best[1] = hist_right[1];
          entropy_left_best = entropy_left;
          entropy_right_best = entropy_right;
        }
      } // if ((hist_left[0] + hist_left[1]) != 0 && (hist_right[0] + hist_right[1]) != 0) {
    }  // while (num_attemps < WL_samples_per_node)
  
    if (max_gain > 0) {
      // Now we have the best WL, calculate the occupancy --> Repeats
      // Some calculations, but we could potentially be looking at 1000s of
      // WLs above, so it should save some time in the long run.
      memset(occupancy_left, 0, sizeof(unsigned int) * num_images * occupancy_length);
      memset(occupancy_right, 0, sizeof(unsigned int) * num_images * occupancy_length);
      cur_u_offset = dt_coeffs0[cur_node];
      cur_v_offset = dt_coeffs1[cur_node];
      cur_threshold = dt_coeffs2[cur_node];
      for (i = 0; i < num_images; i++) {
        // Precompute index zero pointers to save time
        cur_occupancy_im = &occupancy[(cur_stack_ptr * num_images + i) * occupancy_length];
        cur_occupancy_im_left = &occupancy_left[i * occupancy_length];
        cur_occupancy_im_right = &occupancy_right[i * occupancy_length];
        cur_image_data = &image_data[i * width * height];
        cur_label_data = &label_data[i * width * height];
        
        index = 0;
        for (v = 0; v < height; v++) {
          for (u = 0; u < width; u++) {
            if (((cur_occupancy_im[index / 32] >> (index % 32)) & 1) == 1) {
              u_offset = u + cur_u_offset;
              v_offset = v + cur_v_offset;
              if (u_offset >= 0 && u_offset < width && v_offset >= 0 && v_offset < height) {
                index_offset = v_offset * width + u_offset;
                relative_depth = (float)(cur_image_data[index_offset] - cur_image_data[index]) / ((float)cur_image_data[index]);
                if (relative_depth >= cur_threshold) {
                  // Go left
                  cur_occupancy_im_left[index / 32] |= (1 << (index % 32));
                } else {
                  // Go right
                  cur_occupancy_im_right[index / 32] |= (1 << (index % 32));
                }
              } else {
                #ifdef GO_LEFT_OUTSIDE
                // Go left
                cur_occupancy_im_left[index / 32] |= (1 << (index % 32));
                #else
                // Go right
                cur_occupancy_im_right[index / 32] |= (1 << (index % 32));
                #endif
              }
            } // if (((cur_occupancy_im[index / 32] >> (index%32)) & 1) == 1)
            index++;
          }  // for (u = 0; u < width; u++)
        }  // for (v = 0; v < height; v++)
      }  // for (i = 0; i < num_images; i++)

      #ifdef VERBOSE
      printf("    Best weak lerner: <%d, %d, %.3f> with info gain %.3f\n", dt_coeffs0[cur_node], dt_coeffs1[cur_node], dt_coeffs2[cur_node], max_gain);
      printf("    hist_left  = <%d, %d>\n", hist_left_best[0], hist_left_best[1]);
      printf("    hist_right = <%d, %d>\n", hist_right_best[0], hist_right_best[1]);
      printf_flush;
      #endif

      l_child_node = getLChild(cur_node);
      r_child_node = getRChild(cur_node);

      dt_hist[l_child_node*2] = hist_left_best[0];
      dt_hist[l_child_node*2+1] = hist_left_best[1];
      dt_hist[r_child_node*2] = hist_right_best[0];
      dt_hist[r_child_node*2+1] = hist_right_best[1];

      // Pop off the stack
      cur_stack_ptr--;

      // Are we a leaf? If we're not, then put our children and the necessary data
      // on the stack
      if (cur_height < (tree_height - 1)) {
        // Add left child to the stack for processing
        if (hist_left_best[0] != 0 && hist_left_best[1] != 0) {
          cur_stack_ptr++;
          stack[cur_stack_ptr] = l_child_node;
          stack_heights[cur_stack_ptr] = cur_height + 1;
          stack_occupancy_lengths[cur_stack_ptr] = hist_left_best[0] + hist_left_best[1];
          memcpy(&occupancy[cur_stack_ptr * num_images * occupancy_length],
                  occupancy_left,
                  sizeof(unsigned int) * num_images * occupancy_length);
          entropy[cur_stack_ptr] = entropy_left_best;
        } else {
          // Number of sub-children we will be skipping over
          num_nodes_finished += (1<<(tree_height - cur_height)) - 1;
        }

        // Add right child to the stack for processing
        if (hist_right_best[0] != 0 && hist_right_best[1] != 0) {
          cur_stack_ptr++;
          stack[cur_stack_ptr] = r_child_node;
          stack_heights[cur_stack_ptr] = cur_height + 1;
          stack_occupancy_lengths[cur_stack_ptr] = hist_right_best[0] + hist_right_best[1];
          memcpy(&occupancy[cur_stack_ptr * num_images * occupancy_length],
                  occupancy_right,
                  sizeof(unsigned int) * num_images * occupancy_length);
          entropy[cur_stack_ptr] = entropy_right_best;
        } else {
          // Number of sub-children we will be skipping over
          num_nodes_finished += (1<<(tree_height - cur_height)) - 1;
        }
      } else {
        num_nodes_finished += 2;  // To count for the leaf nodes we finished
      }

      num_nodes_finished++;
      repeat_attempts = 0;
    } else {
      #ifdef VERBOSE
      printf("Couldn't find a weak learner, trying again %d of %d times\n", repeat_attempts, max_repeat_attempts);
      printf_flush;
      #endif
      // No information gain, try again on the same node
      repeat_attempts++;
      if (repeat_attempts > max_repeat_attempts) {
        // Don't loop for ever.  Just make this node a leaf.
        // Pop off the stack
        cur_stack_ptr--;
        #ifdef VERBOSE
        printf("Still Couldn't find a weak learner after %d attempts, making this node a leaf\n", repeat_attempts);
        printf_flush;
      	#endif
      }
    }
  }  // while (cur_stack_ptr >= 0)

  // Clean up on finish
  free(stack);
  free(stack_heights);
  free(stack_occupancy_lengths);
  free(occupancy);
  free(occupancy_left);
  free(occupancy_right);
  free(wlset_coeffs0);
  free(wlset_coeffs1);
  free(wlset_coeffs2);
}

// The gateway function
void mexFunction(int nlhs, mxArray *plhs[],
        int nrhs, const mxArray *prhs[]) {
  // For no good reason the Microsoft compiler requires ALL C variable
  // declarations to happen at the START of the function.  Very annoying...
  size_t mrows_image, ncols_image;
  size_t mrows_label, ncols_label;
  size_t mrows, ncols;
  int tree_height;
  int tree_size;
  mwSize hist_array_size[2];
  int num_samples_per_node;
  int width;
  int height;
  int num_images;
  int wl_coeffs_sizes[3];
  int random_seed;
  
  // Check for proper number of arguments.
  if (nrhs != 9) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidNumInputs",
            "9 inputs required.");
  }
  if (nlhs != 4) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidNumOutputs",
            "4 outputs required.");
  }
  
  // Check the types of the inputs
  if (!mxIsClass(prhs[0], "uint16") || mxIsComplex(prhs[0])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[0] is not real uint16");
  }
  if (!mxIsClass(prhs[1], "uint16") || mxIsComplex(prhs[1])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[1] is not real uint16");
  }
  if (!mxIsClass(prhs[2], "int32") || mxIsComplex(prhs[2])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[2] is not real int32");
  }
  if (!mxIsClass(prhs[3], "int32") || mxIsComplex(prhs[3])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[3] is not real int32");
  }
  if (!mxIsClass(prhs[4], "single") || mxIsComplex(prhs[4])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[4] is not real single");
  }
  if (!mxIsClass(prhs[5], "int32") || mxIsComplex(prhs[5])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[5] is not real int32");
  }
  if (!mxIsClass(prhs[6], "int32") || mxIsComplex(prhs[6])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[6] is not real int32");
  }
  if (!mxIsClass(prhs[7], "int32") || mxIsComplex(prhs[7])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[7] is not real int32");
  }
  if (!mxIsClass(prhs[8], "int32") || mxIsComplex(prhs[8])) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputType",
            "prhs[8] is not real int32");
  }
  
  // Check the size of the inputs
  mrows_image = mxGetM(prhs[0]);
  ncols_image = mxGetN(prhs[0]);
  mrows_label = mxGetM(prhs[1]);
  ncols_label = mxGetN(prhs[1]);
  if (mrows_image != mrows_label || ncols_image != ncols_label) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputSize",
            "prhs[0] is no the same size as prhs[1]");
  }
  if (mxGetNumberOfDimensions(prhs[0]) != 2) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputSize",
            "prhs[0] is not 2 dimensional");
  }
  if (mxGetNumberOfDimensions(prhs[1]) != 2) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputSize",
            "prhs[1] is not 2 dimensional");
  }
  mrows = mxGetM(prhs[2]);
  if (mrows != 1) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputSize",
            "prhs[2] should be a 1d column vector");
  }
  mrows = mxGetM(prhs[3]);
  if (mrows != 1) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputSize",
            "prhs[3] should be a 1d column vector");
  }
  mrows = mxGetM(prhs[4]);
  if (mrows != 1) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputSize",
            "prhs[4] should be a 1d column vector");
  }
  mrows = mxGetM(prhs[5]);
  ncols = mxGetN(prhs[5]);
  if (mrows != 1 || ncols != 1) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputSize",
            "prhs[5] should be a single value");
  }
  mrows = mxGetM(prhs[6]);
  ncols = mxGetN(prhs[6]);
  if (mrows != 1 || ncols != 1) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputSize",
            "prhs[6] should be a single value");
  }
  mrows = mxGetM(prhs[7]);
  ncols = mxGetN(prhs[7]);
  if (mrows != 1 || ncols != 2) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputSize",
            "prhs[7] should be a [1x2] vector");
  }
  mrows = mxGetM(prhs[8]);
  ncols = mxGetN(prhs[8]);
  if (mrows != 1 || ncols != 1) {
    mexErrMsgIdAndTxt("MATLAB:generateDecisionTree:invalidInputSize",
            "prhs[8] should be a [1x1] vector");
  }
  
  // Allocate the LHS values:
  tree_height = *(int*)mxGetData(prhs[6]);
  tree_size = calcTreeSize(tree_height);
  plhs[0] = mxCreateNumericArray((mwSize)1,  // mwSize ndim
          (mwSize*)&tree_size,  // const mwSize *dims
          mxINT32_CLASS,  // mxClassID classid
          mxREAL);  // mxComplexity ComplexFlag
  plhs[1] = mxCreateNumericArray((mwSize)1,  // mwSize ndim
          (mwSize*)&tree_size,  // const mwSize *dims
          mxINT32_CLASS,  // mxClassID classid
          mxREAL);  // mxComplexity ComplexFlag
  plhs[2] = mxCreateNumericArray((mwSize)1,  // mwSize ndim
          (mwSize*)&tree_size,  // const mwSize *dims
          mxSINGLE_CLASS,  // mxClassID classid
          mxREAL);  // mxComplexity ComplexFlag
  hist_array_size[0] = (mwSize)2;
  hist_array_size[1] = (mwSize)tree_size;
  plhs[3] = mxCreateNumericArray((mwSize)2,  // mwSize ndim
          (mwSize*)hist_array_size,  // const mwSize *dims
          mxINT32_CLASS,  // mxClassID classid
          mxREAL);  // mxComplexity ComplexFlag
  
  num_samples_per_node = *(int*)mxGetData(prhs[5]);
  width = ((int*)mxGetData(prhs[7]))[0];
  height = ((int*)mxGetData(prhs[7]))[1];
  num_images = (int)ncols_image;
  wl_coeffs_sizes[0] = (int)mxGetN(prhs[2]);
  wl_coeffs_sizes[1] = (int)mxGetN(prhs[3]);
  wl_coeffs_sizes[2] = (int)mxGetN(prhs[4]);
  random_seed = ((int*)mxGetData(prhs[8]))[0];
  
  generateDecisionTree((int*)mxGetData(plhs[0]), (int*)mxGetData(plhs[1]),
          (float*)mxGetData(plhs[2]), (int*)mxGetData(plhs[3]), num_images,
          (short*)mxGetData(prhs[0]), (short*)mxGetData(prhs[1]),
          (int*)mxGetData(prhs[2]), (int*)mxGetData(prhs[3]),
          (float*)mxGetData(prhs[4]), wl_coeffs_sizes, num_samples_per_node,
          tree_height, width, height, random_seed);
}
