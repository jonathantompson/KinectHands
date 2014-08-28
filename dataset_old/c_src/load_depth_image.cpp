#include <stdint.h>  // For uint8_t, uint16_t, etc
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "fastlz.h"
#include "mex.h"
#include "matrix.h"

using namespace std;

#define SAFE_FREE(x) do { if (x != NULL) { free(x); x = NULL; } } while (0); 

const uint32_t im_w = 640;  // TODO: Unfortunately this is hard coded
const uint32_t im_h = 480;  // we should probably store size in a header
const uint32_t im_dim = im_w * im_h;

void LoadKinectImage(const string& file, 
  int16_t* depth_data, uint8_t* rgb_data, const bool compressed = true) {
  std::ifstream in_file(file.c_str(), std::ios::in | std::ios::binary | 
    std::ios::ate);
  if (!in_file.is_open()) {
    throw std::runtime_error(std::string("LoadCompressedImage()") + 
      std::string(": error opening file") + file);
  }
  
  // Calculate the size of the uncompressed data on disk
  uint16_t dummy16;
  uint8_t dummy8;
  static_cast<void>(dummy16);  // for compiler warnings
  static_cast<void>(dummy8);
      
  int data_size = (im_dim * sizeof(dummy16) +   // Depth value
                   im_dim * 3 *sizeof(dummy8));

  // Allocate more than we need just in case the compressed data is actually
  // larger than the uncompressed data (this could happen if the data is
  // incompressible and the compression format adds overhead).
  uint16_t* compressed_data = (uint16_t*)malloc(data_size * 2);  
  uint16_t* uncompressed_data =  (uint16_t*)malloc(data_size * 2);
  
  if (compressed) {
    uint32_t size_bytes = static_cast<uint32_t>(in_file.tellg());
    in_file.seekg (0, std::ios::beg);  // Go to the beginning of the file
    in_file.read(reinterpret_cast<char*>(compressed_data), size_bytes);
    in_file.close();

    int size_decompress = fastlz_decompress(reinterpret_cast<void*>(compressed_data),
      size_bytes,
      reinterpret_cast<void*>(uncompressed_data),
      data_size * 2);
    if (size_decompress != data_size) {
      std::stringstream ss;
      ss << "LoadCompressedImage() - ERROR: uncompressed data size is not "
        "what we expected!  File: ";
      ss << file << ", size: " << size_decompress << ", expected: ";
      ss << data_size;
      throw runtime_error(ss.str());
    }

    memcpy(depth_data, uncompressed_data, im_dim * sizeof(depth_data[0]));
    uint8_t* rgb = reinterpret_cast<uint8_t*>(&uncompressed_data[im_dim]);
    memcpy(rgb_data, rgb, 3 * im_dim * sizeof(rgb_data[0]));
  } else {
    uint32_t uncompressed_size = im_dim * sizeof(depth_data[0]) + 
      3 * im_dim * sizeof(rgb_data[0]);
    uint32_t size_bytes = static_cast<uint32_t>(in_file.tellg());
    if (size_bytes != uncompressed_size) {
      throw std::runtime_error("Size on disk is incorrect!");
    }
    in_file.seekg (0, std::ios::beg);  // Go to the beginning of the file
    in_file.read(reinterpret_cast<char*>(depth_data), 
      im_dim * sizeof(depth_data[0]));
    in_file.read(reinterpret_cast<char*>(rgb_data), 
      3 * im_dim * sizeof(rgb_data[0]));
    in_file.close();
  }
  
  SAFE_FREE(compressed_data);
  SAFE_FREE(uncompressed_data);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // First check that the inputs and output exist
  if (nlhs != 2) {
    mexErrMsgIdAndTxt("MATLAB:load_depth_image:invalidNumOutput",
      "You must define two return values!");
  }
  if (nrhs != 1) {
    mexErrMsgIdAndTxt("MATLAB:load_depth_image:invalidNumInput",
      "Invalid number of inputs");
  } 
  
  // input must be a string
  if (mxIsChar(prhs[0]) != 1) {
    mexErrMsgIdAndTxt("MATLAB:load_depth_image:inputNotString",
      "Input must be a string.");
  }
  
  char *filename = mxArrayToString(prhs[0]);
  if (filename == NULL) {
    mexErrMsgIdAndTxt("MATLAB:load_depth_image:conversionFailed",
      "Could not convert input to string.");
  }
  
  // Load the data in from our compressed format
  uint8_t* rgb = new uint8_t[im_dim * 3];
  uint16_t* depth = new uint16_t[im_dim];
  LoadKinectImage(filename, (int16_t*)depth, rgb);
  
  // Now copy the depth and rgb to the output structure
  mwSize depth_dims[3] = {im_h, im_w};
  plhs[0] = mxCreateNumericArray(2, depth_dims, mxUINT16_CLASS, mxREAL);
  mwSize rgb_dims[3] = {im_h, im_w, 3};
  plhs[1] = mxCreateNumericArray(3, rgb_dims, mxUINT8_CLASS, mxREAL);
  
  int16_t* depth_data = (int16_t*)mxGetData(plhs[0]);
  uint8_t* rgb_data = (uint8_t*)mxGetData(plhs[1]);
  
  for (uint32_t v = 0; v < im_h; v++) {
    for (uint32_t u = 0; u < im_w; u++) {
      depth_data[v + u * im_h] = depth[u + v * im_w];
    }
  }
  mwIndex subs[3];
  for (uint32_t v = 0; v < im_h; v++) {
    for (uint32_t u = 0; u < im_w; u++) {
      for (uint32_t c = 0; c < 3; c++) {
        subs[0] = v;
        subs[1] = u;
        subs[2] = c;
        rgb_data[mxCalcSingleSubscript(plhs[1],3,subs)] = rgb[(u + v * im_w)*3+c];
      }
    }
  }
  
  delete[] rgb;
  delete[] depth;
}