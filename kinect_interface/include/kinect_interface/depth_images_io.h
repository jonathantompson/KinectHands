//
//  depth_images_io.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#ifndef KINECT_INTERFACE_DEPTH_IMAGES_IO_HEADER
#define KINECT_INTERFACE_DEPTH_IMAGES_IO_HEADER

#include <string>
#include <iostream>
#include <fstream>
#include "jtil/math/math_types.h"
#include "jtil/data_str/vector_managed.h"
#include "kinect_interface/depth_image_data.h"

#define src_width 640
#define src_height 480
#define src_dim (src_width * src_height)
// Note (width / downsample) && (height / downsample) must be integer values!

#define BACKGROUND_HSV_THRESH 10
#define BACKGROUND_DEPTH_THRESH 20  // Maximum depth distance
#define BACKGROUND_DEPTH_THRESH_GROW 15
#define RED_MED_FILT_RAD 4
#define RED_SHRINK_FILT_RAD 2  // OLD Value = 3
#define RED_DISCONT_FILT_RAD 3 // OLD_VALUE = 3
#define RED_DISCONT_FILT_DEPTH_THRESH BACKGROUND_DEPTH_THRESH
#define HAND_PTS_GROW_RAD 2000  // Divided by depth!
#define HAND_PTS_GROW_RAD_ITERATIONS 6  // OLD value = 4
#define N_PTS_FILL 8

namespace kinect_interface {

  typedef enum {
    IM_DEPTH,
    IM_RGB,
    IM_HSV,
    IM_HUE,
    IM_SAT,
    IM_VAL,
    IM_NUM_TYPES,
  } IM_TYPE;

  class DepthImagesIO {
  public:
    DepthImagesIO();
    ~DepthImagesIO();

    // This version is for the decision tree forest project
    void LoadDepthImagesFromDirectoryForDT(std::string directory, 
      DepthImageData*& training_data, DepthImageData*& test_data,
      float frac_test_data, uint32_t file_skip);

    // Get a listing of all the files in the directory
    // - load_processed_images = true --> hands_*.bin
    // - load_processed_images = false --> processed_hands_*.bin
    uint32_t GetFilesInDirectory(jtil::data_str::VectorManaged<char*>& files_names, 
      std::string directory, bool load_processed_images);

    static void releaseImages(DepthImageData*& data);

    // LoadRGBImage - Load only the RGB image
    void LoadRGBImage(std::string file, uint8_t* rgb);

    // Load a compressed depth+RGB image
    void LoadCompressedImage(std::string file, int16_t* depth_data, 
      uint8_t* label_data, uint8_t* rgb_data = NULL);

    // Load a compressed depth+RGB image with red hands and do post-processing
    void LoadCompressedImageWithRedHands(std::string file, int16_t* depth_data,
      uint8_t* label_data, uint8_t* rgb_data = NULL, uint8_t* red_pixels = NULL,
      uint8_t* hsv_pixels_ret = NULL);

    // saveProcessedImages - Needs updating
    void saveProcessedImage(std::string directory, char* filename, 
      int16_t* image_data, uint8_t* label_data, int32_t width, int32_t height);

    template <typename T>
    void saveUncompressedDepth(const std::string file, 
      const T* depth_data, const uint32_t w = src_width, 
      const uint32_t h = src_height);

    // floodPixel - Manual editing of a label image (by flooding on the depth)
    void floodPixel(uint8_t* label_image, int16_t* depth_image, int u, int v, 
      int32_t radius, int16_t thresh);

    // convertImageDepthToXYZ - Use OpenNI functions for UVD --> XYZ conversion
    static void convertImageDepthToXYZ(float*& xyz, DepthImageData*& images);
    static void convertSingleImageToXYZ(float* xyz, int16_t* depth);

    // testRedPixel - Single "red-test" of a hsv+rgb pixel (for debugging only)
    static void testRedPixel(uint32_t index, uint8_t* hsv, uint8_t* rgb);

    // Some tweakable parameters of the red hand processing:
    static int32_t red_hue_threshold;
    static int32_t red_sat_threshold;
    static int32_t red_val_threshold;
    static int32_t red_hue_target;
    static int32_t red_sat_target;
    static int32_t red_val_target;
    static float sink_source_beta;
    // static uint32_t graph_cut_affiliation_radius;
    static int32_t red_red_min;
    static int32_t red_blue_max;
    static float adjacency_gamma;
    static float adjacency_beta;

  private:
    bool LoadProcessedCompressedImageWithRedHands(std::string file, 
      int16_t* depth_data, uint8_t* label_data, uint8_t* rgb_data = NULL, 
      uint8_t* red_pixels = NULL, uint8_t* hsv_pixels_ret = NULL);
    void SaveProcessedCompressedImageWithRedHands(std::string file, 
      int16_t* depth_data, uint8_t* label_data, uint8_t* rgb_data, 
      uint8_t* red_pixels, uint8_t* hsv_pixels);

    void getRedPixels(uint8_t* rgb, uint8_t* hsv, uint8_t* red_pixels);
    //void cleanUpSegmentWithGraphCut(uint8_t* return_label_data,
    //  int16_t* image_data, uint8_t* red_label_data);
    void cleanUpRedPixelsUsingDepth(int16_t* depth_data, uint8_t* red_pixels);
    void findHandPoints(uint8_t* label_data, uint8_t* red_pixels, 
      int16_t* depth_data);
    void growHandPoints(uint8_t* new_label_data, uint8_t* label_data, 
      int16_t* depth_data, float radius);
    void processGloveNeighbour(int16_t* depth_data, int* nieghbourPtUV, 
      int curPtIndex);
    void processFloodPixelNeighbour(int16_t* depth_data, uint8_t* label_data, 
      int* nieghbourPtUV, int curPtIndex, uint8_t label_to_flood);

    void resetBlobDetection();
    bool findNextBlob(uint32_t& blob_index, uint32_t& blob_size, 
      int16_t* depth_data, uint8_t* labels);
    void processBlobNeighbour(int16_t* depth_data, int* nieghbourPtUV, 
      int curPtIndex, uint8_t* labels);
    void zeroBlob(const uint32_t blob_index, int16_t* depth_data, 
      uint8_t* labels);

    // Some temporary space for processing each image
    uint16_t* compressed_data;
    uint16_t* uncompressed_data;
    uint8_t* user_pixels;
    uint8_t* hsv;
    uint8_t* rgb;  // Doesn't need to be freed! --> Part of uncompressed data
    uint8_t* label_data_tmp;
    int* label_data_int;
    int* label_data_int_integ;
    int* label_data_int_tmp;
    uint8_t* red_pixels;
    uint8_t* red_pixels_tmp;
    int data_size;
    int processed_data_size;
    bool* pixel_on_queue;
    int* pixel_queue;
    int16_t* cur_image_data;
    uint8_t* cur_label_data;
    int queue_head;
    int queue_tail;
    int32_t delta_hsv[3];
    int16_t flood_threshold;
    float affiliation_diam;
    float affiliation_cnt;
    uint32_t blob_i;

    static const int floodFillKernel_[N_PTS_FILL][2];
  };

  template <typename T>
  void DepthImagesIO::saveUncompressedDepth(const std::string filename, 
    const T* depth_data, const uint32_t w, const uint32_t h) {
    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + filename);
    }
    file.write(reinterpret_cast<const char*>(depth_data), 
      w * h * sizeof(depth_data[0]));
    file.flush();
    file.close();
  }

};  // namespace depth_images_io

#endif  // UNNAMED_LOAD_DEPTH_IMAGE_HEADER
