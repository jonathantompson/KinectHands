//
//  load_depth_images.cpp
//
//  Created by Jonathan Tompson on 7/20/12.
//

#if defined(WIN32) || defined(_WIN32)
  #include <windows.h>
  #include <tchar.h> 
  #include <stdio.h>
  #include <strsafe.h>
  #pragma comment(lib, "User32.lib")
  #else
  #include <sys/types.h>
  #include <dirent.h>
#endif
#include <cstring>
#include <stdexcept>
#include "kinect_interface/depth_images_io.h"
#include "kinect_interface/open_ni_funcs.h"
#include "kinect_interface/hand_detector/decision_tree_structs.h"
#include "jtil/math/math_types.h"
#include "jtil/image_util/image_util.h"
#include "jtil/string_util/string_util.h"
#include "jtil/data_str/vector_managed.h"
#include "jtil/fastlz/fastlz.h"
#include "jtil/exceptions/wruntime_error.h"

using std::string;
using std::cout;
using std::endl;
using std::wruntime_error;
using jtil::data_str::VectorManaged;
using namespace jtil::image_util;

#define SAFE_FREE(x) do { if (x != NULL) { free(x); x = NULL; } } while (0); 
#define SAFE_DELETE(x) do { if (x != NULL) { delete x; x = NULL; } } while (0); 
#define SAFE_DELETE_ARR(x) do { if (x != NULL) { delete[] x; x = NULL; } } while (0); 

namespace kinect_interface {

  // uint32_t DepthImagesIO::graph_cut_affiliation_radius = 5;
  float DepthImagesIO::adjacency_gamma = 25.0f;
  float DepthImagesIO::adjacency_beta = 0.25f;

  //// Values for HandFit from 2013_03_04
  //int32_t DepthImagesIO::red_hue_threshold = 15;
  //int32_t DepthImagesIO::red_sat_threshold = 41; 
  //int32_t DepthImagesIO::red_val_threshold = 73; 
  //int32_t DepthImagesIO::red_hue_target = 1;
  //int32_t DepthImagesIO::red_sat_target = 231;  // 225
  //int32_t DepthImagesIO::red_val_target = 161;
  //int32_t DepthImagesIO::red_red_min = 70;
  //int32_t DepthImagesIO::red_blue_max = 100;
  //int32_t DepthImagesIO::hsv_total_threshold = 660;
  //int32_t DepthImagesIO::red_shrink_filter_rad = 1;
  //int32_t DepthImagesIO::red_discon_filter_rad = 1;
  //int32_t DepthImagesIO::red_med_filter_rad = 4;
  //int32_t DepthImagesIO::hand_pts_grow_rad_iterations = 6;

  //// Values for HandFit from 2013_01_11
  //int32_t DepthImagesIO::red_hue_threshold = 27;
  //int32_t DepthImagesIO::red_sat_threshold = 48; 
  //int32_t DepthImagesIO::red_val_threshold = 79;
  //int32_t DepthImagesIO::red_hue_target = 250;
  //int32_t DepthImagesIO::red_sat_target = 214;
  //int32_t DepthImagesIO::red_val_target = 161;
  //int32_t DepthImagesIO::red_red_min = 95;
  //int32_t DepthImagesIO::red_blue_max = 100;
  //int32_t DepthImagesIO::hsv_total_threshold = 660;
  //int32_t DepthImagesIO::red_shrink_filter_rad = 1;
  //int32_t DepthImagesIO::red_discon_filter_rad = 1;
  //int32_t DepthImagesIO::red_med_filter_rad = 4;
  //int32_t DepthImagesIO::hand_pts_grow_rad_iterations = 6;

  // Values for HandForests from 2013_01_11
  int32_t DepthImagesIO::red_hue_threshold = 15;
  int32_t DepthImagesIO::red_sat_threshold = 41; 
  int32_t DepthImagesIO::red_val_threshold = 73;
  int32_t DepthImagesIO::red_hue_target = 250;
  int32_t DepthImagesIO::red_sat_target = 214;
  int32_t DepthImagesIO::red_val_target = 161;
  int32_t DepthImagesIO::red_red_min = 70;
  int32_t DepthImagesIO::red_blue_max = 80;
  int32_t DepthImagesIO::red_green_max = 40;
  int32_t DepthImagesIO::hsv_total_threshold = 660;
  int32_t DepthImagesIO::red_shrink_filter_rad = 2;
  int32_t DepthImagesIO::red_discon_filter_rad = 3;
  int32_t DepthImagesIO::red_med_filter_rad = 4;
  int32_t DepthImagesIO::hand_pts_grow_rad_iterations = 6;

  //// Values for HandForests from 2012_07_27
  //int32_t DepthImagesIO::red_hue_threshold = 15;
  //int32_t DepthImagesIO::red_sat_threshold = 41; 
  //int32_t DepthImagesIO::red_val_threshold = 73; 
  //int32_t DepthImagesIO::red_hue_target = 1;
  //int32_t DepthImagesIO::red_sat_target = 231;  // 225
  //int32_t DepthImagesIO::red_val_target = 161;
  //int32_t DepthImagesIO::red_red_min = 70;
  //int32_t DepthImagesIO::red_blue_max = 100;
  //int32_t DepthImagesIO::hsv_total_threshold = 660;
  //int32_t DepthImagesIO::red_shrink_filter_rad = 3;
  //int32_t DepthImagesIO::red_discon_filter_rad = 3;
  //int32_t DepthImagesIO::red_med_filter_rad = 4;
  //int32_t DepthImagesIO::hand_pts_grow_rad_iterations = 4;

  const int DepthImagesIO::floodFillKernel_[N_PTS_FILL][2] = 
  {{-1, -1}, {-1, 0}, {-1, +1}, {0, +1}, {+1, +1}, {+1, 0}, {+1, -1}, {0, -1}};

  DepthImagesIO::DepthImagesIO() {
    // Allocate space for the temporary data (for compression)
    uint16_t dummy16;
    uint8_t dummy8;
    int dummyint;
    bool dummybool;
#if defined(WIN32) || defined(_WIN32)
    static_cast<void>(dummy16);  // Get rid of unreference local variable warn
    static_cast<void>(dummy8);
    static_cast<void>(dummyint);
    static_cast<void>(dummybool);
#endif  
    // Allocate enough for the full processed data files as well...
    processed_data_size = (src_dim * sizeof(dummy16) +   // Depth value
      src_dim * 3 *sizeof(dummy8) +  // RGB value
      src_dim * 3 *sizeof(dummy8) +  // HSV value
      src_dim * sizeof(dummy8) +  // label data
      src_dim * sizeof(dummy8));  // red pixels
    data_size = (src_dim * sizeof(dummy16) +   // Depth value
      src_dim * 3 *sizeof(dummy8));  // RGB Val
    // '2x' for conservative over allocation
    uncompressed_data = (uint16_t*)malloc(processed_data_size * 2);  
    compressed_data = (uint16_t*)malloc(processed_data_size * 2);
    user_pixels = (uint8_t*)malloc(src_dim * sizeof(dummy8));
    hsv = (uint8_t*)malloc(src_dim * sizeof(dummy8) * 3);
    rgb = NULL;  // Not actually allocated!
    red_pixels = (uint8_t*)malloc(src_dim * sizeof(dummy8));
    red_pixels_tmp = (uint8_t*)malloc(src_dim * sizeof(dummy8));
    pixel_on_queue = (bool*)malloc(src_dim * sizeof(dummybool));
    pixel_queue = (int*)malloc(src_dim * sizeof(dummyint));
    label_data_tmp = (uint8_t*)malloc(src_dim * sizeof(dummy8));
    label_data_int = (int*)malloc(src_dim * sizeof(dummyint));
    label_data_int_integ = (int*)malloc(src_dim * sizeof(dummyint));
    label_data_int_tmp = (int*)malloc(src_dim * sizeof(dummyint));
    cur_image_data = new int16_t[src_dim];
    cur_label_data = new uint8_t[src_dim];
    // im_graph = NULL;
  }

  DepthImagesIO::~DepthImagesIO() {
    SAFE_FREE(compressed_data);
    SAFE_FREE(uncompressed_data);
    SAFE_FREE(user_pixels);
    SAFE_FREE(hsv);
    SAFE_FREE(red_pixels);
    SAFE_FREE(red_pixels_tmp);
    SAFE_FREE(pixel_on_queue);
    SAFE_FREE(pixel_queue);
    SAFE_FREE(label_data_tmp);
    SAFE_FREE(label_data_int);
    SAFE_FREE(label_data_int_integ);
    SAFE_FREE(label_data_int_tmp);
    SAFE_DELETE_ARR(cur_image_data);
    SAFE_DELETE_ARR(cur_label_data);
    // SAFE_DELETE(im_graph);
  }

  // LoadDepthImagesFromDirectoryForDT
  // This method will split the data into training data and test data, as well
  // as downsample the depth image as desired.
  void DepthImagesIO::LoadDepthImagesFromDirectoryForDT(const string& directory, 
    DepthImageData*& train_data, DepthImageData*& test_data,
    const float frac_test_data, const uint32_t file_stride) {
    VectorManaged<char*> files_in_directory;
    const bool load_processed_images = true;  // don't change this!
    uint32_t num_files = GetFilesInDirectory(files_in_directory, directory,
      load_processed_images);
    if (num_files == 0) {
      throw std::runtime_error("ERROR: no files in the database!\n");
    }
    cout << "  --> Total number of files in the database = " << num_files << endl;
    num_files = num_files / file_stride;
    cout << "  --> Using " << num_files << " of these files:" << endl;

    test_data = new DepthImageData;
    train_data = new DepthImageData;

    uint32_t stride_test_data = (uint32_t)(round(1.0f / frac_test_data));
    test_data->num_images = (uint32_t)(ceil((float)(num_files) / 
      (float)(stride_test_data)));
    train_data->num_images = num_files - test_data->num_images;

    bool load_training_data = test_data->num_images != 0;
    if (!load_training_data) {
      cout << "LoadDepthImagesFromDirectory - Warning, not reserving any ";
      cout << "images for the test set!"  << endl;
      cout << "  --> frac_test_data is too low!" << endl;
    }

    // Allocate enough space for the images:
    uint32_t num_pix = src_dim;
    uint32_t num_pix_downs = num_pix / (DT_DOWNSAMPLE*DT_DOWNSAMPLE);

    if (src_width % DT_DOWNSAMPLE != 0 || src_height % DT_DOWNSAMPLE != 0) {
      throw wruntime_error(string("LoadDepthImagesFromDirectory downsample") +
        string(" factor must be an integer multiple"));
    }

    train_data->image_data = new int16_t[num_pix_downs * train_data->num_images];
    train_data->label_data = new uint8_t[num_pix_downs * train_data->num_images];
    train_data->im_width = src_width / DT_DOWNSAMPLE;
    train_data->im_height = src_height / DT_DOWNSAMPLE;
    train_data->filenames = new char*[train_data->num_images];
    train_data->rgb_data = NULL;

    if (load_training_data) {
      test_data->image_data = new int16_t[num_pix_downs * test_data->num_images];
      test_data->label_data = new uint8_t[num_pix_downs * test_data->num_images];
      test_data->filenames = new char*[test_data->num_images];
    } else {
      test_data->image_data = NULL;
      test_data->label_data = NULL;
      test_data->filenames = NULL;
    }
    test_data->rgb_data = NULL;
    test_data->im_width = src_width / DT_DOWNSAMPLE;
    test_data->im_height = src_height / DT_DOWNSAMPLE;  

    // Find the first file in the directory again so that we can iterate through
    uint32_t cur_test_image = 0;
    uint32_t cur_training_image = 0;
    uint8_t* label_dst;
    int16_t* image_dst;
    for (uint32_t i = 0; i < num_files; i++) {
      if ((i % 100) == 0 || i == num_files - 1) {
        std::cout << "      loading image " << i + 1 << " of " << num_files << std::endl;
      }
      std::string cur_filename = string(*files_in_directory.at(i*file_stride));
      if (DT_DOWNSAMPLE > 1) {
        if (load_training_data && i % stride_test_data == 0) {
          test_data->filenames[cur_test_image] = new char[cur_filename.length() + 1];
          strcpy(test_data->filenames[cur_test_image], cur_filename.c_str());
          image_dst = &test_data->image_data[cur_test_image * num_pix_downs];
          label_dst = &test_data->label_data[cur_test_image * num_pix_downs];
          cur_test_image++;
        } else {
          train_data->filenames[cur_training_image] = new char[cur_filename.length() + 1];
          strcpy(train_data->filenames[cur_training_image], cur_filename.c_str());
          image_dst = &train_data->image_data[cur_training_image * num_pix_downs];
          label_dst = &train_data->label_data[cur_training_image * num_pix_downs];
          cur_training_image++;
        }

        loadProcessedDepthLabel(directory + cur_filename, cur_image_data,
          cur_label_data);
        // Downsample but ignore 0 or background pixel values when filtering
        DownsampleImageWithoutNonZeroPixelsAndBackground<int16_t>(
          image_dst, cur_image_data, src_width, src_height, DT_DOWNSAMPLE,
          GDT_MAX_DIST);
        DownsampleBoolImageConservative<uint8_t>(label_dst, 
          cur_label_data, src_width, src_height, DT_DOWNSAMPLE, 0, 1);

      } else {
        if (load_training_data && i % stride_test_data == 0) {
          test_data->filenames[cur_test_image] = new char[cur_filename.length() + 1];
          strcpy(test_data->filenames[cur_test_image], cur_filename.c_str());
          image_dst = &test_data->image_data[cur_test_image * num_pix_downs];       
          label_dst = &test_data->label_data[cur_test_image * num_pix_downs];
          cur_test_image++;
        } else {
          train_data->filenames[cur_training_image] = new char[cur_filename.length() + 1];
          strcpy(train_data->filenames[cur_training_image], cur_filename.c_str());
          image_dst = &train_data->image_data[cur_training_image * num_pix_downs];       
          label_dst = &train_data->label_data[cur_training_image * num_pix_downs];
          cur_training_image++;
        }
        loadProcessedDepthLabel(directory + cur_filename, image_dst, label_dst);
      }
    }

    // Double check that we allocated the correct number of images (and that we
    // didn't mess up our book keeping).
    if (static_cast<int32_t>(cur_training_image) != train_data->num_images ||
      static_cast<int32_t>(cur_test_image) != test_data->num_images) {
        throw wruntime_error("LoadDepthImagesFromDirectory - something"
          "went wrong.  The number of training and test images are not"
          " what we expected!");
    }
  }

  void DepthImagesIO::releaseImages(DepthImageData*& data) {
    for (int32_t i = 0; i < data->num_images; i++) {
      delete[] data->filenames[i];
    }
    if (data->filenames) {
      delete[] data->filenames;
    }
    if (data->image_data) {
      delete[] data->image_data;
    }
    if (data->label_data) {
      delete[] data->label_data;
    }
    if (data->rgb_data) {
      delete[] data->rgb_data;
    }
    delete data;
    data = NULL;
  }

  float DepthImagesIO::round(const float num) {
    return (num > 0.0f) ? floor(num + 0.5f) : ceil(num - 0.5f);
  }

  void DepthImagesIO::LoadCompressedImage(const string& file, 
    int16_t* depth_data, uint8_t* label_data, uint8_t* rgb_data) {
    std::ifstream in_file(file.c_str(), std::ios::in | std::ios::binary | 
      std::ios::ate);
    if (!in_file.is_open()) {
      throw std::runtime_error(std::string("LoadDepthImage()") + 
        std::string(": error opening file") + file);
    }

    uint32_t size_bytes = static_cast<uint32_t>(in_file.tellg());
    in_file.seekg (0, std::ios::beg);  // Go to the beginning of the file
    in_file.read(reinterpret_cast<char*>(compressed_data), size_bytes);
    in_file.close();

    int size_decompress = fastlz_decompress(reinterpret_cast<void*>(compressed_data),
      size_bytes,
      reinterpret_cast<void*>(uncompressed_data),
      data_size * 2);
    if (size_decompress != data_size) {
      throw wruntime_error(string("LoadDepthImage() - ERROR: ") +
        string("uncompressed data size is not what we expected!"));
    }

    // Extract the user pixels and copy over the depth
    memset(user_pixels, 0, src_dim * sizeof(user_pixels[0]));
    for (uint32_t i = 0; i < src_dim; i++) {
      if ((uncompressed_data[i] & 0x8000) != 0) {
        // Part of the user
        uncompressed_data[i] = uncompressed_data[i] & 0x7fff;
        user_pixels[i] = 1;
      }
    }
    memcpy(depth_data, uncompressed_data, src_dim * sizeof(depth_data[0]));

    // Now we need to process the data to find the hand points
    memcpy(label_data, user_pixels, src_dim * sizeof(label_data[0]));

    rgb = reinterpret_cast<uint8_t*>(&uncompressed_data[src_dim]);
    if (rgb_data != NULL) {
      memcpy(rgb_data, rgb, 3 * src_dim * sizeof(rgb_data[0]));
    }

    for (uint32_t i = 0; i < src_dim; i++) {
      if (depth_data[i] > GDT_MAX_DIST || depth_data[i] == 0) {
        depth_data[i] = GDT_MAX_DIST + 1; // Push to background
      }
    }
  }

  void DepthImagesIO::LoadCompressedImageWithRedHands(const string& file, 
    int16_t* depth_data, uint8_t* label_data, uint8_t* rgb_data, 
    uint8_t* red_pixels_ret, uint8_t* hsv_pixels_ret) {
    // try loading a already processed one:

    LoadCompressedImage(file, depth_data, label_data, rgb_data);

    // Now we need to process the data to find the hand points
    memset(label_data, 0, src_dim * sizeof(label_data[0]));

    convertRGBToHSV<uint8_t>(hsv, rgb, src_width, src_height);  
    if (hsv_pixels_ret != NULL) {
      memcpy(hsv_pixels_ret, hsv, 3 * src_dim * sizeof(hsv_pixels_ret[0]));
    }

    getRedPixels(rgb, hsv, red_pixels);
    // Run an aggressive median filter to remove outliers
    GrowFilter<uint8_t>(red_pixels_tmp, red_pixels, src_width,
      src_height, 1);
    MedianBoolFilter<uint8_t>(red_pixels, red_pixels_tmp, src_width,
      src_height, red_med_filter_rad, 1);
    //memcpy(red_pixels, red_pixels_tmp, sizeof(red_pixels[0])*src_dim);
    if (red_pixels_ret != NULL) {
      memcpy(red_pixels_ret, red_pixels, src_dim * sizeof(red_pixels_ret[0]));
    }
    cleanUpRedPixelsUsingDepth(depth_data, red_pixels);

    // Now use the red pixels as seed points for a floodfill in the depth image
    findHandPoints(label_data_tmp, red_pixels, depth_data);

    // Since the RGB doesn't line up with the depth we need to allow the hand
    // points to grow.  This also cleans up any bad fill behaviour from the HSV.
    growHandPoints(label_data, label_data_tmp, depth_data, HAND_PTS_GROW_RAD);
  }

  bool DepthImagesIO::saveProcessedDepthLabel(const std::string& file, 
    const int16_t* depth_data, const uint8_t* label_data) {

    // Get the image data ready for compressing
    int16_t* depth_dst = (int16_t*)uncompressed_data;
    memcpy(depth_dst, depth_data, src_dim * sizeof(depth_dst[0]));

    for (uint32_t i = 0; i < src_dim; i++) {
      if (label_data[i] == 1) {
        depth_dst[i] *= -1;
      }
    }

    // Now compress it
    static const int compression_level = 1;  // 1 fast, 2 better compression
    int compressed_length = fastlz_compress_level(compression_level, 
      reinterpret_cast<void*>(uncompressed_data), 
      src_dim * sizeof(depth_dst[0]),
      reinterpret_cast<void*>(compressed_data));

    // Seperate out the directory string and the filename
    string name_dir, name_file;
    extractDirFile(file, name_dir, name_file);
    if (name_file.substr(0, 10) != string("processed_")) {
      name_file = string("processed_") + name_file;
    }
    string full_filename = name_dir + name_file;

    // Now save the array to file
    std::cout << "Saving " << full_filename << " to file" << std::endl;
    std::ofstream ofile(full_filename.c_str(), std::ios::out | std::ios::binary);
    if (!ofile.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + full_filename);
    }
    ofile.write(reinterpret_cast<const char*>(compressed_data), compressed_length);
    ofile.flush();
    ofile.close();
    return true;
  }

  void DepthImagesIO::extractDirFile(const string& full_dir_filename, 
    string& dir, string& file) {
    uint32_t cur_char = (uint32_t)full_dir_filename.length()-1;
    while (cur_char > 0 && full_dir_filename.at(cur_char) != '\\' &&
      full_dir_filename.at(cur_char) != '/') {
      cur_char--;
    }
    cur_char++;
    dir = full_dir_filename.substr(0, cur_char);
    file = full_dir_filename.substr(cur_char, full_dir_filename.length()-cur_char);
  }

  bool DepthImagesIO::loadProcessedDepthLabel(const std::string& file, 
    int16_t* depth_data, uint8_t* label_data) {

    // Seperate out the directory string and the filename
    string name_dir, name_file;
    extractDirFile(file, name_dir, name_file);
    if (name_file.substr(0, 10) != string("processed_")) {
      name_file = string("processed_") + name_file;
    }
    string full_filename = name_dir + name_file;

    std::ifstream in_file(full_filename.c_str(), 
      std::ios::in | std::ios::binary | std::ios::ate);
    if (!in_file.is_open()) {
      return false;
    }

    uint32_t size_bytes = static_cast<uint32_t>(in_file.tellg());
    in_file.seekg (0, std::ios::beg);  // Go to the beginning of the file
    in_file.read(reinterpret_cast<char*>(compressed_data), size_bytes);
    in_file.close();

    int size_decompress = fastlz_decompress(reinterpret_cast<void*>(compressed_data),
      size_bytes, (void*)(uncompressed_data), processed_data_size * 2);
    if (size_decompress != (src_dim * sizeof(depth_data[0]))) {
      throw wruntime_error(string("ERROR: uncompressed data") +
        string(" size is not what we expected!"));
    }

    // Copy the data into user space
    int16_t* depth_file = (int16_t*)uncompressed_data;
    memcpy(depth_data, depth_file, src_dim * sizeof(depth_data[0]));

    memset(label_data, 0, src_dim * sizeof(label_data[0]));
    for (uint32_t i = 0; i < src_dim; i++) {
      if (depth_data[i] < 0) {
        label_data[i] = 1;
        depth_data[i] *= -1;
      } 
    }

    return true;
  }

  void DepthImagesIO::LoadRGBImage(const string& file, uint8_t* rgb) {
    std::ifstream in_file(file.c_str(), 
      std::ios::in | std::ios::binary | std::ios::ate);
    if (!in_file.is_open()) {
      throw std::runtime_error(std::string("LoadRGBImage()") + 
        std::string(": error opening file"));
    }

    uint32_t size_bytes = static_cast<uint32_t>(in_file.tellg());
    in_file.seekg (0, std::ios::beg);  // Go to the beginning of the file
    in_file.read(reinterpret_cast<char*>(compressed_data), size_bytes);
    in_file.close();

    int size_decompress = fastlz_decompress(reinterpret_cast<void*>(compressed_data),
      size_bytes,
      reinterpret_cast<void*>(uncompressed_data),
      data_size * 2);
    if (size_decompress != data_size) {
      throw wruntime_error(string("LoadRGBImage() - ERROR: uncompressed data") +
        string(" size is not what we expected!"));
    }

    uint8_t* rgb_file = reinterpret_cast<uint8_t*>(&uncompressed_data[src_dim]);
    memcpy(rgb, rgb_file, src_dim * sizeof(rgb[0]) * 3);
  }

  uint32_t DepthImagesIO::GetFilesInDirectory(
    jtil::data_str::VectorManaged<char*>& files_in_directory, 
    const string& directory, const bool load_processed_images) {
#if defined(WIN32) || defined(_WIN32)
    // Prepare string for use with FindFile functions.  First, copy the
    // string to a buffer, then append '\*' to the directory name.
    TCHAR szDir[MAX_PATH];
    StringCchCopy(szDir, MAX_PATH, 
      jtil::string_util::ToWideString(directory).c_str());
    if (!load_processed_images) {
      StringCchCat(szDir, MAX_PATH, TEXT("\\hands_*.bin"));
    } else {
      StringCchCat(szDir, MAX_PATH, TEXT("\\processed_hands_*.bin"));
    }

    // Find the first file in the directory.
    WIN32_FIND_DATA ffd;
    HANDLE hFind = FindFirstFile(szDir, &ffd);
    if (hFind == INVALID_HANDLE_VALUE) {
      throw wruntime_error(string("GetFilesInDirectory error ") +
        string("getting dir info. Check that directory is not empty!"));
    }

    do {
      if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
      } else {
        std::string cur_filename = 
          jtil::string_util::ToNarrowString(std::wstring(ffd.cFileName));
        char* name = new char[cur_filename.length() + 1];
        strcpy(name, cur_filename.c_str());
        files_in_directory.pushBack(name);
      }
    } while (FindNextFile(hFind, &ffd) != 0);
    FindClose(hFind);
#else
      string name_preamble;
      uint32_t preamble_length;
      if (!load_processed_images) {
        name_preamble = string("hands_");
        preamble_length = 6;
      } else {
        name_preamble = string("processed_hands_");
        preamble_length = 16;
      }
      static unsigned char isFile =0x8;
      static unsigned char isFolder =0x4;
      struct dirent *dp;
      DIR *dfd = opendir(directory.c_str());
      if(dfd != NULL) {
        while((dp = readdir(dfd)) != NULL) {
          std::string cur_name = string(dp->d_name);
          if (dp->d_type == isFile) {
            if (cur_name.length() > 10) {
              if (cur_name.substr(0,preamble_length) == name_preamble && 
                cur_name.substr(cur_name.length()-4,4) == string(".bin")) {
                  char* name = new char[cur_name.length() + 1];
                  strcpy(name, cur_name.c_str());
                  files_in_directory.pushBack(name);
              }
            }
          } else if (dp->d_type == isFolder) {
            std::string cur_foldername = string(dp->d_name);
            // std::cout << "Folder in directory: " << cur_foldername << std::endl;
          }
        }
        closedir(dfd);
      } else {
        std::stringstream ss;
        ss << "GetFilesInDirectory error getting dir info for dir: ";
        ss << directory << std::endl;
        throw std::wruntime_error(ss.str());
      }

#endif
      return files_in_directory.size();
  };

  // testRedPixel - Single "red-test" of a hsv+rgb pixel (for debugging only)
  void DepthImagesIO::testRedPixel(uint32_t index, uint8_t* hsv, uint8_t* rgb) {
    int32_t cur_delta[3];
    int32_t hue_offset = (red_hue_target + 128) % 255;

    cur_delta[0] = (static_cast<int32_t>(hsv[index*3]) + hue_offset) % 255;
    cur_delta[1] = static_cast<int32_t>(hsv[index*3+1]);
    cur_delta[2] = static_cast<int32_t>(hsv[index*3+2]);

    cur_delta[0] = cur_delta[0] - 128;
    cur_delta[1] = cur_delta[1] - red_sat_target;
    cur_delta[2] = cur_delta[2] - red_val_target;

    cur_delta[0] = abs(cur_delta[0]);
    cur_delta[1] = abs(cur_delta[1]);
    cur_delta[2] = abs(cur_delta[2]);

    if (cur_delta[0] >= red_hue_threshold) {
      std::cout << "pixel hue out of range." << std::endl;
    }
    if (cur_delta[1] >= red_sat_threshold) {
      std::cout << "pixel sat out of range." << std::endl;
    }
    if (cur_delta[2] >= red_val_threshold) {
      std::cout << "pixel red val out of range." << std::endl;
    }
    if (rgb[index*3] <= red_red_min) {
      std::cout << "pixel Red bellow min level." << std::endl;
    }
    if ((hsv[index*3] + hsv[index*3+1] + hsv[index*3+2]) < hsv_total_threshold) {
      std::cout << "HSV Total out of range." << std::endl;
    }
    if (rgb[index*3+2] >= red_blue_max) {
      std::cout << "pixel blue above max level." << std::endl;
    }
    if (rgb[index*3+1] >= red_green_max) {
      std::cout << "pixel gren above max level." << std::endl;
    }
  }

  void DepthImagesIO::getRedPixels(uint8_t* rgb, uint8_t* hsv, 
    uint8_t* red_pixels) {
    // Search for pixels that are near <1, 1, 1> in HSV space and which
    // belong to the user
    int32_t cur_delta[3];
    int32_t hue_offset = (red_hue_target + 128) % 255;

    memset(red_pixels, 0, src_dim * sizeof(red_pixels[0]));
    uint32_t index = 0;
    // uint32_t index_offset;
    for (uint32_t v = 0; v < src_height; v++) {
      for (uint32_t u = 0; u < src_width; u++) {
        cur_delta[0] = (static_cast<int32_t>(hsv[index*3]) + hue_offset) % 255;
        cur_delta[1] = static_cast<int32_t>(hsv[index*3+1]);
        cur_delta[2] = static_cast<int32_t>(hsv[index*3+2]);

        cur_delta[0] = cur_delta[0] - 128;
        cur_delta[1] = cur_delta[1] - red_sat_target;
        cur_delta[2] = cur_delta[2] - red_val_target;

        cur_delta[0] = abs(cur_delta[0]);
        cur_delta[1] = abs(cur_delta[1]);
        cur_delta[2] = abs(cur_delta[2]);

        if ((cur_delta[0] < red_hue_threshold && 
             cur_delta[1] < red_sat_threshold && 
             cur_delta[2] < red_val_threshold && 
             rgb[index*3] > red_red_min && 
             rgb[index*3+2] < red_blue_max &&
             rgb[index*3+1] < red_green_max) || 
             ((hsv[index*3] + hsv[index*3+1] + hsv[index*3+2]) >= hsv_total_threshold)) {
          red_pixels[index] = 1;
        }
        index++;
      }
    }
  };

  void DepthImagesIO::cleanUpRedPixelsUsingDepth(int16_t* depth_data, 
    uint8_t* red_pixels) {

    //std::cout << "cleanUpRedPixelsUsingDepth() HACK ON" << std::endl;
    //for (uint32_t i = 0; i < src_dim; i++) {
    //  if (depth_data[i] >= 1500) {
    //    red_pixels[i] = 0;
    //    depth_data[i] = GDT_MAX_DIST + 1;
    //  }
    //}

    // Get rid of zero or large depths
    for (uint32_t i = 0; i < src_dim; i++) {
      if (depth_data[i] == 0 || depth_data[i] >= GDT_MAX_DIST) {
        red_pixels[i] = 0;
      }
    }

    // Now perform a shrink operation of a N pixel radius in-case depth and RGB
    // pixels don't overlap
    memcpy(red_pixels_tmp, red_pixels, src_dim * sizeof(red_pixels_tmp[0]));
    if (red_shrink_filter_rad > 0) {
      // Shrink horizontally
      int32_t index = 0;
      for (int32_t v = 0; v < src_height; v++) {
        for (int32_t u = 0; u < src_width; u++) {
          if (red_pixels_tmp[index] == 0) {
            for (int32_t u_offset = u - red_shrink_filter_rad; 
              u_offset <= u + red_shrink_filter_rad; u_offset++) {
                if (u_offset < src_width && u_offset >= 0) {
                  red_pixels[v * src_width + u_offset] = 0;
                }
            }
          }
          index++;
        }
      }
      // Shrink vertically
      index = 0;
      for (int32_t v = 0; v < src_height; v++) {
        for (int32_t u = 0; u < src_width; u++) {
          if (red_pixels[index] == 0) {
            for (int32_t v_offset = v - red_shrink_filter_rad; 
              v_offset <= v + red_shrink_filter_rad; v_offset++) {
                if (v_offset < src_height && v_offset >= 0) {
                  red_pixels_tmp[v_offset * src_width + u] = 0;
                }
            }
          }
          index++;
        }
      }
    }

    // Filter out any pixels that are near a discontinuity
    if (red_discon_filter_rad > 0) {
      int16_t cur_depth_min;
      int16_t cur_depth_max;
      uint32_t index = 0;
      for (int32_t v = 0; v < src_height; v++) {
        for (int32_t u = 0; u < src_width; u++) {
          cur_depth_min = GDT_MAX_DIST;
          cur_depth_max = 0;
          if (red_pixels_tmp[index] == 1) {
            for (int32_t v_offset = v - red_discon_filter_rad; 
              v_offset <= v + red_discon_filter_rad; v_offset++) {
                for (int32_t u_offset = u - red_discon_filter_rad; 
                  u_offset <= u + red_discon_filter_rad; u_offset++) {   
                    int32_t index_offset = v_offset * src_width + u_offset;
                    if (depth_data[index_offset] > cur_depth_max) {
                      cur_depth_max = depth_data[index_offset];
                    }
                    if (depth_data[index_offset] < cur_depth_min) {
                      cur_depth_min = depth_data[index_offset];
                    }         
                }
            }
            if ((cur_depth_max - cur_depth_min) > RED_DISCONT_FILT_DEPTH_THRESH) {
              red_pixels_tmp[index] = 0;
            }
          }
          index++;
        }
      }
    }

    memcpy(red_pixels, red_pixels_tmp, src_dim * sizeof(red_pixels[0]));

    //// Get rid of any blobs below a threshold number of pixels
    //uint32_t blob_pixel;
    //uint32_t blob_size;
    //resetBlobDetection();
    //while (findNextBlob(blob_pixel, blob_size, depth_data, red_pixels)) {
    //  if (blob_size < 100) {
    //    zeroBlob(blob_pixel, depth_data, red_pixels);
    //  }
    //}
  };

  void DepthImagesIO::resetBlobDetection() {
    memset(pixel_on_queue, false, src_dim * sizeof(pixel_on_queue[0]));
    queue_head = 0;
    queue_tail = 0;  // When queue_head_ == queue_tail_ the queue is empty
    blob_i = 0;
  }

  bool DepthImagesIO::findNextBlob(uint32_t& blob_index, uint32_t& blob_size,
    int16_t* depth_data, uint8_t* labels) {
    blob_size = 0;
    int neighbourPtIndexUV[2];
    while (blob_i < src_dim) {
      if (!pixel_on_queue[blob_i] && labels[blob_i] > 0) {
         // We haven't yet searched from this pixel.  Put it on the queue and 
        // flood fill from it.
        pixel_on_queue[blob_i] = true;
        pixel_queue[queue_tail] = blob_i;
        queue_tail++;

        while (queue_head != queue_tail) {
          // Take the current pixel off the queue
          int curPtIndex = pixel_queue[queue_head];
          int curPtU = curPtIndex % src_width;
          int curPtV = curPtIndex / src_width; 
          queue_head++;

          // Incrument the blob size
          blob_size++;

          for (int i = 0; i < N_PTS_FILL; i ++) {
            neighbourPtIndexUV[0] = curPtU + floodFillKernel_[i][0];
            neighbourPtIndexUV[1] = curPtV + floodFillKernel_[i][1];
            processBlobNeighbour(depth_data, neighbourPtIndexUV, curPtIndex,
              labels);
          }
        }
      }
      if (blob_size > 0) {
        blob_index = blob_i;
        blob_i++;
        return true;
      }
      blob_i++;
    }
    return false;
  }

  void DepthImagesIO::processBlobNeighbour(int16_t* depth_data, 
    int* nieghbourPtUV, int curPtIndex, uint8_t* labels) {
    if (nieghbourPtUV[0] >= 0 && nieghbourPtUV[0] < src_width && 
      nieghbourPtUV[1] >= 0 && nieghbourPtUV[1] < src_height) {
      int nieghbourPtIndex = nieghbourPtUV[1]*src_width + nieghbourPtUV[0];
      if (!pixel_on_queue[nieghbourPtIndex]) {  // don't add it if we have
        if (depth_data[nieghbourPtIndex] > 0 && labels[nieghbourPtIndex]) {
          // See if the pixel is close to the current point (otherwise it's the 
          // background)
          int16_t delta_depth = (depth_data[nieghbourPtIndex] - 
            depth_data[curPtIndex]);
          delta_depth = delta_depth >= 0 ? delta_depth : -delta_depth;  // abs

          if (delta_depth < BACKGROUND_DEPTH_THRESH*3) {
            pixel_queue[queue_tail] = nieghbourPtIndex;
            pixel_on_queue[nieghbourPtIndex] = true;
            queue_tail++;
          }
        }
      }
    }
  }

  void DepthImagesIO::zeroBlob(const uint32_t blob_index, int16_t* depth_data, 
    uint8_t* labels) {
    memset(pixel_on_queue, false, src_dim * sizeof(pixel_on_queue[0]));
    queue_head = 0;
    queue_tail = 0;  // When queue_head_ == queue_tail_ the queue is empty
    pixel_on_queue[blob_index] = true;
    pixel_queue[queue_tail] = blob_index;
    queue_tail++;

    int neighbourPtIndexUV[2];
    while (queue_head != queue_tail) {
      // Take the current pixel off the queue
      int curPtIndex = pixel_queue[queue_head];
      int curPtU = curPtIndex % src_width;
      int curPtV = curPtIndex / src_width; 
      queue_head++;

      // Zero the pixel label
      labels[curPtIndex] = 0;

      for (int i = 0; i < N_PTS_FILL; i ++) {
        neighbourPtIndexUV[0] = curPtU + floodFillKernel_[i][0];
        neighbourPtIndexUV[1] = curPtV + floodFillKernel_[i][1];
        processBlobNeighbour(depth_data, neighbourPtIndexUV, curPtIndex,
          labels);
      }
    }
  }

  // findHandPoints - Idea, with RGB image converted to HSV space and
  // all pixels within some offset of "red" (already calculated), use the "red"
  // points to seed floodfills on the depth image that uses BOTH depth and HSV
  // value as the per-pixel floodfill test.  That is only include the
  // neighboring pixel if it is within some depth threshold and if it is roughly
  // the same color.
  void DepthImagesIO::findHandPoints(uint8_t* label_data, uint8_t* red_pixels, 
    int16_t* depth_data) {
    memset(label_data, 0, src_dim * sizeof(label_data[0]));
    memset(pixel_on_queue, false, src_dim * sizeof(pixel_on_queue[0]));
    queue_head = 0;
    queue_tail = 0;  // When queue_head_ == queue_tail_ the queue is empty
    int neighbourPtIndexUV[2];
    for (uint32_t i = 0; i < src_dim; i++) {
      if (!pixel_on_queue[i] && (red_pixels[i] != 0) && (depth_data[i] > 0)) {
        // We haven't yet searched from this pixel.  Put it on the queue and 
        // flood fill from it.
        pixel_on_queue[i] = true;
        pixel_queue[queue_tail] = i;
        queue_tail++;
        while (queue_head != queue_tail) {
          // Take the current pixel off the queue
          int curPtIndex = pixel_queue[queue_head];
          int curPtU = curPtIndex % src_width;
          int curPtV = curPtIndex / src_width; 
          queue_head++;

          // Add the point to the set
          label_data[curPtIndex] = 1;

          for (int i = 0; i < N_PTS_FILL; i ++) {
            neighbourPtIndexUV[0] = curPtU + floodFillKernel_[i][0];
            neighbourPtIndexUV[1] = curPtV + floodFillKernel_[i][1];
            processGloveNeighbour(depth_data, neighbourPtIndexUV, curPtIndex);
          }
        }
      }
    }
  }

  void DepthImagesIO::processGloveNeighbour(int16_t* depth_data, 
    int* nieghbourPtUV, int curPtIndex) {
      if (nieghbourPtUV[0] >= 0 && nieghbourPtUV[0] < src_width && 
        nieghbourPtUV[1] >= 0 && nieghbourPtUV[1] < src_height) {
          int nieghbourPtIndex = nieghbourPtUV[1]*src_width + nieghbourPtUV[0];
          if (!pixel_on_queue[nieghbourPtIndex]) {  // don't add it if we have
            if (depth_data[nieghbourPtIndex] > 0) {
              // See if the pixel is close to the current point (otherwise it's the 
              // background)
              int16_t delta_depth = (depth_data[nieghbourPtIndex] - 
                depth_data[curPtIndex]);
              delta_depth = delta_depth >= 0 ? delta_depth : -delta_depth;  // abs

              delta_hsv[0] = (static_cast<int32_t>(hsv[nieghbourPtIndex*3]) - 
                static_cast<int32_t>(hsv[curPtIndex*3]));
              delta_hsv[1] = (static_cast<int32_t>(hsv[nieghbourPtIndex*3 + 1]) - 
                static_cast<int32_t>(hsv[curPtIndex*3 + 1]));        
              delta_hsv[2] = (static_cast<int32_t>(hsv[nieghbourPtIndex*3 + 2]) - 
                static_cast<int32_t>(hsv[curPtIndex*3 + 2]));               
              // int32_t len2_sq = (delta_hsv[0] * delta_hsv[0] + 
              //                    delta_hsv[1] * delta_hsv[1] + 
              //                    delta_hsv[2] * delta_hsv[2]);    
              // int32_t len2 = (abs(delta_hsv[0]) + abs(delta_hsv[1]) + abs(delta_hsv[2]));
              int32_t len2 = (abs(delta_hsv[0]) + abs(delta_hsv[1]));

              if (delta_depth < BACKGROUND_DEPTH_THRESH && 
                len2 < BACKGROUND_HSV_THRESH) {
                  pixel_queue[queue_tail] = nieghbourPtIndex;
                  pixel_on_queue[nieghbourPtIndex] = true;
                  queue_tail++;
              }
            }
          }
      }
  }

  // This is very expensive!
  void DepthImagesIO::growHandPoints(uint8_t* new_label_data, 
    uint8_t* label_data, int16_t* depth_data, float radius) {
    int32_t cur_iteration;
    for (cur_iteration = 0; cur_iteration < hand_pts_grow_rad_iterations; cur_iteration++) {
      if ((cur_iteration % 2) == 1) {
        uint8_t* temp = new_label_data;
        new_label_data = label_data;
        label_data = temp;
      }
      for (int32_t v = 0; v < src_height; v++) {
        for (int32_t u = 0; u < src_width; u++) {
          int32_t index = v * src_width + u;
          if (label_data[index] == 1) {
            int16_t cur_depth = depth_data[index];
            int16_t cur_rad = static_cast<int16_t>(round(radius / static_cast<float>(cur_depth)));
            if (cur_rad == 0) {
              cur_rad = 1;
            }
            // Grow the current label if the depth is close
            for (int32_t v_offset = v - cur_rad; v_offset <= v + cur_rad; v_offset++) {
              if (v_offset >= 0 && v_offset < src_height) {
                for (int32_t u_offset = u - cur_rad; u_offset <= u + cur_rad; u_offset++) {
                  if (u_offset >= 0 && u_offset < src_width) {
                    int32_t index_offset = v_offset * src_width + u_offset;
                    int16_t delta_depth = (depth_data[index_offset] - cur_depth);
                    delta_depth = delta_depth < 0 ? -delta_depth : delta_depth;  // abs
                    if ((index_offset == index || delta_depth < BACKGROUND_DEPTH_THRESH_GROW) && 
                      (depth_data[index_offset] != 0 && depth_data[index_offset] < GDT_MAX_DIST)) {
                        new_label_data[index_offset] = 1;
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
    if ((cur_iteration % 2) == 0) {
      memcpy(new_label_data, label_data, src_dim*sizeof(new_label_data[0]));
    }
  }
  
  void DepthImagesIO::floodPixel(uint8_t* label_image, int16_t* depth_image, 
    int u, int v, int32_t radius, int16_t threshold) {
    flood_threshold = threshold;
    memset(pixel_on_queue, false, src_dim * sizeof(pixel_on_queue[0]));
    queue_head = 0;
    queue_tail = 0;  // When queue_head_ == queue_tail_ the queue is empty
    int neighbourPtIndexUV[2];
    uint32_t start_index = v * src_width + u;
    // Start a flood fill from the input pixel
    uint8_t label_to_flood = label_image[start_index] == 1 ? 0 : 1;
    pixel_on_queue[start_index] = true;
    pixel_queue[queue_tail] = start_index;
    queue_tail++;
    while (queue_head != queue_tail) {
      // Take the current pixel off the queue
      int curPtIndex = pixel_queue[queue_head];
      int curPtU = curPtIndex % src_width;
      int curPtV = curPtIndex / src_width; 
      queue_head++;
      int cur_rad_sq = (u - curPtU)*(u - curPtU) + (v - curPtV)*(v - curPtV);
      if (cur_rad_sq <= radius) {
        // Add the point to the set
        label_image[curPtIndex] = label_to_flood;

        for (int i = 0; i < N_PTS_FILL; i ++) {
          neighbourPtIndexUV[0] = curPtU + floodFillKernel_[i][0];
          neighbourPtIndexUV[1] = curPtV + floodFillKernel_[i][1];
          processFloodPixelNeighbour(depth_image, label_image, neighbourPtIndexUV, 
            curPtIndex, label_to_flood);
        }
      }
    }
  }

  void DepthImagesIO::processFloodPixelNeighbour(int16_t* depth_data, 
    uint8_t* label_data, int* nieghbourPtUV, int curPtIndex, 
    uint8_t label_to_flood) {
    if (nieghbourPtUV[0] >= 0 && nieghbourPtUV[0] < src_width && 
      nieghbourPtUV[1] >= 0 && nieghbourPtUV[1] < src_height) {
        int nieghbourPtIndex = (nieghbourPtUV[1]*src_width) + 
          nieghbourPtUV[0];
        if (!pixel_on_queue[nieghbourPtIndex]) {  // don't add it if we have
          if (depth_data[nieghbourPtIndex] > 0 && 
            label_data[nieghbourPtIndex] != label_to_flood) {
              // See if the pixel is close to the current point (otherwise it's the 
              // background)
              int16_t delta_depth = (depth_data[nieghbourPtIndex] - 
                depth_data[curPtIndex]);
              delta_depth = delta_depth >= 0 ? delta_depth : -delta_depth;  // abs
              if (delta_depth < flood_threshold) {
                pixel_queue[queue_tail] = nieghbourPtIndex;
                pixel_on_queue[nieghbourPtIndex] = true;
                queue_tail++;
              }
          }
        }
    }
  }

  void DepthImagesIO::convertImageDepthToXYZ(float*& xyz, DepthImageData*& images) {
    float* uvd_data = new float[src_dim * 3];

    for (int32_t i = 0; i < images->num_images; i++) {
      int16_t* image_src = &images->image_data[src_dim * i];
      OpenNIFuncs::ConvertDepthImageToProjective(
        reinterpret_cast<uint16_t*>(image_src), uvd_data);

      float* xyz_dest = &xyz[src_dim * i * 3];
      OpenNIFuncs::xnConvertProjectiveToRealWorld(src_dim, uvd_data, xyz_dest);
    }
    delete[] uvd_data;
  }

  void DepthImagesIO::convertSingleImageToXYZ(float* xyz, int16_t* depth) {
    float* uvd_data = new float[src_dim * 3];
    OpenNIFuncs::ConvertDepthImageToProjective((uint16_t*)depth, uvd_data);
    OpenNIFuncs::xnConvertProjectiveToRealWorld(src_dim, uvd_data, xyz);
    delete[] uvd_data;
  }

  // ************* NO LONGER USING GRAPH CUT *************
  //void DepthImagesIO::AddVertex(GraphFloat* graph, uint32_t cur_ind, 
  //  int* label_count) {
  //  vert[cur_ind] = graph->add_node();
  //  float D_k0;  // Background edge weight
  //  float D_k1;  // Forground edge weight

  //  if (label_count[cur_ind] == 0) {
  //    D_k0 = 0;
  //    D_k1 = std::numeric_limits<float>::infinity();
  //  } else if (label_count[cur_ind] == affiliation_cnt) {
  //    D_k0 = std::numeric_limits<float>::infinity();
  //    D_k1 = 0;
  //  } else {
  //    // D_k1 and D_k0 \in 0->1
  //    D_k1 = static_cast<float>(label_count[cur_ind]) / affiliation_cnt * 0.6f;
  //    D_k0 = 1.0f - D_k1;  // 0->1
  //    D_k1 = -log10f(D_k1);
  //    D_k0 = -log10f(D_k0);
  //  }

  //  graph->add_tweights(vert[cur_ind], D_k0, D_k1);
  //}

  //// Adjacency edge weight is: 
  //// E_a = gamma * max(abs(delta_depth) / max_depth, 1)^2
  //float DepthImagesIO::CalculateAdjacentEdgeWeight(int ind0, int ind1, 
  //  int16_t* image_data) {
  //  float delta_depth = static_cast<float>(image_data[ind1] - image_data[ind0]);

  //  float delta_depth_sq = delta_depth * delta_depth;
  //  float E_a = adjacency_gamma * exp(-delta_depth_sq * adjacency_beta);

  //  return E_a;
  //}

  //void DepthImagesIO::graphErrorFunc(const char* err) {
  //  throw std::runtime_error(string("graphErrorFunc() - ERROR: ") + 
  //    string(err));
  //}

  //void DepthImagesIO::cleanUpSegmentWithGraphCut(uint8_t* return_label_data,
  //  int16_t* image_data, uint8_t* red_label_data) {
  //  // First, for each pixel count the number of 1 labels within some radius
  //  // But we need overflow, so we must do this with int precision
  //  for (uint32_t i = 0; i < src_dim; i++) {
  //    label_data_int[i] = static_cast<int>(red_label_data[i]);
  //  }
  //  IntegrateBooleanLabel<int>(label_data_int_integ, label_data_int_tmp,
  //                             label_data_int, src_width, src_height, graph_cut_affiliation_radius, 1);
  //  
  //  affiliation_diam = static_cast<float>(graph_cut_affiliation_radius * 2 + 1);
  //  affiliation_cnt = affiliation_diam * affiliation_diam;

  //  // For now delete the graph every frame.
  //  // This will be slow --> but Graph does not have the ability to remove the edge!
  //  if(im_graph != NULL) { 
  //    delete im_graph; 
  //  }
  //  static const int num_nodes = src_dim;
  //  static const int num_edges = (src_width - 1) * (src_height - 1) * 2 + // cross edges
  //                               (src_height - 1) * src_width + // vertical edges
  //                               (src_width - 1) * src_height; // horizontal edges
  //  im_graph = new GraphFloat(num_nodes, num_edges, graphErrorFunc);

  //  // Add all pixels

  //  // Iterate through the vertices and add nodes if we need them
  //  // bool addNode;
  //  uint32_t curInd, neighbourInd;
  //  float edgeWeight;
  //  // Add all the vertices
  //  for (uint32_t curY = 0; curY < src_height; curY ++) {
  //    for (uint32_t curX = 0; curX < src_width; curX ++) {
  //      curInd = curY * src_width + curX;
  //      AddVertex(im_graph, curInd, label_data_int_integ);
  //    }
  //  }   
  //  // Add all the edges
  //  for (uint32_t curY = 0; curY < src_height; curY ++) {
  //    for (uint32_t curX = 0; curX < src_width; curX ++) {
  //      curInd = curY * src_width + curX;

  //      // Add it's right edge
  //      if(curX < (src_width-1)) {
  //        neighbourInd = curY * src_width + curX + 1;
  //        edgeWeight = CalculateAdjacentEdgeWeight(curInd, neighbourInd, image_data);
  //        im_graph->add_edge(vert[curInd],vert[neighbourInd], edgeWeight, edgeWeight);
  //      } // end if (curX < (width-1))

  //      // Add it's down edge
  //      if(curY < (src_height-1)) {
  //        neighbourInd = (curY + 1) * src_width + curX;
  //        edgeWeight = CalculateAdjacentEdgeWeight(curInd, neighbourInd, image_data);
  //        im_graph->add_edge(vert[curInd],vert[neighbourInd], edgeWeight, edgeWeight);
  //      } // end if (curY < (height-1))

  //      // Add it's bottom-right edge
  //      if(curX < (src_width-1) && curY < (src_height-1)) {
  //        neighbourInd = (curY + 1) * src_width + curX + 1;
  //        edgeWeight = CalculateAdjacentEdgeWeight(curInd, neighbourInd, image_data);
  //        im_graph->add_edge(vert[curInd],vert[neighbourInd], edgeWeight, edgeWeight);
  //      } // end if (curX < (width-1))

  //      // Add it's bottom-left edge
  //      if(curX > 0 && curY < (src_height-1)) {
  //        neighbourInd = (curY + 1) * src_width + curX -1;
  //        edgeWeight = CalculateAdjacentEdgeWeight(curInd, neighbourInd, image_data);
  //        im_graph->add_edge(vert[curInd],vert[neighbourInd], edgeWeight, edgeWeight);
  //      } // end if (curY < (height-1))
  //    }
  //  }

  //  // Now do the graph cut
  //  float flow = im_graph->maxflow();

  //  // Now go through the graph and copy the assignments
  //  for(int i = 0; i < src_dim; i ++ ) {
  //    if (im_graph->what_segment(vert[i]) == GraphFloat::SOURCE) {
  //      return_label_data[i] = 1;
  //    } else {
  //      return_label_data[i] = 0;
  //    }
  //  }
  //}

}  // namespace depth_images_io
