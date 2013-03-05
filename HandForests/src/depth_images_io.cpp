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
#include "math/math_types.h"
#include "depth_images_io.h"
#include "generate_decision_tree.h"
#include "image_util.h"
#include "string_util/string_util.h"
#include "data_str/vector_managed.h"
#include "fastlz/fastlz.h"
#include "open_ni_funcs.h"

using std::string;
using std::runtime_error;
using data_str::VectorManaged;

#define SAFE_FREE(x) do { if (x != NULL) { free(x); x = NULL; } } while (0); 
#define SAFE_DELETE(x) do { if (x != NULL) { delete x; x = NULL; } } while (0); 

namespace depth_images_io {

  uint32_t DepthImagesIO::graph_cut_affiliation_radius = 5;
  float DepthImagesIO::adjacency_gamma = 25.0f;
  float DepthImagesIO::adjacency_beta = 0.25f;
  float DepthImagesIO::sink_source_beta = 5.0f;

  int32_t DepthImagesIO::red_hue_threshold = 27;
  int32_t DepthImagesIO::red_sat_threshold = 48;  // 40
  int32_t DepthImagesIO::red_val_threshold = 79;
  int32_t DepthImagesIO::red_hue_target = 250;
  int32_t DepthImagesIO::red_sat_target = 214;  // 225
  int32_t DepthImagesIO::red_val_target = 161;
  int32_t DepthImagesIO::red_red_min = 95;
  int32_t DepthImagesIO::red_blue_max = 100;
  int32_t DepthImagesIO::hsv_total_threshold = 660;

  // Previous Values (11 Jan)
  //int32_t DepthImagesIO::red_hue_threshold = 32;
  //int32_t DepthImagesIO::red_sat_threshold = 35;
  //int32_t DepthImagesIO::red_val_threshold = 60;
  //int32_t DepthImagesIO::red_hue_target = 250;
  //int32_t DepthImagesIO::red_sat_target = 240;
  //int32_t DepthImagesIO::red_val_target = 128;
  //int32_t DepthImagesIO::red_red_min = 40;

  // Previous Values (25 Oct)
  //int32_t DepthImagesIO::red_hue_threshold = 32;
  //int32_t DepthImagesIO::red_sat_threshold = 16;
  //int32_t DepthImagesIO::red_val_threshold = 60;
  //int32_t DepthImagesIO::red_hue_target = 250;
  //int32_t DepthImagesIO::red_sat_target = 250;
  //int32_t DepthImagesIO::red_val_target = 128;
  //int32_t DepthImagesIO::red_red_min = 40;

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
    im_graph = NULL;
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
    SAFE_FREE(cur_image_data);
    SAFE_FREE(cur_label_data);
    SAFE_DELETE(im_graph);
  }

  /*  // NEEDS UPDATING --> ONLY LOAD PROCESSED IMAGES NOW.
  // LoadDepthImagesFromDirectory - Top level function
  // This version will split the data into training data and test data, as well
  // as downsample the depth image as desired.
  void LoadDepthImagesFromDirectoryForDT(std::string directory,
    ImageData*& training_data,
    ImageData*& test_data,
    float frac_test_data,
    uint32_t file_skip,
    bool load_processed_images) {
      if (data_size == 0) {
        init_image_io();
      }

      VectorManaged<char*> files_in_directory;
      uint32_t num_files = GetFilesInDirectory(files_in_directory, directory, load_processed_images);
      if (num_files == 0) {
        throw std::runtime_error("ERROR: no files in the database!\n");
      }
      std::cout << "  --> Total number of files in the database = " << num_files << std::endl;
      num_files = num_files / file_skip;
      std::cout << "  --> Using " << num_files << " of these files:" << std::endl;

      test_data = new ImageData;
      training_data = new ImageData;

      uint32_t stride_test_data = static_cast<uint32_t>(round(1.0f / frac_test_data));
      test_data->num_images = static_cast<uint32_t>(ceil(static_cast<float>(num_files) / 
        static_cast<float>(stride_test_data)));
      training_data->num_images = num_files - test_data->num_images;

      bool load_training_data = test_data->num_images != 0;
      if (!load_training_data) {
        std::cout << "LoadDepthImagesFromDirectory - Warning, not reserving any images ";
        std::cout << "for the test set!"  << std::endl << "  frac_test_data is too low!" << std::endl;
      }

      // Allocate enough space for the images:
      uint32_t num_pixels = src_dim;
      uint32_t num_pixels_downsampled = num_pixels / (DT_DOWNSAMPLE*DT_DOWNSAMPLE);

      if (src_width % DT_DOWNSAMPLE != 0 || src_height % DT_DOWNSAMPLE != 0) {
        throw runtime_error(string("LoadDepthImagesFromDirectory downsample") +
          string(" factor must be an integer multiple"));
      }

      training_data->image_data = new int16_t[num_pixels_downsampled * training_data->num_images];
      training_data->label_data = new uint8_t[num_pixels_downsampled * training_data->num_images];
      training_data->im_width = src_width / DT_DOWNSAMPLE;
      training_data->im_height = src_height / DT_DOWNSAMPLE;
      training_data->filenames = new char*[training_data->num_images];
      training_data->rgb_data = NULL;

      if (load_training_data) {
        test_data->image_data = new int16_t[num_pixels_downsampled * test_data->num_images];
        test_data->label_data = new uint8_t[num_pixels_downsampled * test_data->num_images];
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
        std::string cur_filename = string(*files_in_directory.at(i*file_skip));
        if (DT_DOWNSAMPLE > 1) {
          if (load_training_data && i % stride_test_data == 0) {
            test_data->filenames[cur_test_image] = new char[cur_filename.length() + 1];
            strcpy(test_data->filenames[cur_test_image], cur_filename.c_str());
            image_dst = &test_data->image_data[cur_test_image * num_pixels_downsampled];
            label_dst = &test_data->label_data[cur_test_image * num_pixels_downsampled];
            cur_test_image++;
          } else {
            training_data->filenames[cur_training_image] = new char[cur_filename.length() + 1];
            strcpy(training_data->filenames[cur_training_image], cur_filename.c_str());
            image_dst = &training_data->image_data[cur_training_image * num_pixels_downsampled];
            label_dst = &training_data->label_data[cur_training_image * num_pixels_downsampled];
            cur_training_image++;
          }

          LoadDepthImageForDT(directory + cur_filename, cur_image_data,
            cur_label_data, load_processed_images);
          // Downsample but ignore 0 or background pixel values when filtering
          DownsampleImageWithoutNonZeroPixelsAndBackground<int16_t>(image_dst, 
            cur_image_data, src_width, src_height, DT_DOWNSAMPLE);
          DownsampleLabelImageWithoutNonZeroPixelsAndBackground<uint8_t>(label_dst, 
            cur_label_data, src_width, src_height, DT_DOWNSAMPLE);

        } else {
          if (load_training_data && i % stride_test_data == 0) {
            test_data->filenames[cur_test_image] = new char[cur_filename.length() + 1];
            strcpy(test_data->filenames[cur_test_image], cur_filename.c_str());
            image_dst = &test_data->image_data[cur_test_image * num_pixels_downsampled];       
            label_dst = &test_data->label_data[cur_test_image * num_pixels_downsampled];
            cur_test_image++;
          } else {
            training_data->filenames[cur_training_image] = new char[cur_filename.length() + 1];
            strcpy(training_data->filenames[cur_training_image], cur_filename.c_str());
            image_dst = &training_data->image_data[cur_training_image * num_pixels_downsampled];       
            label_dst = &training_data->label_data[cur_training_image * num_pixels_downsampled];
            cur_training_image++;
          }
          LoadDepthImageForDT(directory + cur_filename, image_dst, label_dst,
            load_processed_images);
        }
      }

      // Double check that we allocated the correct number of images (and that we
      // didn't mess up our book keeping).
      if (static_cast<int32_t>(cur_training_image) != training_data->num_images ||
        static_cast<int32_t>(cur_test_image) != test_data->num_images) {
          throw runtime_error(string("LoadDepthImagesFromDirectory - something") +
            string("went wrong.  The number of training and test images are not") + 
            string(" what we expected!"));
      }
  }
  */

  void DepthImagesIO::releaseImages(ImageData*& data) {
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

  float round(float num) {
    return (num > 0.0f) ? floor(num + 0.5f) : ceil(num - 0.5f);
  }

  void DepthImagesIO::LoadCompressedImage(string file, int16_t* depth_data, 
    uint8_t* label_data, uint8_t* rgb_data) {
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
      throw runtime_error(string("LoadDepthImage() - ERROR: ") +
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

    // Now segment based on the union of the user pixels and the hand pixels
    // Push everything else into the background
    // EDIT: NO LONGER DOING THIS!  MAKE SURE YOU RECORD WITHOUT ANYTHING IN
    //       THE BACKGROUND.
    //for (uint32_t i = 0; i < src_dim; i++) {
    //  if (user_pixels[i] == 0 && label_data[i] == 0) {
    //    depth_data[i] = GDT_MAX_DIST + 1; // Push to background
    //  }
    //}  

    for (uint32_t i = 0; i < src_dim; i++) {
      if (depth_data[i] > GDT_MAX_DIST || depth_data[i] == 0) {
        depth_data[i] = GDT_MAX_DIST + 1; // Push to background
      }
    }
  }

  void DepthImagesIO::LoadCompressedImageWithRedHands(string file, 
    int16_t* depth_data, uint8_t* label_data, uint8_t* rgb_data, 
    uint8_t* red_pixels_ret, uint8_t* hsv_pixels_ret) {
    // try loading a already processed one:

    bool ret = LoadProcessedCompressedImageWithRedHands(file, depth_data, 
      label_data, rgb_data, red_pixels, hsv_pixels_ret);
    if (ret) {
      return;
    }

    LoadCompressedImage(file, depth_data, label_data, rgb_data);

    // Now we need to process the data to find the hand points
    memset(label_data, 0, src_dim * sizeof(label_data[0]));

    convertRGBToHSV<uint8_t>(hsv, rgb, src_width, src_height);  
    if (hsv_pixels_ret != NULL) {
      memcpy(hsv_pixels_ret, hsv, 3 * src_dim * sizeof(hsv_pixels_ret[0]));
    }

    getRedPixels(rgb, hsv, red_pixels);
    // Run an aggressive median filter to remove outliers
    MedianBoolFilter<uint8_t>(red_pixels_tmp, red_pixels, src_width,
      src_height, RED_MED_FILT_RAD, 1);
    memcpy(red_pixels, red_pixels_tmp, sizeof(red_pixels[0])*src_dim);
    if (red_pixels_ret != NULL) {
      memcpy(red_pixels_ret, red_pixels, src_dim * sizeof(red_pixels_ret[0]));
    }
    cleanUpRedPixelsUsingDepth(depth_data, red_pixels);

    // Now use the red pixels as seed points for a floodfill in the depth image
    findHandPoints(label_data_tmp, red_pixels, depth_data);

    // Since the RGB doesn't line up with the depth we need to allow the hand
    // points to grow.  This also cleans up any bad fill behaviour from the HSV.
    growHandPoints(label_data, label_data_tmp, depth_data, HAND_PTS_GROW_RAD);

    // Save the data to disk:
    //SaveProcessedCompressedImageWithRedHands(file, depth_data, label_data,
    //  rgb, red_pixels, hsv);
  }

  void DepthImagesIO::SaveProcessedCompressedImageWithRedHands(string file, 
    int16_t* depth_data, uint8_t* label_data, uint8_t* rgb_data, 
    uint8_t* red_pixels, uint8_t* hsv_pixels) {

    // Get the image data ready for compressing
    int16_t* depth_dst = (int16_t*)uncompressed_data;
    memcpy(depth_dst, depth_data, src_dim * sizeof(depth_dst[0]));

    uint8_t* rgb_dst = (uint8_t*)&depth_dst[src_dim];
    memcpy(rgb_dst, rgb_data, src_dim * sizeof(rgb_dst[0]) * 3);

    uint8_t* hsv_dst = &rgb_dst[src_dim * 3];
    memcpy(hsv_dst, hsv_pixels, src_dim * sizeof(hsv_dst[0]) * 3);

    uint8_t* red_pixels_dst = &hsv_dst[src_dim * 3];
    memcpy(red_pixels_dst, red_pixels, src_dim * sizeof(red_pixels_dst[0]));

    uint8_t* label_data_dst = &red_pixels_dst[src_dim];
    memcpy(label_data_dst, label_data, src_dim * sizeof(label_data_dst[0]));

    // Now compress it
    static const int compression_level = 1;  // 1 fast, 2 better compression
    int compressed_length = fastlz_compress_level(compression_level, 
      reinterpret_cast<void*>(uncompressed_data), 
      processed_data_size,
      reinterpret_cast<void*>(compressed_data));

    // Seperate out the directory string and the filename
    uint32_t cur_char = (uint32_t)file.length()-1;
    while (cur_char > 0 && file.at(cur_char) != '\\' &&
      file.at(cur_char) != '/') {
      cur_char--;
    }
    cur_char++;
    string name_dir = file.substr(0, cur_char);
    string name_file = file.substr(cur_char, file.length()-cur_char);
    string full_filename = name_dir + string("processed_") + name_file;

    // Now save the array to file
    std::cout << "Saving " << full_filename << " to file" << std::endl;
    std::ofstream ofile(full_filename.c_str(), std::ios::out | std::ios::binary);
    if (!ofile.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + full_filename);
    }
    ofile.write(reinterpret_cast<const char*>(compressed_data), compressed_length);
    ofile.flush();
    ofile.close();
  }

  bool DepthImagesIO::LoadProcessedCompressedImageWithRedHands(string file, 
    int16_t* depth_data, uint8_t* label_data, uint8_t* rgb_data, 
    uint8_t* red_pixels_ret, uint8_t* hsv_pixels_ret) {
    // Seperate out the directory string and the filename
    uint32_t cur_char = (uint32_t)file.length()-1;
    while (cur_char > 0 && file.at(cur_char) != '\\' &&
      file.at(cur_char) != '/') {
      cur_char--;
    }
    cur_char++;
    string name_dir = file.substr(0, cur_char);
    string name_file = file.substr(cur_char, file.length()-cur_char);
    string full_filename = name_dir + string("processed_") + name_file;

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
      size_bytes,
      reinterpret_cast<void*>(uncompressed_data),
      processed_data_size * 2);
    if (size_decompress != processed_data_size) {
      throw runtime_error(string("ERROR: uncompressed data") +
        string(" size is not what we expected!"));
    }

    // Copy the data into user space
    int16_t* depth_file = (int16_t*)uncompressed_data;
    memcpy(depth_data, depth_file, src_dim * sizeof(depth_data[0]));

    uint8_t* rgb_file = reinterpret_cast<uint8_t*>(&depth_file[src_dim]);
    if (rgb_data != NULL) {
      memcpy(rgb_data, rgb_file, src_dim * sizeof(rgb[0]) * 3);
    }

    uint8_t* hsv_file = &rgb_file[src_dim * 3];
    if (hsv_pixels_ret != NULL) {
      memcpy(hsv_pixels_ret, hsv_file, src_dim * sizeof(hsv_pixels_ret[0]) * 3);
    }

    uint8_t* red_pixels_file = &hsv_file[src_dim * 3];
    if (red_pixels_ret != NULL) {
      memcpy(red_pixels_ret, red_pixels_file, src_dim * sizeof(red_pixels_ret[0]));
    }

    uint8_t* label_data_file = &red_pixels_file[src_dim];
    if (label_data != NULL) {
      memcpy(label_data, label_data_file, src_dim * sizeof(label_data[0]));
    }
    return true;
  }

  void DepthImagesIO::LoadRGBImage(string file, uint8_t* rgb) {
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
      throw runtime_error(string("LoadRGBImage() - ERROR: uncompressed data") +
        string(" size is not what we expected!"));
    }

    uint8_t* rgb_file = reinterpret_cast<uint8_t*>(&uncompressed_data[src_dim]);
    memcpy(rgb, rgb_file, src_dim * sizeof(rgb[0]) * 3);
  }

  uint32_t DepthImagesIO::GetFilesInDirectory(
    data_str::VectorManaged<char*>& files_in_directory, string directory, 
    bool load_processed_images) {
#if defined(WIN32) || defined(_WIN32)
    // Prepare string for use with FindFile functions.  First, copy the
    // string to a buffer, then append '\*' to the directory name.
    TCHAR szDir[MAX_PATH];
    StringCchCopy(szDir, MAX_PATH, directory.c_str());
    if (!load_processed_images) {
      StringCchCat(szDir, MAX_PATH, TEXT("\\hands_*.bin"));
    } else {
      StringCchCat(szDir, MAX_PATH, TEXT("\\processed_hands_*.bin"));
    }

    // Find the first file in the directory.
    WIN32_FIND_DATA ffd;
    HANDLE hFind = FindFirstFile(szDir, &ffd);
    if (hFind == INVALID_HANDLE_VALUE) {
      throw runtime_error(string("GetFilesInDirectory error ") +
        string("getting dir info. Check that directory is not empty!"));
    }

    do {
      if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
      } else {
        std::string cur_filename = string(ffd.cFileName);
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
          std::cout << "Found directory item: " << cur_name;
          std::cout << std::endl;
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
        throw runtime_error(ss.str());
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
      std::cout << "pixel val out of range." << std::endl;
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
    if ((cur_delta[0] < red_hue_threshold && 
         cur_delta[1] < red_sat_threshold && 
         cur_delta[2] < red_val_threshold && 
         rgb[index*3] > red_red_min && rgb[index*3+2] < red_blue_max) || 
        ((hsv[index*3] + hsv[index*3+1] + hsv[index*3+2]) >= hsv_total_threshold)) {
      std::cout << "Pixel is in HSV and RGB range" << std::endl;
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
             rgb[index*3] > red_red_min && rgb[index*3+2] < red_blue_max) || 
             ((hsv[index*3] + hsv[index*3+1] + hsv[index*3+2]) >= hsv_total_threshold)) {
          red_pixels[index] = 1;
        }
        index++;
      }
    }
  };

  void DepthImagesIO::cleanUpRedPixelsUsingDepth(int16_t* depth_data, 
    uint8_t* red_pixels) {

    // Get rid of zero or large depths
    for (uint32_t i = 0; i < src_dim; i++) {
      if (depth_data[i] == 0 || depth_data[i] >= GDT_MAX_DIST) {
        red_pixels[i] = 0;
      }
    }

    // Now perform a shrink operation of a N pixel radius in-case depth and RGB
    // pixels don't overlap
    memcpy(red_pixels_tmp, red_pixels, src_dim * sizeof(red_pixels_tmp[0]));
    if (RED_SHRINK_FILT_RAD > 0) {
      // Shrink horizontally
      int32_t index = 0;
      for (int32_t v = 0; v < src_height; v++) {
        for (int32_t u = 0; u < src_width; u++) {
          if (red_pixels_tmp[index] == 0) {
            for (int32_t u_offset = u - RED_SHRINK_FILT_RAD; 
              u_offset <= u + RED_SHRINK_FILT_RAD; u_offset++) {
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
            for (int32_t v_offset = v - RED_SHRINK_FILT_RAD; 
              v_offset <= v + RED_SHRINK_FILT_RAD; v_offset++) {
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
    if (RED_DISCONT_FILT_RAD > 0) {
      int16_t cur_depth_min;
      int16_t cur_depth_max;
      uint32_t index = 0;
      for (int32_t v = 0; v < src_height; v++) {
        for (int32_t u = 0; u < src_width; u++) {
          cur_depth_min = GDT_MAX_DIST;
          cur_depth_max = 0;
          if (red_pixels_tmp[index] == 1) {
            for (int32_t v_offset = v - RED_DISCONT_FILT_RAD; 
              v_offset <= v + RED_DISCONT_FILT_RAD; v_offset++) {
                for (int32_t u_offset = u - RED_DISCONT_FILT_RAD; 
                  u_offset <= u + RED_DISCONT_FILT_RAD; u_offset++) {   
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
    uint32_t cur_iteration;
    for (cur_iteration = 0; cur_iteration < HAND_PTS_GROW_RAD_ITERATIONS; cur_iteration++) {
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

  void DepthImagesIO::saveProcessedImage(std::string directory, char* filename, 
    int16_t* image_data, uint8_t* label_data, int32_t width,
    int32_t height) {
    throw runtime_error("saveProcessedImage - NEEDS UPDATING!");
    // Get the image data ready for processing
    memcpy(uncompressed_data, image_data, width * height * sizeof(cur_image_data[0]));
    for (int32_t i = 0; i < width * height; i++) {
      if (label_data[i] == 1) {
        uncompressed_data[i] |= 0x8000;
      }
    }
    static const int compression_level = 1;  // 1 fast, 2 better compression
    int compressed_length = fastlz_compress_level(compression_level, 
      reinterpret_cast<void*>(uncompressed_data), 
      width * height * sizeof(uncompressed_data[0]),
      reinterpret_cast<void*>(compressed_data));
    string cur_filename(filename);
    // Now save the array to file
    if (cur_filename.substr(0,10) != string("processed_")) {
      cur_filename = string("processed_") + cur_filename;
    }
    std::cout << "Saving " << cur_filename << " to file" << std::endl;
    string full_filename = directory + cur_filename;
    std::ofstream file(full_filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("error opening file:") + filename);
    }
    file.write(reinterpret_cast<const char*>(compressed_data), compressed_length);
    file.flush();
    file.close();
  }

  void DepthImagesIO::convertImageDepthToXYZ(float*& xyz, ImageData*& images) {
    float* uvd_data = new float[src_dim * 3];

    for (int32_t i = 0; i < images->num_images; i++) {
      int16_t* image_src = &images->image_data[src_dim * i];
      kinect::OpenNIFuncs::ConvertDepthImageToProjective(
        reinterpret_cast<uint16_t*>(image_src),
        reinterpret_cast<kinect::Vector3D*>(uvd_data));

      float* xyz_dest = &xyz[src_dim * i * 3];
      kinect::OpenNIFuncs::xnConvertProjectiveToRealWorld(src_dim,
        reinterpret_cast<kinect::Vector3D*>(uvd_data),
        reinterpret_cast<kinect::Vector3D*>(xyz_dest));
    }
    delete[] uvd_data;
  }

  void DepthImagesIO::convertSingleImageToXYZ(float* xyz, int16_t* depth) {
    float* uvd_data = new float[src_dim * 3];
    kinect::OpenNIFuncs::ConvertDepthImageToProjective(
      reinterpret_cast<uint16_t*>(depth),
      reinterpret_cast<kinect::Vector3D*>(uvd_data));
    kinect::OpenNIFuncs::xnConvertProjectiveToRealWorld(src_dim,
      reinterpret_cast<kinect::Vector3D*>(uvd_data),
      reinterpret_cast<kinect::Vector3D*>(xyz));
    delete[] uvd_data;
  }

  void DepthImagesIO::AddVertex(GraphFloat* graph, uint32_t cur_ind, 
    int* label_count) {
    vert[cur_ind] = graph->add_node();
    float D_k0;  // Background edge weight
    float D_k1;  // Forground edge weight

    if (label_count[cur_ind] == 0) {
      D_k0 = 0;
      D_k1 = std::numeric_limits<float>::infinity();
    } else if (label_count[cur_ind] == affiliation_cnt) {
      D_k0 = std::numeric_limits<float>::infinity();
      D_k1 = 0;
    } else {
      // D_k1 and D_k0 \in 0->1
      D_k1 = static_cast<float>(label_count[cur_ind]) / affiliation_cnt * 0.6f;
      D_k0 = 1.0f - D_k1;  // 0->1
      D_k1 = -log10f(D_k1);
      D_k0 = -log10f(D_k0);
    }

    graph->add_tweights(vert[cur_ind], D_k0, D_k1);
  }

  // Adjacency edge weight is: 
  // E_a = gamma * max(abs(delta_depth) / max_depth, 1)^2
  float DepthImagesIO::CalculateAdjacentEdgeWeight(int ind0, int ind1, 
    int16_t* image_data) {
    float delta_depth = static_cast<float>(image_data[ind1] - image_data[ind0]);

    float delta_depth_sq = delta_depth * delta_depth;
    float E_a = adjacency_gamma * exp(-delta_depth_sq * adjacency_beta);

    return E_a;
  }

  void DepthImagesIO::graphErrorFunc(const char* err) {
    throw std::runtime_error(string("graphErrorFunc() - ERROR: ") + 
      string(err));
  }

  void DepthImagesIO::cleanUpSegmentWithGraphCut(uint8_t* return_label_data,
    int16_t* image_data, uint8_t* red_label_data) {
    // First, for each pixel count the number of 1 labels within some radius
    // But we need overflow, so we must do this with int precision
    for (uint32_t i = 0; i < src_dim; i++) {
      label_data_int[i] = static_cast<int>(red_label_data[i]);
    }
    IntegrateBooleanLabel<int>(label_data_int_integ, label_data_int_tmp,
                               label_data_int, src_width, src_height, graph_cut_affiliation_radius, 1);
    
    affiliation_diam = static_cast<float>(graph_cut_affiliation_radius * 2 + 1);
    affiliation_cnt = affiliation_diam * affiliation_diam;

    // For now delete the graph every frame.
    // This will be slow --> but Graph does not have the ability to remove the edge!
    if(im_graph != NULL) { 
      delete im_graph; 
    }
    static const int num_nodes = src_dim;
    static const int num_edges = (src_width - 1) * (src_height - 1) * 2 + // cross edges
                                 (src_height - 1) * src_width + // vertical edges
                                 (src_width - 1) * src_height; // horizontal edges
    im_graph = new GraphFloat(num_nodes, num_edges, graphErrorFunc);

    // Add all pixels

    // Iterate through the vertices and add nodes if we need them
    // bool addNode;
    uint32_t curInd, neighbourInd;
    float edgeWeight;
    // Add all the vertices
    for (uint32_t curY = 0; curY < src_height; curY ++) {
      for (uint32_t curX = 0; curX < src_width; curX ++) {
        curInd = curY * src_width + curX;
        AddVertex(im_graph, curInd, label_data_int_integ);
      }
    }   
    // Add all the edges
    for (uint32_t curY = 0; curY < src_height; curY ++) {
      for (uint32_t curX = 0; curX < src_width; curX ++) {
        curInd = curY * src_width + curX;

        // Add it's right edge
        if(curX < (src_width-1)) {
          neighbourInd = curY * src_width + curX + 1;
          edgeWeight = CalculateAdjacentEdgeWeight(curInd, neighbourInd, image_data);
          im_graph->add_edge(vert[curInd],vert[neighbourInd], edgeWeight, edgeWeight);
        } // end if (curX < (width-1))

        // Add it's down edge
        if(curY < (src_height-1)) {
          neighbourInd = (curY + 1) * src_width + curX;
          edgeWeight = CalculateAdjacentEdgeWeight(curInd, neighbourInd, image_data);
          im_graph->add_edge(vert[curInd],vert[neighbourInd], edgeWeight, edgeWeight);
        } // end if (curY < (height-1))

        // Add it's bottom-right edge
        if(curX < (src_width-1) && curY < (src_height-1)) {
          neighbourInd = (curY + 1) * src_width + curX + 1;
          edgeWeight = CalculateAdjacentEdgeWeight(curInd, neighbourInd, image_data);
          im_graph->add_edge(vert[curInd],vert[neighbourInd], edgeWeight, edgeWeight);
        } // end if (curX < (width-1))

        // Add it's bottom-left edge
        if(curX > 0 && curY < (src_height-1)) {
          neighbourInd = (curY + 1) * src_width + curX -1;
          edgeWeight = CalculateAdjacentEdgeWeight(curInd, neighbourInd, image_data);
          im_graph->add_edge(vert[curInd],vert[neighbourInd], edgeWeight, edgeWeight);
        } // end if (curY < (height-1))
      }
    }

    // Now do the graph cut
    float flow = im_graph->maxflow();

    // Now go through the graph and copy the assignments
    for(int i = 0; i < src_dim; i ++ ) {
      if (im_graph->what_segment(vert[i]) == GraphFloat::SOURCE) {
        return_label_data[i] = 1;
      } else {
        return_label_data[i] = 0;
      }
    }
  }

}  // namespace depth_images_io
