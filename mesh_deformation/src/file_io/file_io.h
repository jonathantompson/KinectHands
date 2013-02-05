//
//  file_io.h
//  KinectHands
//
//  Created by Jonathan Tompson on 6/5/12.
//

#ifndef FILE_IO_FILE_IO_HEADER
#define FILE_IO_FILE_IO_HEADER

#include <iostream>
#include <fstream>
#include "math/math_types.h"

namespace file_io {
  std::string GetHomePath();
  
  template <class T>
  void SaveArrayToFile(const T* arr, uint32_t size, const std::string& filename) {
#ifdef _WIN32
    std::string full_filename = GetHomePath() + filename;
#endif
#ifdef __APPLE__
    std::string full_filename = GetHomePath() + std::string("Desktop/") + 
                                filename;
#endif    
    std::ofstream file(full_filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("file_io::SaveArrayToFile()") + 
                               std::string(": error opening file:") + full_filename);
    }
    file.write(reinterpret_cast<const char*>(arr), size * sizeof(arr[0]));
    file.flush();
    file.close();
  }
  
  template <class T>
  void LoadArrayFromFile(T** arr, uint32_t* size, const std::string& filename) {
#ifdef _WIN32
    std::string full_filename = GetHomePath() + filename;
#endif
#ifdef __APPLE__
    std::string full_filename = GetHomePath() + std::string("Desktop/") + 
    filename;
#endif    
    // ios::ate -> Postion read pointer at end of file (so we can read the size)
    std::ifstream file(full_filename.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("file_io::LoadArrayFromFile()") + 
                               std::string(": error opening file:") + full_filename);
    }
    
    std::ifstream::pos_type size_bytes = file.tellg();
    (*size) = size_bytes / sizeof(float);
    (*arr) = new float[*size];
    file.seekg (0, std::ios::beg);  // Go to the beginning of the file
    file.read (reinterpret_cast<char*>(*arr), size_bytes);
    file.close();
  } 
  
};  // namespace file_io

#endif  // FILE_IO_FILE_IO_HEADER
