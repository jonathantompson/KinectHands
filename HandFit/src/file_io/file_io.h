//
//  file_io.h
//  KinectHands
//
//  Created by Jonathan Tompson on 6/5/12.
//

#ifndef FILE_IO_FILE_IO_HEADER
#define FILE_IO_FILE_IO_HEADER

#include <stdexcept>
#include <iostream>
#include <fstream>
#include "math/math_types.h"

namespace file_io {
  std::string GetHomePath();
  
  template <class T>
  void SaveArrayToFile(const T* arr, uint32_t size, const std::string& filename) {
    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error(std::string("file_io::SaveArrayToFile()") + 
                               std::string(": error opening file:") + filename);
    }
    file.write(reinterpret_cast<const char*>(arr), size * sizeof(arr[0]));
    file.flush();
    file.close();
  }
  
};  // namespace file_io

#endif  // FILE_IO_FILE_IO_HEADER
