//
//  file_io.cpp
//  KinectHands
//
//  Created by Jonathan Tompson on 6/5/12.
//

#include <iostream>
#include "file_io/file_io.h"
#include <stdio.h>
#include <stdlib.h>

namespace file_io {
  
#ifdef __APPLE__
  std::string GetHomePath() {
    return getenv("HOME") + std::string("/");
  }
#endif
#ifdef _WIN32
  std::string GetHomePath() {
    return std::string("");  // empty string for now
  }
#endif  
  
}  // namespace file_io
