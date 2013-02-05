//
//  window.h
//
//  Created by Jonathan Tompson on 4/26/12.
//
//  Pure virtual class interface for cross platform window API
//  as well as a convenient cross platform error box method

#ifndef WINDOWING_WINDOW_HEADER
#define WINDOWING_WINDOW_HEADER

#ifdef _WIN32
#include <Windows.h>
typedef HWND WindowHandleType; 
#else
// #error "Windowing system not yet implemented for Mac OS X"
#endif

namespace windowing {

  void ErrorBox(const char* str);
};  // unnamed windowing

#endif  // WINDOWING_WINDOW_HEADER
