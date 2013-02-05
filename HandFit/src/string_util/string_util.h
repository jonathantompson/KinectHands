//
//  string_util.h
//
//  Created by Jonathan Tompson on 5/1/12.
//

#ifndef STRING_UTIL_STRING_UTIL_HEADER
#define STRING_UTIL_STRING_UTIL_HEADER

#ifdef _WIN32
  #include <windows.h>
#endif
#include <string>
#include <sstream>
#include "exceptions/wruntime_error.h"

namespace string_util {
#ifdef _WIN32
  std::wstring Win32ErrorToString(DWORD dwErrorCode);
#endif

  // Convert a string to a number
  template <class T>
  T Str2Num(const std::string& s) {
    std::istringstream stream(s);
    T t;
    stream >> t;
    return t; 
  };

  // Convert a number to a string
  template <class T>
  std::string Num2Str(const T num) {
    std::stringstream stream;
    stream << num;
    return stream.str();
  };

  std::wstring ToWideString(const char* pStr, int len);
  std::wstring ToWideString(const std::string& str);
  std::string ToNarrowString(const wchar_t* pStr, int len);
  std::string ToNarrowString(const std::wstring& str);
  
};  // namespace string_util

#ifdef _WIN32
#define ER_CHECK_WIN32(expr) \
  if ((expr) == 0) { \
    throw std::wruntime_error(string_util::Win32ErrorToString(GetLastError()));\
  }
#endif

#endif  // STRING_UTIL_STRING_UTIL_HEADER
