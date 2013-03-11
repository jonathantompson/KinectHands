//
//  string_util.cpp
//
//  Created by Jonathan Tompson on 5/1/12.
//

#ifdef _WIN32
#include <Windows.h>
#include <strsafe.h>
#else
#include <vector>
#endif

#include <string>  // for string
#include "string_util/string_util.h"

namespace string_util {

  /*
#ifdef _WIN32
  std::wstring Win32ErrorToString(DWORD dwErrorCode) {
    // Retrieve the system error message for the last-error code

    LPVOID lpMsgBuf;
    LPVOID lpDisplayBuf;
    DWORD dw = GetLastError(); 

    FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | 
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        dw,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR) &lpMsgBuf,
        0, NULL);

    // Display the error message and return in
    lpDisplayBuf = (LPVOID)LocalAlloc(LMEM_ZEROINIT, 
        (lstrlen((LPCTSTR)lpMsgBuf) + 
        lstrlen((LPCTSTR)dwErrorCode) + 40) * sizeof(TCHAR)); 
    StringCchPrintf((LPTSTR)lpDisplayBuf, 
        LocalSize(lpDisplayBuf) / sizeof(TCHAR),
        TEXT("%s failed with error %d: %s"), 
        dwErrorCode, dw, lpMsgBuf); 
    // MessageBox(NULL, (LPCTSTR)lpDisplayBuf, TEXT("Error"), MB_OK); 

    std::wstring strRet((LPTSTR)lpDisplayBuf);

    LocalFree(lpMsgBuf);
    LocalFree(lpDisplayBuf);
    return strRet;
  }
#endif
  */

  std::wstring ToWideString(const char* pStr , int len = -1) {
    static_cast<void>(len);
#ifdef _WIN32
    // figure out how many wide characters we are going to get 
    int nChars = MultiByteToWideChar(CP_ACP , 0 , pStr , len , NULL , 0); 
    if (len == -1) {
      --nChars; 
    }
    if (nChars == 0) {
      return L"";
    }

    // convert the narrow string to a wide string 
    // nb: slightly naughty to write directly into the string like this
    std::wstring buf;
    buf.resize(nChars); 
    MultiByteToWideChar(CP_ACP , 0 , pStr , len , 
      const_cast<wchar_t*>(buf.c_str()) , nChars); 

    return buf;
# else
    const size_t wn = std::mbsrtowcs(NULL, &pStr, 0, NULL);
    if (wn == size_t(-1)) {
      throw std::wruntime_error(L"Error in mbsrtowcs()");
    }
    
    std::vector<wchar_t> buf(wn + 1);
    const size_t wn_again = std::mbsrtowcs(buf.data(), &pStr, wn + 1, NULL);
    
    if (wn_again == size_t(-1)) {
      throw std::wruntime_error(L"Error in mbsrtowcs()");
    }
    
    return std::wstring(buf.data(), wn);
#endif
  }

  std::wstring ToWideString(const std::string& str) {
    return ToWideString(str.c_str(), -1);
  }

  std::string ToNarrowString(const wchar_t* pStr , int len = -1) {
    static_cast<void>(len);
#ifdef _WIN32    
    // figure out how many narrow characters we are going to get 
    int nChars = WideCharToMultiByte(CP_ACP , 0 , pStr , len , NULL , 0 , 
      NULL , NULL); 
    if (len == -1) {
      --nChars; 
    }
    if (nChars == 0) {
      return "";
    }

    // convert the wide string to a narrow string
    // nb: slightly naughty to write directly into the string like this
    std::string buf;
    buf.resize(nChars);
    WideCharToMultiByte(CP_ACP , 0 , pStr , len , 
      const_cast<char*>(buf.c_str()) , nChars , NULL , NULL); 

    return buf; 
#else
    const size_t wn = std::wcsrtombs(NULL, &pStr, 0, NULL);
    if (wn == size_t(-1)) {
      throw std::wruntime_error(L"Error in wcsrtombs()");
    }
    
    std::vector<char> buf(wn + 1);
    const size_t wn_again = std::wcsrtombs(buf.data(), &pStr, wn + 1, NULL);
    
    if (wn_again == size_t(-1)) {
      throw std::wruntime_error(L"Error in wcsrtombs()");
    }
    
    return std::string(buf.data(), wn);
#endif
  }

  std::string ToNarrowString(const std::wstring& str) {
    return ToNarrowString(str.c_str(), -1);
  }

}  // namespace string_util
