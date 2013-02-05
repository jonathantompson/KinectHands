//
//  window_win32.cpp
//
//  Created by Jonathan Tompson on 5/1/12.
//

#include <Windows.h>
#include <Commctrl.h>  // for InitCommonControls among others
#include <WinUser.h>  // for MAKEINTRESOURCE 
#include "windowing/window_base.h"
#include "string_util/string_util.h"

#define ID_START  1
#define ID_CANCEL 2

namespace windowing {

  void ErrorBox(const char* str) {
    // Note: this will likely throw an unhandeled exception if MessageBox
    // returns an error.  But an unhandled exception is better than nothing.
    InitCommonControls();
    MessageBox(NULL, (LPCTSTR)str, TEXT("Error"), MB_OK | MB_ICONERROR);
  } 

}  // namespace windowing
