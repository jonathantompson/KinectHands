//
//  main.cpp
//
//  Created by Jonathan Tompson on 4/26/12.
//

#if defined(_WIN32)
  #include <Windows.h>
  #include <crtdbg.h>  // for _CrtSetDbgFlag
  #include <Commctrl.h>  // for InitCommonControls among others
#endif
#include <exception>
#include "app/app.h"
#include "jtil/string_util/string_util.h"
#include "jtil/windowing/window.h"
#include "jtil/exceptions/wruntime_error.h"
#include "jtil/debug_util/debug_util.h"  // Must come last in .cpp that includes main

using jtil::windowing::NativeErrorBox;
using app::App;

void Hello()
{}

#if defined(_WIN32)
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPreviousInstance,
                   LPSTR lpcmdline, int nCmdShow) {
#else
int main(int argc, const char* argv[]) { 
#endif
#if defined(_DEBUG) || defined(DEBUG)
  jtil::debug::EnableMemoryLeakChecks();
  // jtil::debug::EnableAggressiveMemoryLeakChecks();
  // jtil::debug::SetBreakPointOnAlocation(514482);
#endif

  //// Memory leak bug!
  //std::thread th(Hello);
  //th.join();  
  //return 0;

#if defined(_WIN32)
  InitCommonControls();
#endif

  try {
    App::newApp();
    App::initApp();
    App::runApp();  // <-- Loop here
    App::killApp();
  } catch(const std::wruntime_error &e) {
    // Try closing the window.  
    // --> In fullscreen the error message might be hidden from view.
    App::killApp();
    NativeErrorBox(e.errorMsg().c_str());
    printf("%s", e.what());
    return -1;
  }

  return 0;
}  //  WinMain or main
