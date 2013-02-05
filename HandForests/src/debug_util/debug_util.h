//
//  debug_util.h
//
//  Created by Jonathan Tompson on 6/2/12.
//
//  A layer for interfacing with visual studio and XCode's debug utilities
//  
//  NOTE: This header should be included last
//

#ifndef DEBUG_UTIL_DEBUG_UTIL_HEADER
#define DEBUG_UTIL_DEBUG_UTIL_HEADER

#ifdef _DEBUG
  #ifdef _WIN32
    #include <Windows.h>
    #include <crtdbg.h>  // for _CrtSetDbgFlag
    // Unfortunately, there are some issues with VS2012 RC and the redefined 
    // malloc.  For now we need to get rid of them.
    // #define new new(_NORMAL_BLOCK, __FILE__, __LINE__)
    // #define malloc(s) _malloc_dbg(s, _NORMAL_BLOCK, __FILE__, __LINE__)
  #endif
  #ifdef _APPLE__
    #error Debug Utilities are not yet implemented for Mac OS X 
  #endif
#endif

namespace debug {

  // EnableMemoryLeakChecks - On exit report memory leaks, should be the first
  // line in main() function.
  void EnableMemoryLeakChecks();

  // SetBreakPointOnAlocation - Breakpoint once the allocation count hits 
  // alloc_num.  Set in main() function after EnableMemoryLeakChecks()
  void SetBreakPointOnAlocation(int alloc_num);

};  // namespace debug

#endif  // DEBUG_UTIL_DEBUG_UTIL_HEADER

