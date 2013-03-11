//
//  debug_util_win32.cpp
//
//  Created by Jonathan Tompson on 6/2/12.
//
//  A layer for interfacing with visual studio and XCode's debug utilities
//

#include "debug_util.h"

namespace debug {

  void EnableMemoryLeakChecks() {
#ifdef _DEBUG
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif
  }

  void SetBreakPointOnAlocation(int alloc_num) {
#ifdef _DEBUG
    _CrtSetBreakAlloc(alloc_num);
#endif
  }

}  // namespace debug
