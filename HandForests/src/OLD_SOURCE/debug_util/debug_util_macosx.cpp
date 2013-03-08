//
//  debug_util_macosx.cpp
//
//  Created by Jonathan Tompson on 6/2/12.
//
//  A layer for interfacing with visual studio and XCode's debug utilities
//

#include <stdio.h>
#include "debug_util.h"

namespace debug {

  void EnableMemoryLeakChecks() {
    printf("WARNING: EnableMemoryLeakChecks not implemented yet for Mac\n");
  }

  void SetBreakPointOnAlocation(int alloc_num) {
    static_cast<void>(alloc_num);
    printf("WARNING: SetBreakPointOnAlocation not implemented yet for Mac\n");
  }

}  // namespace debug
