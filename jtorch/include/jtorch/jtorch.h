//
//  jtorch.h
//
//  Created by Jonathan Tompson on 5/14/13.
//
//  NOTE: YOU MUST CALL jtorch::InitTorch() before using any of these functions
//  since a valid OpenCL context must exist.  Then you can only use these
//  functions in the same thread that InitTorch was called.
//
//  Call ShutdownJTorch() when finished.
//

#ifndef JTORCH_JTORCH_HEADER
#define JTORCH_JTORCH_HEADER

#include <iostream>
#include <iomanip>
#include <fstream>

namespace jcl { class JCL; }

namespace jtorch {

  void InitJTorch();
  void ShutdownJTorch();

  extern jcl::JCL* cl_context;

};  // namespace jtorch

#endif  // JTORCH_JTORCH_HEADER
