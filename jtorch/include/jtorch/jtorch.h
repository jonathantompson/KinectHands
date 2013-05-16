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

#include <string>

namespace jcl { class JCL; }

namespace jtorch {

  // path_to_jtorch should be the path to "KinectHands/jtorch"
  void InitJTorch(const std::string& path_to_jtorch);  // Throws exception on multiple init
  void InitJTorchSafe(const std::string& path_to_jtorch);  // Multiple init OK
  void ShutdownJTorch();

  // Some constants and globals for the jtorch instance
  extern jcl::JCL* cl_context;
  extern std::string jtorch_path;
  const int max_local_workgroup_size = 8;
  const uint32_t deviceid = 0;

};  // namespace jtorch

#endif  // JTORCH_JTORCH_HEADER
