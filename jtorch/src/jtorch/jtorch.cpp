#include <mutex>
#include "jcl/jcl.h"
#include "jtorch/jtorch.h"
#include "jtil/exceptions/wruntime_error.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

namespace jtorch {

  jcl::JCL* cl_context = NULL;
  std::mutex cl_context_lock_;

  void InitJTorch() {
    std::lock_guard<std::mutex> lck(cl_context_lock_);
    if (cl_context != NULL) {
      throw std::wruntime_error("jtorch::InitJTorch() - ERROR: Init called "
        "twice!");
    }
    cl_context = new jcl::JCL(jcl::CLDeviceCPU, jcl::CLVendorAny);
  }

  void ShutdownJTorch() {
    std::lock_guard<std::mutex> lck(cl_context_lock_);
    SAFE_DELETE(cl_context);
  }

}  // namespace jtorch