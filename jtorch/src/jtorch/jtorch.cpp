#include <mutex>
#include "jcl/jcl.h"
#include "jtorch/jtorch.h"
#include "jtil/exceptions/wruntime_error.h"

#define SAFE_DELETE(x) if (x != NULL) { delete x; x = NULL; }
#define SAFE_DELETE_ARR(x) if (x != NULL) { delete[] x; x = NULL; }

namespace jtorch {

  jcl::JCL* cl_context = NULL;
  std::mutex cl_context_lock_;
  std::string jtorch_path;

  void InitJTorchInternal(const std::string& path_to_jtorch, 
    const bool use_cpu) {
    if (use_cpu) {
      cl_context = new jcl::JCL(jcl::CLDeviceCPU, jcl::CLVendorAny);
    } else {
      cl_context = new jcl::JCL(jcl::CLDeviceGPU, jcl::CLVendorAny);
    }
    jtorch_path = path_to_jtorch;
    if (jtorch_path.at(jtorch_path.size()-1) != '\\' && 
      jtorch_path.at(jtorch_path.size()-1) != '/') {
      jtorch_path = jtorch_path + '/';
    }
  }

  void InitJTorch(const std::string& path_to_jtorch, const bool use_cpu) {
    std::lock_guard<std::mutex> lck(cl_context_lock_);
    if (cl_context != NULL) {
      throw std::wruntime_error("jtorch::InitJTorch() - ERROR: Init called "
        "twice!");
    }
    InitJTorchInternal(path_to_jtorch, use_cpu);
  }

  void InitJTorchSafe(const std::string& path_to_jtorch, const bool use_cpu) {
    std::lock_guard<std::mutex> lck(cl_context_lock_);
    if (cl_context != NULL) {
      return;
    }
    InitJTorchInternal(path_to_jtorch, use_cpu);
  }

  void ShutdownJTorch() {
    std::lock_guard<std::mutex> lck(cl_context_lock_);
    SAFE_DELETE(cl_context);
  }

}  // namespace jtorch