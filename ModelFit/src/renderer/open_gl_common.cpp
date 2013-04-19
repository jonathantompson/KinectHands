#include <sstream>
#include <string>
#include "jtil/string_util/string_util.h"
#include "jtil/exceptions/wruntime_error.h"
#include "renderer/open_gl_common.h"

namespace renderer {
  void CheckOpenGLError() {
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
      std::wstringstream ss;
      ss << L"Renderer::checkOpenGLError() - ERROR: '";
      ss << jtil::string_util::ToWideString(
        reinterpret_cast<const char *>(gluErrorString(err)));
      ss << L"'";
      throw std::wruntime_error(ss.str());
    }
  }
  
};  // namespace renderer

