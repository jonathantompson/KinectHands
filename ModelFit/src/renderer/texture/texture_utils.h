//
//  texture_utils.h
//
//  Created by Jonathan Tompson on 10/7/12.
//
//  Just a few simple utility functions

#ifndef RENDERER_TEXURE_UTILS_HEADER
#define RENDERER_TEXURE_UTILS_HEADER

#include "renderer/open_gl_common.h"
#include "jtil/math/math_types.h"

namespace renderer {
  uint32_t ElementSizeOfGLType(GLint gl_type);
  uint32_t NumElementsOfGLFormat(GLint gl_format);
};  // namespace renderer

#endif  // RENDERER_TEXURE_UTILS_HEADER