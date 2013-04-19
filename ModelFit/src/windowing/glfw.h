//
//  glfw.h
//
//  Created by Jonathan Tompson on 6/7/12.
//

#ifndef GLOBAL_HEADER
#define GLOBAL_HEADER

#ifndef GLEW_STATIC
  #define GLEW_STATIC
#endif
#include "jtil/glew/glew.h"

// Get rid of annoying macro redefinition warnings
#ifdef _WIN32
  #include <windows.h>
#endif

#define GL3_PROTOTYPES 1  // Ensure we're using OpenGL's core profile only
#include <GL/glfw.h>

#endif  // GLOBAL_HEADER
