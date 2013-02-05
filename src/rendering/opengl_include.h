#ifndef OPENGL_INCLUDE_HEADER
#define OPENGL_INCLUDE_HEADER

#if defined( __WIN32__ ) || defined( _WIN32 ) || defined( WIN32 )
  //#include <windows.h>
  #define GLEW_STATIC
  // #include <winbase.h>
  // #include <gl\glew.h>
  // #include <gl\wglew.h>
  #include <glut.h>
  // #include <glu.h>
#else
  #include <GLUT/glut.h>
#endif

#endif  // OPENGL_INCLUDE_HEADER