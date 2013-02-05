//
//  light_dir_handles.h
//
//  Created by Jonathan Tompson on 9/18/12.
//

#ifndef RENDERER_LIGHT_DIR_HANDLES_HEADER
#define RENDERER_LIGHT_DIR_HANDLES_HEADER

#include "renderer/open_gl_common.h"

namespace renderer {

  class ShaderProgram;
  class LightDir;
  class Renderer;

  struct LightDirHandles {
    GLint h_color;
    GLint h_ambient_intensity;
    GLint h_diffuse_intensity;
    GLint h_direction_view;
    void getHandles(ShaderProgram* sp);
    void setHandles(LightDir* light, Renderer* render);
  };

};  // renderer namespace

#endif  // RENDERER_LIGHT_DIR_HANDLES_HEADER
