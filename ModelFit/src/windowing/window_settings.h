//
//  window_settings.h
//
//  Created by Jonathan Tompson on 09/25/12.
//

#ifndef WINDOWING_WINDOW_SETTINGS_HEADER
#define WINDOWING_WINDOW_SETTINGS_HEADER

#include <mutex>
#include <string>
#include "jtil/math/math_types.h"  // for uint32_t

namespace windowing {
  
  struct WindowSettings {
    virtual ~WindowSettings() { }

    std::string title;
    int width;
    int height;
    bool fullscreen;
    
    // Back buffer format
    int num_depth_bits;
    int num_stencil_bits;
    int num_rgba_bits;
    
    // OpenGL version
    int gl_major_version;
    int gl_minor_version;

    // FSAA samples (very expensive and kinda dumb)
    int fsaa_samples;
    
    WindowSettings& operator=(const WindowSettings &rhs);
  };

};  // namespace windowing

#endif  // WINDOWING_WINDOW_SETTINGS_HEADER

