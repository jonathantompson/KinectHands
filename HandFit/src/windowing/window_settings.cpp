#include "windowing/window_settings.h"

namespace windowing {
  WindowSettings& WindowSettings::operator=(const WindowSettings &rhs) {
    // Check for self-assignment!
    if (this == &rhs) {
      return *this;
    }
    
    this->title = rhs.title;
    this->width = rhs.width;
    this->height = rhs.height;
    this->fullscreen = rhs.fullscreen;
    
    this->num_depth_bits = rhs.num_depth_bits;
    this->num_stencil_bits = rhs.num_stencil_bits;
    this->num_rgba_bits = rhs.num_rgba_bits;
    
    this->fsaa_samples = rhs.fsaa_samples;
    this->gl_major_version = rhs.gl_major_version;
    this->gl_minor_version = rhs.gl_minor_version;
    
    return *this;
  }
  
}  // namespace windowing
