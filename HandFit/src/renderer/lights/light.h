//
//  light.h
//
//  Created by Jonathan Tompson on 6/15/12.
//

#ifndef RENDERER_LIGHT_HEADER
#define RENDERER_LIGHT_HEADER

#include "math/math_types.h"

namespace renderer {

  typedef enum {
    LIGHT_BASE,
    LIGHT_SPOT,
    LIGHT_SPOT_SM,
    LIGHT_DIR,
    LIGHT_POINT,
  } LightType;

  class Light {
  public:
    Light();
    virtual ~Light();

    inline virtual LightType type() { return LIGHT_BASE; }

    inline virtual void update() { }

  protected:

    // Non-copyable, non-assignable.
    Light(Light&);
    Light& operator=(const Light&);
  };

};  // namespace renderer

#endif  // RENDERER_LIGHT_HEADER
