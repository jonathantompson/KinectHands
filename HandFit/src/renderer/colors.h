//
//  colors.h
//
//  Created by Jonathan Tompson on 9/12/2012.
//

#ifndef RENDERER_COLORS_HEADER
#define RENDERER_COLORS_HEADER

#include "math/math_types.h"

namespace renderer {

  extern const math::Float3 white;
  extern const math::Float3 black;
  extern const math::Float3 red;
  extern const math::Float3 lred;
  extern const math::Float3 green;
  extern const math::Float3 lgreen;
  extern const math::Float3 blue;
  extern const math::Float3 lblue;
  extern const math::Float3 yellow;
  extern const math::Float3 pink;
  extern const math::Float3 cyan;
  extern const math::Float3 grey;
  extern const math::Float3 gold;

  const uint32_t n_colors = 13;
  extern const math::Float3 colors[n_colors];

};  // renderer namespace

#endif  // RENDERER_COLORS_HEADER
