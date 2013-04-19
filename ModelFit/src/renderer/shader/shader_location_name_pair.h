//
//  shader_location_name_pair.h
//
//  Created by Jonathan Tompson on 9/12/12.
//

#ifndef RENDERER_SHADER_LOCATION_NAME_PAIR_HEADER
#define RENDERER_SHADER_LOCATION_NAME_PAIR_HEADER

#include <string>
#include "jtil/math/math_types.h"

namespace renderer {

  struct ShaderLocationNamePair {
    ShaderLocationNamePair(uint32_t _id, std::string _name): id(_id), 
      name(_name) { }
    int id;
    std::string name;
  };

};  // renderer namespace

#endif  // RENDERER_SHADER_LOCATION_NAME_PAIR_HEADER
