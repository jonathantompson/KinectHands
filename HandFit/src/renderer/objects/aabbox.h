//
//  aabbox.h
//
//  Created by Jonathan Tompson on 6/24/12.
//

#ifndef RENDERER_OBJECTS_AABBOX_HEADER
#define RENDERER_OBJECTS_AABBOX_HEADER

#include "math/math_types.h"

namespace data_str { template <typename T> class Vector; }

namespace renderer {
  namespace objects {
    class AABBox {
    public:
      AABBox();
      ~AABBox();

      void init(data_str::Vector<math::Float3>* vertices);
      void update(math::Float4x4* mat_world);

      inline math::Float3* min_bounds() { return &min_; }
      inline math::Float3* max_bounds() { return &max_; }
      inline math::Float3* center() { return &center_; }
      inline math::Float3* half_lengths() { return &half_lengths_; }

    private:
      math::Float3 min_;
      math::Float3 max_;
      math::Float3 object_bounds_[8];
      math::Float3 world_bounds_[8];
      math::Float3 center_;
      math::Float3 half_lengths_;

      // Expand(), check current min/max value against input vector and set new 
      // min/max if appropriate
      void expand(math::Float3* vec);
    };
  };  // namespace objects
};  // namespace renderer

#endif  // RENDERER_OBJECTS_AABBOX_HEADER
