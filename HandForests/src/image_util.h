//
//  image_util.h
//
//  Created by Jonathan Tompson on 7/20/12.
//

#ifndef UNNAMED_IMAGE_UTIL_HEADER
#define UNNAMED_IMAGE_UTIL_HEADER

#include <string>
#include "generate_decision_tree.h"
#include "math/math_types.h"

template <class T>
void DownsampleImage(T* dst, T* src, uint32_t width, uint32_t height,
  uint32_t downsample) {
  uint32_t width_downsample = width / downsample;
  float scale = 1.0f / static_cast<float>(downsample*downsample);
  for (uint32_t v = 0; v < height; v+= downsample) {
    for (uint32_t u = 0; u < width; u+= downsample) {
      float val = 0.0f;
      // Average over the current source pixel in a downsample*downsample rect
      for (uint32_t v_offset = 0; v_offset < downsample; v_offset++) {
         for (uint32_t u_offset = 0; u_offset < downsample; u_offset++) {
           val += static_cast<float>(src[(v + v_offset) * width + (u + u_offset)]);
        }
      }
      val = val * scale;
      dst[(v/downsample) * width_downsample + (u/downsample)] = static_cast<T>(val);
    }
  }
};


template <class T>
void DownsampleImageWithoutNonZeroPixelsAndBackground(T* dst, T* src, 
  uint32_t width, uint32_t height, uint32_t downsample) {
  uint32_t width_downsample = width / downsample;
  for (uint32_t v = 0; v < height; v+= downsample) {
    for (uint32_t u = 0; u < width; u+= downsample) {
      float val = 0.0f;
      uint32_t num_pixels = 0;
      // Average over the current source pixel in a downsample*downsample rect
      for (uint32_t v_offset = 0; v_offset < downsample; v_offset++) {
         for (uint32_t u_offset = 0; u_offset < downsample; u_offset++) {
           uint32_t ind = (v + v_offset) * width + (u + u_offset);
           if (src[ind] != 0 && src[ind] < GDT_MAX_DIST) {
             val += static_cast<float>(src[ind]);
             num_pixels++;
           }
        }
      }
      if (num_pixels > 0) {
        val = val / static_cast<float>(num_pixels);
        dst[(v/downsample) * width_downsample + (u/downsample)] = static_cast<T>(val);
      } else {
        dst[(v/downsample) * width_downsample + (u/downsample)] = GDT_MAX_DIST + 1;
      }
    }
  }
};

template <class T>
void DownsampleLabelImageWithoutNonZeroPixelsAndBackground(T* dst, T* src, 
  uint32_t width, uint32_t height, uint32_t downsample) {
  uint32_t width_downsample = width / downsample;
  for (uint32_t v = 0; v < height; v+= downsample) {
    for (uint32_t u = 0; u < width; u+= downsample) {
      T cur_label = 1;
      // If any of the surrounding pixels are 0, let this one be zero
      // --> Conservative.
      for (uint32_t v_offset = 0; v_offset < downsample && cur_label == 1; v_offset++) {
        for (uint32_t u_offset = 0; u_offset < downsample  && cur_label == 1; u_offset++) {
          uint32_t ind = (v + v_offset) * width + (u + u_offset);
          if (src[ind] == 0) {
            cur_label = 0;
          }
        }
      }
      dst[(v/downsample) * width_downsample + (u/downsample)] = cur_label;
    }
  }
};

template <class Tlabel, class Timage>
void MedianLabelFilter(Tlabel* dst_label, Tlabel* src_label, Timage* image_data, 
                       int32_t width, int32_t height, 
                       int32_t radius) {
  int32_t cur_pixel[NUM_LABELS];
  for (int32_t v = 0; v < height; v++) {
    for (int32_t u = 0; u < width; u++) {
      int32_t index = v * width + u;
      if (image_data[index] == 0 || image_data[index] >= GDT_MAX_DIST) {
        dst_label[index] = 0;
      } else {
        // Reset the running sum
        for (uint8_t j = 0; j < NUM_LABELS; j++) {
          cur_pixel[j] = 0;
        }

        // Add up the pixel labels in a radius around the current pixel
        for (int32_t v_offset = v - radius; v_offset <= v + radius; v_offset++) {
          if (v_offset >= 0 && v_offset < height) {
            for (int32_t u_offset = u - radius; u_offset <= u + radius; u_offset++) {
              if (u_offset >= 0 && u_offset < width) {
                int32_t index_offset = v_offset * width + u_offset;
                if (image_data[index_offset] != 0 && 
                    image_data[index_offset] < GDT_MAX_DIST) {
                  cur_pixel[src_label[index_offset]]++;
                }
              }
            }
          }
        }

        // Pick the highest frequency label
        int8_t max_label;
        int32_t max_label_freq = -1;
        for (int8_t i = 0; i < NUM_LABELS; i++) {
          if (cur_pixel[i] > max_label_freq) {
            max_label_freq = cur_pixel[i];
            max_label = i;
          }
        }
        dst_label[index] = max_label;
      }
    }
  };
};

// O(n) integration (radius independant!)
template <class T>
void IntegrateBooleanLabel(T* dst, T* tmp, T* src, int32_t width, int32_t height, 
                           int32_t radius, T true_value) {
  if (width < (2 * radius + 1) || height < (2 * radius + 1)) {
    throw std::runtime_error("MedianBoolFilter - Error, im size is too small");
  }
  if (radius < 1) {
    memcpy(dst, src, width*height*sizeof(dst[0]));
  }

  // Count the number of pixels horizontally
  for (int32_t v = 0; v < height; v++) {
    for (int32_t u = 0; u < width; u++) {
      int32_t index = v * width + u;
      if (u == 0) {
        tmp[index] = 0;
        for (int32_t u_offset = 0; u_offset <= radius; u_offset++) {
          int32_t index_offset = v * width + u + u_offset;
          if (src[index_offset] == true_value) {
            tmp[index]++;
          }
        }
      } else {
        tmp[index] = tmp[index - 1];
        // Subtract off the pixel 1 radius + 1 back
        int32_t u_offset = u - radius - 1;
        if (u_offset >= 0) {
          int32_t index_offset = v * width + u - radius - 1;
          if (src[index_offset] == true_value) {
            tmp[index]--;
          }
        }
        // Add in the pixel 1 radius forward
        u_offset = u + radius;
        if (u_offset < width) {
          int32_t index_offset = v * width + u + radius;
          if (src[index_offset] == true_value) {
            tmp[index]++;
          }
        }
      }
    }
  }
  // Now tmp holds the running sum of true pixels horizontally accross

  // Count the number of pixels vertically into dst
  for (int32_t v = 0; v < height; v++) {
    for (int32_t u = 0; u < width; u++) {
      int32_t index = v * width + u;
      if (v == 0) {
        dst[index] = 0;
        for (int32_t v_offset = 0; v_offset <= radius; v_offset++) {
          int32_t index_offset = (v + v_offset) * width + u;
          dst[index] += tmp[index_offset];
        }
      } else {
        dst[index] = dst[index - width];
        // Subtract off the pixel 1 radius + 1 back
        int32_t v_offset = v - radius - 1;
        if (v_offset >= 0) {
          int32_t index_offset = (v - radius - 1) * width + u;
          dst[index] -= tmp[index_offset];
        }
        // Add in the pixel 1 radius forward
        v_offset = v + radius;
        if (v_offset < height) {
          int32_t index_offset = (v + radius) * width + u;
          dst[index] += tmp[index_offset];
        }
      }
    }
  }
}
  
template <class T>
void MedianBoolFilter(T* dst, T* src, int32_t width, int32_t height, 
                      int32_t radius, T true_value) {

  if (radius > 11) {  // Conservatively guess that the input type is 8 bits only
    throw std::runtime_error("MedianBoolFilter - Error, radius too large");
  }

  // The following will destroy src (but means we don't need a temporary array)
  IntegrateBooleanLabel<T>(src, dst, src, width, height, radius, true_value);

  // Now src contains the count of true values for every square of 2*rad+1
  // Figure out if the ones were majority
  uint32_t full_square = (2 * radius + 1) * (2 * radius + 1);
  for (int32_t v = 0; v < height; v++) {
    for (int32_t u = 0; u < width; u++) {
      int32_t num_pixels = 0;
      if (v >= radius && v <= (height - radius - 1) && 
          u >= radius && u <= (width - radius - 1)) {
        num_pixels = full_square;  // Early out for most pixels
      } else {
        int32_t u_start = (u - radius) < 0 ? 0 : (u - radius);
        int32_t u_finish = (u + radius) >= width ? (width - 1) : (u + radius);
        int32_t v_start = (v - radius) < 0 ? 0 : (v - radius);
        int32_t v_finish = (v + radius) >= height ? (height - 1) : (v + radius);   
        num_pixels = ((u_finish - u_start + 1) * (v_finish - v_start + 1));
      } 
      int32_t index = v * width + u;
      if (src[index] > (num_pixels / 2)) {      
        dst[index] = 1;
      } else {
        dst[index] = 0;
      }
    }
  }
};

// Equations from here: http://en.wikipedia.org/wiki/HSL_and_HSV
// and here: http://mjijackson.com/2008/02/rgb-to-hsl-and-rgb-to-hsv-color-model-conversion-algorithms-in-javascript
template <class T> void convertRGBToHSV(T* dst, T* src, const uint32_t w, 
                                        const uint32_t h) {
  for (uint32_t pix = 0; pix < w * h; pix++) {
    float R = static_cast<float>(src[3*pix]) / 255.0f;
    float G = static_cast<float>(src[3*pix+1]) / 255.0f;
    float B = static_cast<float>(src[3*pix+2]) / 255.0f;
    
    float max = R >= G ? R : G;
    max = max >= B ? max : B;
    float V = max;
    
    float min = R < G ? R : G;
    min = min < B ? min : B; 
    
    float D = max - min;
    float S = (fabsf(max) < EPSILON) ? 0 : D / max;
    
    float H;
    if(max == min) {
      H = 0; // achromatic
    } else {
      if (max == R) {
        H = (G - B) / D + (G < B ? 6 : 0);
      } else if (max == G) {
        H = (B - R) / D + 2;
      } else {  // max == B
        H = (R - G) / D + 4;
      }
      H /= 6;
    }
    
    dst[3*pix] = static_cast<T>(H * 255.0f);
    dst[3*pix+1] = static_cast<T>(S * 255.0f);
    dst[3*pix+2] = static_cast<T>(V * 255.0f);
    
  }
};

// Equations from here: http://en.wikipedia.org/wiki/HSL_and_HSV
template <class T> void convertHSVToRGB(T* dst, T* src, const uint32_t w, 
                                        const uint32_t h) {
  for (uint32_t pix = 0; pix < w * h; pix++) {
    float H = static_cast<float>(src[3*pix]) / 255.0f;
    float S = static_cast<float>(src[3*pix+1]) / 255.0f;
    float V = static_cast<float>(src[3*pix+2]) / 255.0f;
    
    float i = floorf(H * 6);
    float f = H * 6 - i;
    float p = V * (1 - S);
    float q = V * (1 - f * S);
    float t = V * (1 - (1 - f) * S);
    
    float R, G, B;
    switch (static_cast<int>(i) % 6) {
      case 0: R = V; G = t; B = p; break;
      case 1: R = q; G = V; B = p; break;
      case 2: R = p; G = V; B = t; break;
      case 3: R = p; G = q; B = V; break;
      case 4: R = t; G = p; B = V; break;
      case 5: R = V; G = p; B = q; break;
    };
    
    dst[3*pix] = static_cast<T>(R * 255.0f);
    dst[3*pix+1] = static_cast<T>(G * 255.0f);
    dst[3*pix+2] = static_cast<T>(B * 255.0f);
  }
};

template <class T> void convertHueToRGB(T* dst, T* src, const uint32_t w, 
                                        const uint32_t h) {
  for (uint32_t pix = 0; pix < w * h; pix++) {
    T H = src[3*pix];
        
    dst[3*pix] = H;
    dst[3*pix+1] = H;
    dst[3*pix+2] = H;
  }
};  

template <class T> void convertSaturationToRGB(T* dst, T* src, const uint32_t w, 
                                               const uint32_t h) {
  for (uint32_t pix = 0; pix < w * h; pix++) {
    T S = src[3*pix+1];
    
    dst[3*pix] = S;
    dst[3*pix+1] = S;
    dst[3*pix+2] = S;
  }
};

template <class T> void convertValueToRGB(T* dst, T* src, const uint32_t w, 
                                          const uint32_t h) {
  for (uint32_t pix = 0; pix < w * h; pix++) {
    T V = src[3*pix+2];
    
    dst[3*pix] = V;
    dst[3*pix+1] = V;
    dst[3*pix+2] = V;
  }
};

template <class T> void ShrinkFilter(T* dst, T* src, const int32_t w, 
                                     const int32_t h, const int32_t rad) {
  T dummy;
  static_cast<void>(dummy);
  memcpy(dst, src, w * h * sizeof(dummy));
  if (rad > 0) {
    // Shrink horizontally
    int32_t index = 0;
    for (int32_t v = 0; v < h; v++) {
      for (int32_t u = 0; u < w; u++) {
        if (src[index] == 0) {
          for (int32_t u_offset = u - rad; u_offset <= u + rad; u_offset++) {
            if (u_offset < w && u_offset >= 0) {
              dst[v * w + u_offset] = 0;
            }
          }
        }
        index++;
      }
    }
    // Shrink vertically
    index = 0;
    for (int32_t v = 0; v < h; v++) {
      for (int32_t u = 0; u < w; u++) {
        if (src[index] == 0) {
          for (int32_t v_offset = v - rad; v_offset <= v + rad; v_offset++) {
            if (v_offset < h && v_offset >= 0) {
              dst[v_offset * w + u] = 0;
            }
          }
        }
        index++;
      }
    }
  }
};

#endif  // UNNAMED_LOAD_DEPTH_IMAGE_HEADER