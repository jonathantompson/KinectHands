//
//  effects.h
//  A C++ port of Ken's Java effects class
//
//  Created by Jonathan Tompson on 4/3/12.
//  Copyright (c) 2012 NYU. All rights reserved.
//

#ifndef EFFECTS_HEADER
#define EFFECTS_HEADER

// 24bit (3 bytes) RGB struct.  Very useful if data is in RGB and you want to do
// a blur on it (and avoid packing to 32bit int with alpha chanel = 255)
struct PixelRGB {
 public:
  PixelRGB() {
    r = 0;
    g = 0;
    b = 0;
  }
  explicit PixelRGB(int val) {
    r = (unsigned char)val;
    g = (unsigned char)val;
    b = (unsigned char)val;
  }
  unsigned char r;  // 8bit
  unsigned char g;  // 8bit
  unsigned char b;  // 8bit
  PixelRGB & operator=(const PixelRGB &rhs) {
    r = rhs.r;
    g = rhs.g;
    b = rhs.b;
    return *this;
  }
  PixelRGB & operator+=(const PixelRGB &rhs) {
    r += rhs.r;
    g += rhs.g;
    b += rhs.b;
    return *this;
  }
  PixelRGB & operator-=(const PixelRGB &rhs) {
    r -= rhs.r;
    g -= rhs.g;
    b -= rhs.b;
    return *this;
  }
  PixelRGB & operator*=(const PixelRGB &rhs) {
    r *= rhs.r;
    g *= rhs.g;
    b *= rhs.b;
    return *this;
  }
  PixelRGB & operator/=(const PixelRGB &rhs) {
    r /= rhs.r;
    g /= rhs.g;
    b /= rhs.b;
    return *this;
  }
  bool operator==(const PixelRGB &other) const {
    return (r == other.r) &&
    (g == other.g) && 
    (b == other.b);
  }
  bool operator!=(const PixelRGB &other) const {
    return !(*this == other);
  }
  const PixelRGB operator+(const PixelRGB &other) const {
    return PixelRGB(*this) += other;
  }
  const PixelRGB operator-(const PixelRGB &other) const {
    return PixelRGB(*this) -= other;
  }
  const PixelRGB operator*(const PixelRGB &other) const {
    return PixelRGB(*this) *= other;
  }
  const PixelRGB operator/(const PixelRGB &other) const {
    return PixelRGB(*this) /= other;
  }
};

template <class T>
class Effects {
 public: 
  Effects(int w, int h);
  ~Effects();
  
  void blurX(T* dst, int radius, int w, int h); 
  void blurY(T* dst, int radius, int w, int h);
  
  ////////////////// PRIVATE METHODS AND DATA //////////////////
  
 private:
  inline int rL(int radius, int x       ) 
    { return radius <= x ? radius : x; }
  inline int rR(int radius, int x, int w) 
    { return radius <= w - 1 - x ? radius : w - 1 - x; }
  inline int rT(int radius, int x       ) 
    { return radius <= x ? radius : x; }
  inline int rB(int radius, int x, int h) 
    { return radius <= h - 1 - x ? radius : h - 1 - x; }
  
  int w, h, deres;
  T* line;
  T** tmp; 
  T** rgb;
};

// **************************************************
//      TEMPLATE CODE MUST BE IN THE HEADER
// **************************************************

template <class T>
Effects<T>::Effects(int w, int h) {
  tmp = new T*[6];
  rgb = new T*[3];
  this->w = w;
  this->h = h;
  deres = 0;
  int temp = 2 * (w >= h ? w : h);
  line = new T[temp];
  for (int d = 0 ; d < 6 ; d++)
    tmp[d] = new T[w * h >> 2 * d];
  for (int i = 0 ; i < 3 ; i++)
    rgb[i] = new T[w * h];
}

template <class T>
Effects<T>::~Effects() {
  for (int d = 0 ; d < 6 ; d++)
    delete[] tmp[d];
  for (int i = 0 ; i < 3 ; i++)
    delete[] rgb[i];
  delete[] tmp;
  delete[] rgb;
  delete[] line;
}

template <class T>
void Effects<T>::blurX(T* dst, int radius, int w, int h) {
  for (int y = 0 ; y < h ; y++) {
    int j = w * y;
    for (int x = 0 ; x < w ; x++) {
      if (dst[j] != 0) {
        int x0 = x;
        int i0 = x + w * y;
        int i1 = w + w * y;
        while (dst[i1 - 1] == 0)
          i1--;
        int x1 = x0 + i1 - i0;
        
        // C --> void * memset ( void * ptr, int value, size_t num );
        int memset_length = (x1 + rR(radius, x1, w) + 1) - 
        (x0 - rL(radius, x0));
        memset(line + (x0 - rL(radius, x0)), 0, sizeof(T)*memset_length);
        for (int i = i0 ; i < i1 ; i++) {
          T d = dst[i];
          line[x - rL(radius, x)] += d;
          line[x + rR(radius, x, w) + 1] -= d;
          x++;
        }
        T sum = 0;
        int i = (x0 - rL(radius, x0)) + w * y;
        for (x = x0 - rL(radius, x0) ; x < x1 + rR(radius, x1, w) + 1 ; x++) {
          dst[i] = (sum += line[x]) / (T)(rL(radius, x) + rR(radius, x, w) + 1);
          i++;
        }
        break;
      }
      j++;
    }
  }
}

template <class T>
void Effects<T>::blurY(T* dst, int radius, int w, int h) {
  for (int x = 0 ; x < w ; x++) {
    int j = x;
    for (int y = 0 ; y < h ; y++) {
      if (dst[j] != 0) {
        int y0 = y;
        int i0 = x + w * y0;
        int i1 = x + w * h;
        while (dst[i1-w] == 0)
          i1-=w;
        int y1 = (i1 - i0) / w + y0;
        
        // C --> void * memset ( void * ptr, int value, size_t num );
        int memset_length = (y1 + rB(radius, y1, h) + 1) - 
        (y0 - rT(radius, y0));
        memset(line + (y0 - rT(radius, y0)), 0, sizeof(T)*memset_length);
        
        for (int i = i0 ; i < i1 ; i+=w) {
          T d = dst[i];
          line[y - rT(radius, y)] += d;
          line[y + rB(radius, y, h) + 1] -= d;
          y++;
        }
        T sum = 0; 
        int i = x + w * (y0 - rT(radius, y0));
        for (y = y0 - rT(radius, y0) ; y < y1 + rB(radius, y1, h) + 1 ; y++) {
          dst[i] = (sum += line[y]) / (rT(radius, y) + rB(radius, y, h) + 1);
          i+=w;
        }
        break;
      }
      j+= w;
    }
  }
}

#endif
