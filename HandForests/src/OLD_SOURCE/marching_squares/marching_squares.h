//
//  marching_squares.h
//  Created by Jonathan Tompson on 10/24/12.
//
//  Takes as input a 2D image and a threshold and returns a list of UV 
//  coordinates defining the contours.  There may be more than one contour, so 
//  a list is also generated of the starts of each contour
//
//  Note: - forground is defined as image(u,v) >= threshold
//        - background is defined as image(u,v) < threshold
//
//  This class will also simplify the contour using a minheap approach and is
//  reasonably accurate at maintaining geometry structure even at high
//  decimation rates.
//
//  USAGE:
//
//  #include "marching_squares/marching_squares.h"
//  using marching_squares::MarchingSquares;
//
//  void main {
//    // Load contour from file:
//    static const uint32_t w = 640;
//    static const uint32_t h = 480;
//    float* image = ...;
//
//    static const float thresh = 0.5f;
//    MarchingSquares<float>* ms = new MarchingSquares<float>(image, thresh, w, h);
//
//    static const uint32_t target_contour_length = 1000;
//    ms->simplifyContour(target_contour_length);
//
//    delete ms;
//  }
//

#ifndef MARCHING_SQUARES_MARCHING_SQUARES_HEADER
#define MARCHING_SQUARES_MARCHING_SQUARES_HEADER

#include "math/math_types.h"
#include "data_str/vector.h"
#include "marching_squares/contour.h"
#include "marching_squares/min_heap_contours.h"

namespace marching_squares {

  typedef enum {
    MS_LEFT,
    MS_RIGHT,
    MS_UP,
    MS_DOWN,
    MS_UNDEFINED,
  } MS_DIRECTION;

  template <typename T>
  class MarchingSquares {
  public:
    // Top level constructor will run the marching squares algorithm.
    MarchingSquares(T* mask, T threshold, uint32_t width, uint32_t height);

    // Optional call to simplify the contour (geometry feature preserving)
    void simplifyContour(uint32_t target_contour_count);

    // Getter methods
    data_str::Vector<Contour>* getContourStructure() { return &contours_; }
    data_str::Vector<uint32_t>* getContourStarts() { return &contours_starts_; }
    void printContours();

  private:
    void marchingSquares(T* mask);
    void finishLastContour(uint32_t contour_start);
    void appendContour(uint32_t image_index, uint32_t contour_start,
      uint32_t contour_index);
    void createMinHeap();
    void cullContours(uint32_t target_contour_count);
    void fixContourStarts();

    void stepUp(uint32_t* point, MS_DIRECTION* prev_direction);
    void stepDown(uint32_t* point, MS_DIRECTION* prev_direction);
    void stepLeft(uint32_t* point, MS_DIRECTION* prev_direction);
    void stepRight(uint32_t* point, MS_DIRECTION* prev_direction);

    // Some temporary structures for building the data
    data_str::Vector<Contour> contours_;
    data_str::Vector<uint32_t> contours_starts_;
    data_str::Vector<uint32_t> contours_num_elements_;
    data_str::Vector<float> contours_lengths_;
    data_str::Vector<uint16_t> coded_image_;
    data_str::Vector<bool> finished_contours_;
    MinHeapContours heap_;
    uint32_t width_, height_;
  };
};  // namespace marching_squares

#endif  // MARCHING_SQUARES_MARCHING_SQUARES_HEADER
