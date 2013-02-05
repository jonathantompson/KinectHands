//
//  contour_simplification.h
//  Created by Jonathan Tompson on 6/21/12.
//
//  Takes as input a 2D boolean mask and a corresponding 2D array of vertex
//  points, and returns a simplified list of contour edges.  
//  For example, the 2D vertex points might be from Kinect data and the 2D
//  mask might be generated from a depth threshold check.
//
//  If you're looking for a standard "marching squares" implementation then
//  you should look in the HandFit project.
//  

#ifndef CONTOUR_SIMPLIFICATION_CONTOUR_SIMPLIFICATION_HEADER
#define CONTOUR_SIMPLIFICATION_CONTOUR_SIMPLIFICATION_HEADER

#include "contour_simplification/contour.h"
#include "data_str/vector.h"
#include "math/math_types.h"
#include "contour_simplification/min_heap_contours.h"

namespace contour_simplification {

  typedef enum {
    MS_LEFT,
    MS_RIGHT,
    MS_UP,
    MS_DOWN,
    MS_UNDEFINED,
  } MS_DIRECTION;

  class ContourSimplification {
  public:
    ContourSimplification() { }

    // Top level external function.  See description in header for details.
    void simplifyContour(uint32_t target_contour_count,
      data_str::Vector<math::Float3>* vertices, 
      data_str::Vector<unsigned char>* mask,
      uint32_t width, uint32_t height);

    data_str::Vector<Contour>* getContourStructure() { return &contours_; }
    data_str::Vector<uint32_t>* getContourStarts() { return &contours_starts_; }
    void printContours();
    void saveContourToFile(std::string file);

  private:
    void marchingSquares(data_str::Vector<math::Float3>* vertices, 
      data_str::Vector<unsigned char>* mask);
    void finishLastContour(uint32_t contour_start);
    void appendContour(math::Float3* vertex, uint32_t contour_start,
      uint32_t contour_index);
    void createMinHeap();
    void cullContours(uint32_t target_contour_count, 
      data_str::Vector<math::Float3>* vertices);
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
};  // namespace contour_simplification

#endif  // CONTOUR_SIMPLIFICATION_CONTOUR_SIMPLIFICATION_HEADER
