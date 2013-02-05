//
//  min_heap_contours.h
//
//  Created by Jonathan Tompson on 10/24/12.
//
//  A very simple binary min heap class that orders by the "cost" variable of 
//  each contour.
//  
//  NOTE: The heap can be built in O(n) time.  this is a modified version of the
//        regular data_str::MinHeap to use pointers to Contours for speed.
//

#ifndef MARCHING_SQUARES_MIN_HEAP_CONTOURS_HEADER
#define MARCHING_SQUARES_MIN_HEAP_CONTOURS_HEADER

#include <string>
#include <stdio.h>  // For printf()
#ifdef __APPLE__
  #include <stdexcept>
#endif
#include "math/math_types.h"  // for uint
#include "data_str/vector.h"
#include "marching_squares/contour.h"

namespace marching_squares {

  class MinHeapContours {
  public:
    MinHeapContours();
    ~MinHeapContours();

    void buildHeap(data_str::Vector<Contour>* we);
    Contour* removeMin();
    void fixHeap(uint32_t i);
    void remove(uint32_t i);
    void insert(Contour* Contour);
  
    inline uint32_t size() { return pvec_.size(); }
    inline uint32_t capacity() { return pvec_.capacity(); }
    
    bool validate();  // For testing purposes (do not delete)
    void print();

  private:
    data_str::Vector<Contour*> pvec_;
    
    inline static uint32_t nextPow2(uint32_t v);
    uint32_t reheapifyUp(uint32_t i);  // Returns the new position after heapify
    uint32_t reheapifyDown(uint32_t i);
    inline static uint32_t getParent(uint32_t i);
    inline static uint32_t getLChild(uint32_t i);
    inline static uint32_t getRChild(uint32_t i);
  };

};  // namespace marching_squares

#endif  // MARCHING_SQUARES_MIN_HEAP_CONTOURS_HEADER

