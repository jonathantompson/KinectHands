//
//  min_heap_edges.h
//
//  Created by Jonathan Tompson on 6/26/12.
//
//  A very simple binary min heap class that orders by the "cost" variable of 
//  each edge.
//  
//  NOTE: The heap can be built in O(n) time.  this is a modified version of the
//        regular data_str::MinHeap to use pointers to edges for speed.
//

#ifndef MESH_SIMPLIFICATION_MIN_HEAP_EDGES_HEADER
#define MESH_SIMPLIFICATION_MIN_HEAP_EDGES_HEADER

#include <string>
#include <stdio.h>  // For printf()
#ifdef __APPLE__
#include <stdexcept>
#endif
#include "alignment/data_align.h"
#include "math/math_types.h"  // for uint
#include "data_str/vector.h"
#include "mesh_simplification/edge.h"

namespace mesh_simplification {
  class Edge;
  
  class MinHeapEdges {
  public:
    MinHeapEdges(data_str::Vector<Edge>* we);
    ~MinHeapEdges();

    Edge* removeMin();
    void fixHeap(uint32_t i);
    void remove(uint32_t i);
    void insert(Edge* edge);
  
    inline uint32_t size() { return pvec_.size(); }
    inline uint32_t capacity() { return pvec_.capacity(); }
    
    bool validate();  // For testing purposes (do not delete)
    void print();

  private:
    data_str::Vector<Edge*> pvec_;
    
    inline static uint32_t nextPow2(uint32_t v);
    uint32_t reheapifyUp(uint32_t i);  // Returns the new position after heapify
    uint32_t reheapifyDown(uint32_t i);
    inline static uint32_t getParent(uint32_t i);
    inline static uint32_t getLChild(uint32_t i);
    inline static uint32_t getRChild(uint32_t i);
  };

};  // namespace mesh_simplification

#endif  // MESH_SIMPLIFICATION_MIN_HEAP_EDGES_HEADER

