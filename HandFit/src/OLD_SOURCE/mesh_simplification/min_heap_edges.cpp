//
//  min_heap_edges.cpp
//
//  Created by Jonathan Tompson on 6/26/12.
//
//  A very simple binary min heap class that orders by the "cost" variable of 
//  each edge.
//  
//  NOTE: The heap can be built in O(n) time.  this is a modified version of the
//        regular data_str::MinHeap to use pointers to edges for speed.
//

#include <string>
#include <stdio.h>  // For printf()
#ifdef __APPLE__
#include <stdexcept>
#endif
#include "mesh_simplification/min_heap_edges.h"
#include "alignment/data_align.h"
#include "math/math_types.h"  // for uint
#include "data_str/vector.h"
#include "mesh_simplification/edge.h"
#include "exceptions/wruntime_error.h"

namespace mesh_simplification {

  MinHeapEdges::MinHeapEdges(data_str::Vector<Edge>* we) {
    if (we->size() == 0) {
      throw std::wruntime_error("MinHeapEdges::MinHeapEdges() - Error, data size is 0!");
    }
    uint32_t capacity_ = nextPow2(we->size());
    pvec_.capacity(capacity_);
    // Copy over the data
    pvec_.resize(we->size());
    pvec_[0] = we->at(0);  // First pointer
    pvec_[0]->heap_index = 0;
    // Use pointer arithmetic to get the rest of the pointer values
    for (uint32_t i = 1; i < pvec_.size(); i++) {
      pvec_[i] = pvec_[i-1] + 1;
      pvec_[i]->heap_index = i;
    }
    
    // Now heapify from the bottom up
    // Note: the first non-leaf is capacity / 2
    for (int i = pvec_.capacity()/2; i != MAX_UINT32; i--) {  // Assume wrapping
      // Reheapify the subtree starting from i:
      uint32_t old_pos = MAX_UINT32;
      uint32_t cur_pos = i;
      while (old_pos != cur_pos) {
        old_pos = cur_pos;
        cur_pos = reheapifyDown(cur_pos);
      }
    }
  };
  
  MinHeapEdges::~MinHeapEdges() {
    pvec_.resize(0);  // But don't release any of the pointers (the memory
                      // belongs to someone else).
  }
  
  bool MinHeapEdges::validate() {
    for (uint32_t i = 1; i < pvec_.size(); i++) {
      uint32_t ind_parent = getParent(i);
      if (pvec_[ind_parent]->cost > pvec_[i]->cost) {
        return false;  // the parent cost is greater than a child!
      }
      if (pvec_[i]->heap_index != i) {
        return false;
      }
    }
    return true;
  };  
  
  void MinHeapEdges::print() {
    if (pvec_.size() == 0) {
      printf("MinHeap[] = []\n");
      return;
    }
    printf("MinHeap[0:%d] = [", pvec_.size()-1);
    for (uint32_t i = 0; i < pvec_.size(); i++) {
      printf("%.3e", pvec_[i]->cost);
      if (i != pvec_.size()-1) {
        printf(", ");
      }
    }
    printf("]\n");
  };   
  
  uint32_t MinHeapEdges::reheapifyUp(uint32_t i) {
    if (i == 0) {
      return i;
    }
    uint32_t ind_parent = getParent(i);
    if (pvec_[ind_parent]->cost > pvec_[i]->cost) {
      Edge* parent = pvec_[ind_parent];
      pvec_[ind_parent] = pvec_[i];
      pvec_[ind_parent]->heap_index = ind_parent;
      pvec_[i] = parent;
      parent->heap_index = i;
      return ind_parent;
    } else {
      return i;
    }
  };
  
  Edge* MinHeapEdges::removeMin() {
    if (pvec_.size() == 0) {
      throw std::wruntime_error("Heap<T>::removeMin() - Heap size is 0!");
    }
    
    pvec_[0]->heap_index = MAX_UINT32;  // invalidate the element
    
    Edge* ret_val = pvec_[0];
    pvec_[0] = pvec_[pvec_.size()-1];  // Move the last element to the root
    pvec_[0]->heap_index = 0;
    pvec_.popBack();
    if (pvec_.size() > 0) {
      uint32_t old_pos = MAX_UINT32;
      uint32_t cur_pos = 0;
      while (old_pos != cur_pos) {
        old_pos = cur_pos;
        cur_pos = reheapifyDown(cur_pos);
      }
    }
    return ret_val;
  };
  
  uint32_t MinHeapEdges::reheapifyDown(uint32_t i) {
    uint32_t max_index = pvec_.size()-1;
    if (i >= max_index) {
      return i;
    }
    uint32_t ind_l_child = getLChild(i);
    uint32_t ind_r_child = getRChild(i);
    bool bigger_than_l_child = false;
    bool bigger_than_r_child = false;
    
    if ((ind_l_child <= max_index) && (pvec_[i]->cost > pvec_[ind_l_child]->cost)) {
      bigger_than_l_child = true;
    }
    if ((ind_r_child <= max_index) && (pvec_[i]->cost > pvec_[ind_r_child]->cost)) {
      bigger_than_r_child = true;
    }
    
    uint32_t swap_index = i;
    if (bigger_than_l_child && bigger_than_r_child) {
      // Swap with the smallest child
      if (pvec_[ind_l_child]->cost > pvec_[ind_r_child]->cost) {
        swap_index = ind_r_child;
      } else {
        swap_index = ind_l_child;
      }
    } else if (bigger_than_l_child) {
      swap_index = ind_l_child;
    } else if (bigger_than_r_child) {
      swap_index = ind_r_child;
    }
    
    if (swap_index != i) {
      Edge* child = pvec_[swap_index];
      pvec_[swap_index] = pvec_[i];
      pvec_[swap_index]->heap_index = swap_index;
      pvec_[i] = child;
      pvec_[i]->heap_index = i;
    }
    return swap_index;
  };  
  
  // Define this locally (it actually exists in math base) to maintain the
  // independance of this header (the cost is a little code bloat).
  uint32_t MinHeapEdges::nextPow2(uint32_t v) {
    // From: http://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
  };
  
  uint32_t MinHeapEdges::getParent(uint32_t i) {
    return (static_cast<int>(i)-1) / 2;
  };
  
  uint32_t MinHeapEdges::getLChild(uint32_t i) {
    return 2*i + 1;
  };
  
  uint32_t MinHeapEdges::getRChild(uint32_t i) {
    return 2*i + 2;
  };

  // fixHeap is called when the cost associated with node i has been modified
  // we have to update the heap to ensure valid heap properties
  // This operation is O(log(n))
  void MinHeapEdges::fixHeap(uint32_t i) {
    if (i == MAX_UINT32) {
      return;  // edge has already been removed from the heap, but may not have
               // been removed from the mesh
    }

    uint32_t parent = getParent(i);
    
    if (i != 0 && pvec_[parent]->cost > pvec_[i]->cost) {
      // Out of order, cost became less than our parent
      uint32_t old_pos = MAX_UINT32;
      uint32_t cur_pos = i;
      while (old_pos != cur_pos) {
        old_pos = cur_pos;
        cur_pos = reheapifyUp(cur_pos);  // Try bubbling up
      }
    } else {
      uint32_t old_pos = MAX_UINT32;
      uint32_t cur_pos = i;
      while (old_pos != cur_pos) {
        old_pos = cur_pos;
        cur_pos = reheapifyDown(cur_pos);  // try bubbling down
      }      
    }
  }
  
  // remove
  // This operation is O(log(n))
  void MinHeapEdges::remove(uint32_t i) {
    if (i == MAX_UINT32) {
      return;  // edge has already been removed from the heap, but wasn't
               // removed from the mesh until later.
    }
    
    // swap the ith value with the last value
    pvec_[i]->heap_index = MAX_UINT32;  // invalidate the element
    if (i < pvec_.size()-1) {
      pvec_[i] = pvec_[pvec_.size()-1];  // Move the last element to position i
      pvec_[i]->heap_index = i;
      pvec_.popBack();
      fixHeap(i);
    } else {
      pvec_.popBack();
      // No need to fix heap because it was already the last element
    }
  }

  void MinHeapEdges::insert(Edge* edge) {
    uint32_t i = pvec_.size();

    edge->heap_index = i;
    pvec_.pushBack(edge);

    uint32_t parent = getParent(i);
    
    if (i != 0 && pvec_[parent]->cost > pvec_[i]->cost) {
      // Out of order, cost became less than our parent
      uint32_t old_pos = MAX_UINT32;
      uint32_t cur_pos = i;
      while (old_pos != cur_pos) {
        old_pos = cur_pos;
        cur_pos = reheapifyUp(cur_pos);  // Try bubbling up
      }
    }
  }

}  // namespace mesh_simplification
