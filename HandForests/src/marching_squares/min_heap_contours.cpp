#include <string>
#include <stdio.h>  // For printf()
#ifdef __APPLE__
  #include <stdexcept>
#endif
#include "math/math_types.h"  // for uint
#include "data_str/vector.h"
#include "marching_squares/min_heap_contours.h"
#include "marching_squares/contour.h"

namespace marching_squares {
  MinHeapContours::MinHeapContours() {
    
  };

 void MinHeapContours::buildHeap(data_str::Vector<Contour>* contours) {
   if (contours->size() == 0) {
     throw std::runtime_error("MinHeapContours::buildHeap() - Error, data size is 0!");
   }
   
   uint32_t capacity_ = nextPow2(contours->size());
   if (pvec_.capacity() < capacity_) {
     pvec_.capacity(capacity_);
   }
   pvec_.resize(0);
   
   // Copy over the data
   for (uint32_t i = 0; i < contours->size(); i++) {
     Contour* cont = contours->at(i);
     if (cont->next != MAX_UINT32 && cont->prev != MAX_UINT32) {
       pvec_.pushBack(cont);
       cont->heap_index = pvec_.size() - 1;
     }
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
  
  MinHeapContours::~MinHeapContours() {
    pvec_.resize(0);  // But don't release any of the pointers (the memory
                      // belongs to someone else).
  }

  bool MinHeapContours::validate() {
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
  
  void MinHeapContours::print() {
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
  
  uint32_t MinHeapContours::reheapifyUp(uint32_t i) {
    if (i == 0) {
      return i;
    }
    uint32_t ind_parent = getParent(i);
    if (pvec_[ind_parent]->cost > pvec_[i]->cost) {
      Contour* parent = pvec_[ind_parent];
      pvec_[ind_parent] = pvec_[i];
      pvec_[ind_parent]->heap_index = ind_parent;
      pvec_[i] = parent;
      parent->heap_index = i;
      return ind_parent;
    } else {
      return i;
    }
  };
  
  Contour* MinHeapContours::removeMin() {
    if (pvec_.size() == 0) {
      throw std::runtime_error("Heap<T>::removeMin() - Heap size is 0!");
    }
    
    pvec_[0]->heap_index = MAX_UINT32;  // invalidate the element
    
    Contour* ret_val = pvec_[0];
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
  
  uint32_t MinHeapContours::reheapifyDown(uint32_t i) {
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
      Contour* child = pvec_[swap_index];
      pvec_[swap_index] = pvec_[i];
      pvec_[swap_index]->heap_index = swap_index;
      pvec_[i] = child;
      pvec_[i]->heap_index = i;
    }
    return swap_index;
  };  
  
  // Define this locally (it actually exists in math base) to maintain the
  // independance of this header (the cost is a little code bloat).
  uint32_t MinHeapContours::nextPow2(uint32_t v) {
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
  
  uint32_t MinHeapContours::getParent(uint32_t i) {
    return (static_cast<int>(i)-1) / 2;
  };
  
  uint32_t MinHeapContours::getLChild(uint32_t i) {
    return 2*i + 1;
  };
  
  uint32_t MinHeapContours::getRChild(uint32_t i) {
    return 2*i + 2;
  };

  // fixHeap is called when the cost associated with node i has been modified
  // we have to update the heap to ensure valid heap properties
  // This operation is O(log(n))
  void MinHeapContours::fixHeap(uint32_t i) {
    if (i == MAX_UINT32) {
      return;  // Contour has already been removed from the heap, but may not have
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
  void MinHeapContours::remove(uint32_t i) {
    if (i == MAX_UINT32) {
      return;  // Contour has already been removed from the heap, but wasn't
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

  void MinHeapContours::insert(Contour* Contour) {
    uint32_t i = pvec_.size();

    Contour->heap_index = i;
    pvec_.pushBack(Contour);

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

}  // namespace contour_simplification
