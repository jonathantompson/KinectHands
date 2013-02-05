//
//  vector.h
//
//  Created by Jonathan Tompson on 4/26/12.
//
//  A very simple templated vector class
//  
//  NOTE: When pushBack is called, Vector will make a copy of the input element
//        and add that to the vector.  Ownership for pointers is NOT
//        transferred.  That is, the vector is not responsible for handling
//        cleanup when Vector<type*> is used.
//

#ifndef DATA_STR_VECTOR_HEADER
#define DATA_STR_VECTOR_HEADER

#include <stdio.h>  // For printf()
#include "alignment/data_align.h"
#include "math/math_types.h"  // for uint
#include "exceptions/wruntime_error.h"

#if defined(__APPLE__)
  #define INLINE inline
#elif defined(_WIN32)
  #define INLINE __forceinline
#else
  #define INLINE inline
#endif

namespace data_str {

  template <typename T>
  class Vector {
  public:
    explicit Vector(uint32_t capacity = 0);
    ~Vector();

    void capacity(uint32_t capacity);  // Request manual capacity increase
    void clear();
    INLINE void pushBack(const T& elem);  // Add a copy of the element to back
    INLINE T& popBack();  // remove last element and call it's destructor
    INLINE T* at(uint32_t index);  // Get an internal reference
    INLINE void set(uint32_t index, const T& elem);
    void resize(uint32_t size_);
    INLINE uint32_t size() { return size_; }
    INLINE uint32_t capacity() { return capacity_; }
    bool operator == (const Vector<T>& a);  // O(n) - linear search
    Vector<T>& operator= (const Vector<T>& other);  // O(n) - copy
    INLINE T operator[](uint32_t index) const;
    INLINE T & operator[](uint32_t index);

  private:
    uint32_t size_;
    uint32_t capacity_;  // will only grow or shrink by a factor of 2
    T* pvec_;
  };

  template <typename T>
  Vector<T>::Vector(uint32_t capacity) {  // capacity = 0
    pvec_ = NULL;
    capacity_ = 0;
    size_ = 0;
    if (capacity != 0) {
      this->capacity(capacity);
    }
  };

  template <typename T>
  Vector<T>::~Vector() {
    if (pvec_) { 
#ifdef _WIN32
      _aligned_free(pvec_); 
#else
      free(pvec_); 
#endif
      pvec_ = NULL; 
    }
    capacity_ = 0;
    size_ = 0;
  };

  template <typename T>
  void Vector<T>::capacity(uint32_t capacity) {
    if (capacity != capacity_ && capacity != 0) {
      T* pvec_old = pvec_;

#ifdef _WIN32 
      T dummy;
      static_cast<void>(dummy);  // Get rid of unreference local variable warn
      void* temp = NULL;
      temp = _aligned_malloc(capacity * sizeof(dummy), ALIGNMENT);
#else
      T dummy;
      void* temp = NULL;
      temp = malloc(capacity * sizeof(dummy));
#endif

      if (temp == NULL) { 
        throw std::wruntime_error("Vector<T>::capacity: Malloc Failed.");
      }

      // Use placement new to call the constructors for the array
      pvec_ = reinterpret_cast<T*>(temp);
      for (uint32_t i = 0; i < capacity; i ++) {
        pvec_[i] = *(new(pvec_ + i) T());  // Call placement new on each item
      }

      if (pvec_old) {
        if (capacity <= capacity_) {
          for (uint32_t i = 0; i < capacity; i ++) {
            pvec_[i] = pvec_old[i];
          }
        } else {
          for (uint32_t i = 0; i < capacity_; i ++) {
            pvec_[i] = pvec_old[i];
          }
        }

#ifdef _WIN32
        _aligned_free(pvec_old); 
#else
        free(pvec_old); 
#endif
        pvec_old = NULL;
      }

      capacity_ = capacity;
      if (size_ > capacity_) {  // If we've truncated the array then resize_
        size_ = capacity_;
      }
    } else if (capacity == 0) {  // Clear array if desired capacity is zero
      clear();
    }
  };

  template <typename T>
  void Vector<T>::clear() {
    size_ = 0;
    if (pvec_) { 
#ifdef _WIN32
      _aligned_free(pvec_); 
#else
      free(pvec_);
#endif      
      pvec_ = NULL; 
    }
    capacity_ = 0;
  };

  template <typename T>
  void Vector<T>::pushBack(const T& elem) {
    if (capacity_ == 0)
      capacity(1);
    else if (size_ == capacity_)
      capacity(capacity_ * 2);  // Grow the array by size_ 2
    pvec_[size_] = elem;
    size_ += 1;
  };
  
  template <typename T>
  T& Vector<T>::popBack() {
    if (size_ > 0) {
      size_ -= 1;  // just reduce the size_ by 1
      return pvec_[size_];
    } else {
      throw std::wruntime_error("Vector<T>::popBack: Out of bounds");
    }
  };

  template <typename T>
  T* Vector<T>::at(uint32_t index ) {
#ifdef _DEBUG
    if (index > (size_-1))
      throw std::wruntime_error("Vector<T>::at: Out of bounds");
#endif
    return &pvec_[index];
  };

  template <typename T>
  T Vector<T>::operator[](uint32_t index) const { 
#ifdef _DEBUG
    if (index > (size_-1))
      throw std::wruntime_error("Vector<T>::at: Out of bounds");
#endif
    return pvec_[index]; 
  };

  template <typename T>
  T& Vector<T>::operator[](uint32_t index) { 
#ifdef _DEBUG
    if (index > (size_-1))
      throw std::wruntime_error("Vector<T>::at: Out of bounds");
#endif
    return pvec_[index]; 
  };

  template <typename T>
  void Vector<T>::set(uint32_t index, const T& val ) {
#ifdef _DEBUG
    if (index > (size_-1))
      throw std::wruntime_error("Vector<T>::at: Out of bounds");
#endif
    pvec_[index] = val;
  };

  template <typename T>
  void Vector<T>::resize(uint32_t size) { 
#ifdef _DEBUG
    if ( size > capacity_ || size < 0 ) { 
      throw std::wruntime_error("Vector<T>::resize: Out of bounds");
    } else { 
#endif
      size_ = size; 
#ifdef _DEBUG
    } 
#endif
  };   

  template <typename T>
  bool Vector<T>::operator== (const Vector& a) {
    if (this == &a) {  // if both point to the same memory
      return true; 
    }
    if (size_ != a.size_) { 
      return false; 
    }
    for (uint32_t i = 0; i <= (size_-1); i++) {
      if (pvec_[i] != a.pvec_[i]) { 
        return false; 
      }
    }
    return true;
  };

  template <typename T>
  Vector<T>& Vector<T>::operator= (const Vector<T>& other) {
    if (this != &other) {  // protect against invalid self-assignment
      this->clear();
      this->capacity(other.capacity_);
      for (uint32_t i = 0; i < other.size_; i ++) {
        this->pvec_[i] = other.pvec_[i];
      }
      this->size_ = other.size_;
    }
    // by convention, always return *this
    return *this;
  };

};  // namespace data_str

#endif  // DATA_STR_VECTOR_HEADER

