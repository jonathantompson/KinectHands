//
//  hash_set.h
//
//  Created by Jonathan Tompson on 4/26/12.
//
//  This is an open addressing hash table.  Closed hashing with chaining can be
//  slow due to lots of malloc calls.  The probing is just simple linear 
//  probing.  The max load factor is 0.5, in order to maintain reasonable
//  lookup times.
//
//  This is a hash_set so it is a set of keys (use hash_map for key/values).
//  WORKS BEST IF SIZE IS A PRIME NUMBER, ESPECIALLY WITH A MOD HASH FUNCTION
//

#ifndef DATA_STR_HASH_SET_HEADER
#define DATA_STR_HASH_SET_HEADER

#include <stdlib.h>  // For exit()
#include <stdio.h>  // For printf()
#include "math/math_types.h"  // for uint
#include "exceptions/wruntime_error.h"

#ifndef NULL
#define NULL 0
#endif

namespace data_str {

  template <class TKey>
  class HashSet {
  public:
    // Function pointer for the hash function
    typedef uint32_t (*HashFunc) (uint32_t size, TKey key);

    HashSet(uint32_t size, HashFunc hash_func);
    ~HashSet();

    inline uint32_t size() { return size_; }
    inline uint32_t count() { return count_; }
    inline float load() { return load_factor_; }
    bool insert(TKey key);
    bool remove(TKey key);
    bool lookup(TKey key);

  private:
    uint32_t size_;
    uint32_t count_;
    TKey* table_;
    bool* bucket_full_;  // Array of booleans telling us if an item exists there
    float load_factor_;
    static const float max_load_;

    HashFunc hash_func_;
    uint32_t linearProbeFunc(uint32_t hash, uint32_t probe_index);
    void rehash();

    // Non-copyable, non-assignable.
    HashSet(HashSet&);
    HashSet& operator=(const HashSet&);
  };

  template <class TKey>
  const float HashSet<TKey>::max_load_ = 0.5f;

  template <class TKey>
  uint32_t HashSet<TKey>::linearProbeFunc(uint32_t hash, uint32_t probe_index) {
    return (hash + probe_index) % size_;
  };

  template <class TKey>
  HashSet<TKey>::HashSet(uint32_t size, HashFunc hash_func) {
    hash_func_ = hash_func;
    load_factor_ = 0;
    count_ = 0;
    size_ = size;
    if (size_ < 1) {
      throw std::wruntime_error("HashSet<TKey>::HashSet: size < 1");
    }
    table_ = new TKey[size_];
    bucket_full_ = new bool[size_];
    memset(bucket_full_, false, size_*sizeof(bucket_full_[0]));
  };

  template <class TKey>
  void HashSet<TKey>::rehash() {
    // printf("HashSet rehash\n");
    uint32_t new_size = size_*2;
    TKey* new_table = new TKey[new_size];
    bool* new_bucket_full = new bool[new_size];
    memset(new_bucket_full, false, new_size*sizeof(new_bucket_full[0]));
    // manually insert all the old key/value pairs into the hash table
    for (uint32_t j = 0; j < size_; j ++) {
      if (bucket_full_[j]) {
        TKey curKey = table_[j];
        bool value_inserted = false;
        // Now insert this key/value pair
        for (uint32_t i = 0; i < size_; i ++) {
          uint32_t hash = linearProbeFunc(hash_func_(size_, curKey), i);
          if (!new_bucket_full[hash]) {
            new_bucket_full[hash] = true;
            new_table[hash] = curKey;
            value_inserted = true;
            break;
          }
        }  // end for (uint32_t i = 0; i < size_; i ++)
        if (!value_inserted) {
          printf("HashSet<TKey>::rehash - Couldn't insert a value!  ");
          printf("This shouldn't happen.  Check hash table logic.\n");
          throw std::wruntime_error("HashSet<TKey>::rehash: insert failed");
        }
      }  // end if (bucket_full_[i])
    }
    size_ = new_size;
    delete[] table_;
    delete[] bucket_full_;
    table_ = new_table;
    bucket_full_ = new_bucket_full;
  };

  template <class TKey>
  HashSet<TKey>::~HashSet() {
    if (table_)
      delete[] table_;
    if (bucket_full_)
      delete[] bucket_full_;
  };

  template <class TKey>
  bool HashSet<TKey>::insert(TKey key) {
    // key trying to insert until the table is full.  We're doing linear probes
    // http://en.wikipedia.org/wiki/Linear_probing
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(hash_func_(size_, key), i);
      if (!bucket_full_[hash]) {
        bucket_full_[hash] = true;
        table_[hash] = key;
        count_++;
        load_factor_ = static_cast<float>(count_) / static_cast<float>(size_);
        while (load_factor_ > max_load_) {
          rehash();
          load_factor_ = static_cast<float>(count_) / 
            static_cast<float>(size_);
        }
        return true;
      } else {
        if (table_[hash] == key)  // Key already exists
          return false;
      }
    }
    return false;
  };

  template <class TKey>
  bool HashSet<TKey>::remove(TKey key) {
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(hash_func_(size_, key), i);
      if (!bucket_full_[hash]) {
        return false;
      } else {
        if (table_[hash] == key) {
          bucket_full_[hash] = false;
          count_--;
          load_factor_ = static_cast<float>(count_) / 
            static_cast<float>(size_);
          return true;
        }
      }
    }
    return false;
  };

  template <class TKey>
  bool HashSet<TKey>::lookup(TKey key) {
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(hash_func_(size_, key), i);
      if (!bucket_full_[hash]) {
        return false;
      } else {
        if (table_[hash] == key) {
          return true;
        }
      }
    }
    return false;
  };

};  // data_str namespace

#endif  // DATA_STR_HASH_SET_HEADER
