//
//  hash_map.h
//
//  Created by Jonathan Tompson on 4/26/12.
//
//  This is an open addressing hash table.  Closed hashing with chaining can be
//  slow due to lots of malloc calls.  The probing is just simple linear 
//  probing.  The max load factor is 0.5, in order to maintain reasonable
//  lookup times.
//
//  This is a hash_map so it is a set of key and value pairs.
//  WORKS BEST IF SIZE IS A PRIME NUMBER, ESPECIALLY WITH A MOD HASH FUNCTION
//
//  NOTE: When insert is called, HashMap will make a copy of the input element's
//        value field (Hashmap owns the copy.).
//        IMPORTANT:  When the <TValue> is a pointer (ie <int*>), then ownership
//        IS TRANSFERRED.  The HashMap will call the destructor for that
//        memory when clear is called or the HashMap destructor is called.
//

#ifndef DATA_STR_HASH_MAP_MANAGED_HEADER
#define DATA_STR_HASH_MAP_MANAGED_HEADER

#include <stdio.h>  // For printf()
#include "math/math_types.h"  // for uint
#include "data_str/pair.h"
#include "exceptions/wruntime_error.h"

#ifndef NULL
#define NULL 0
#endif

namespace data_str {

  template <class TKey, class TValue>
  class HashMapManaged {
  public:
    // Function pointer for the hash function
    typedef uint32_t (*HashFunc) (uint32_t size, TKey key);

    HashMapManaged(uint32_t size, HashFunc hash_func);
    ~HashMapManaged();

    inline uint32_t size() { return size_; }
    inline uint32_t count() { return count_; }
    inline float load() { return load_factor_; }
    bool insert(const TKey& key, const TValue& value);
    bool remove(const TKey& key);
    bool lookup(const TKey& key, TValue& value);

  protected:
    uint32_t size_;
    uint32_t count_;
    Pair<TKey, TValue>* table_;
    bool* bucket_full_;  // Array of booleans telling us if an item exists there
    float load_factor_;
    static const float max_load_;

    HashFunc hash_func_;
    uint32_t linearProbeFunc(uint32_t hash, uint32_t probe_index);
    void rehash();

    // Non-copyable, non-assignable.
    HashMapManaged(HashMapManaged&);
    HashMapManaged& operator=(const HashMapManaged&);
  };

  template <class TKey, class TValue>
  const float HashMapManaged<TKey, TValue>::max_load_ = 0.5f;

  template <class TKey, class TValue>
  uint32_t HashMapManaged<TKey, TValue>::linearProbeFunc(uint32_t hash, 
    uint32_t probe_index) {
    return (hash + probe_index) % size_;
  };

  template <class TKey, class TValue>
  HashMapManaged<TKey, TValue>::HashMapManaged(uint32_t size, 
                                               HashFunc hash_func) {
    hash_func_ = hash_func;
    load_factor_ = 0;
    count_ = 0;
    size_ = size;
    if (size_ < 1) {
      throw std::wruntime_error(std::wstring(L"HashMapManaged<TKey, TValue>") +
        std::wstring(L"::HashMapManaged: size < 1"));
    }
    table_ = new Pair<TKey, TValue>[size_];
    bucket_full_ = new bool[size_];
    memset(bucket_full_, false, size_*sizeof(bucket_full_[0]));
  };

  template <class TKey, class TValue>
  void HashMapManaged<TKey, TValue>::rehash() {
    // printf("HashMapManaged rehash\n");
    uint32_t new_size = size_*2;
    Pair<TKey, TValue>* new_table = new Pair<TKey, TValue>[new_size];
    bool* new_bucket_full = new bool[new_size];
    memset(new_bucket_full, false, new_size*sizeof(new_bucket_full[0]));
    // manually insert all the old key/value pairs into the hash table
    for (uint32_t j = 0; j < size_; j ++) {
      if (bucket_full_[j]) {
        TKey curKey = table_[j].first;
        TValue curValue = table_[j].second;
        bool value_inserted = false;
        // Now insert this key/value pair
        for (uint32_t i = 0; i < size_; i ++) {
          uint32_t hash = linearProbeFunc(hash_func_(size_, curKey), i);
          if (!new_bucket_full[hash]) {
            new_bucket_full[hash] = true;
            new_table[hash].first = curKey;
            new_table[hash].second = curValue;
            value_inserted = true;
            break;
          }
        }  // end for (uint32_t i = 0; i < size_; i ++)
        if (!value_inserted) {
          printf("HashMapManaged<TKey, TValue>::rehash - Couldn't insert!  ");
          printf("This shouldn't happen.  Check hash table logic.\n");
          throw std::wruntime_error(
            "HashMapManaged<TKey, TValue>::rehash: insert failed");
        }
      }  // end if (bucket_full_[i])
    }
    size_ = new_size;
    delete[] table_;
    delete[] bucket_full_;
    table_ = new_table;
    bucket_full_ = new_bucket_full;
  };

  template <class TKey, class TValue>
  HashMapManaged<TKey, TValue>::~HashMapManaged() {
    if (table_)
      delete[] table_;
    if (bucket_full_)
      delete[] bucket_full_;
  };

  template <class TKey, class TValue>
  bool HashMapManaged<TKey, TValue>::insert(const TKey& key, 
                                            const TValue& value) {
    // keep trying to insert until the table is full.  We're doing linear probes
    // http://en.wikipedia.org/wiki/Linear_probing
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(hash_func_(size_, key), i);
      if (!bucket_full_[hash]) {
        bucket_full_[hash] = true;
        table_[hash].first = key;
        table_[hash].second = value;
        count_++;
        load_factor_ = static_cast<float>(count_) / static_cast<float>(size_);
        while (load_factor_ > max_load_) {
          rehash();
          load_factor_ = static_cast<float>(count_) / 
            static_cast<float>(size_);
        }
        return true;
      } else {
        if (table_[hash].first == key)  // Key already exists
          return false;
      }
    }
    return false;
  };

  template <class TKey, class TValue>
  bool HashMapManaged<TKey, TValue>::remove(const TKey& key) {
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(hash_func_(size_, key), i);
      if (!bucket_full_[hash]) {
        return false;
      } else {
        if (table_[hash].first == key) {
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

  template <class TKey, class TValue>
  bool HashMapManaged<TKey, TValue>::lookup(const TKey& key, TValue& value) {
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(hash_func_(size_, key), i);
      if (!bucket_full_[hash]) {
        return false;
      } else {
        if (table_[hash].first == key) {
          value = table_[hash].second;
          return true;
        }
      }
    }
    return false;
  };
  
  // *************************************************************************
  // Template specialization for pointers, note if <int*> is used, TValue is
  // an int in this case.  NOT a int*.  Hence, we need Pair<TKey, TValue*> and 
  // not Pair<TKey, TValue>.
  // NOTE: We could redefine explicit functions by extending the normal 
  // HashMapManaged<>, but it turns out we need to re-write most of it anyway, 
  // so it's easier to just keep them seperate.
  template <class TKey, class TValue>
  class HashMapManaged<TKey, TValue*> {
  public:
    // Function pointer for the hash function
    typedef uint32_t (*HashFunc) (uint32_t size, TKey key);

    HashMapManaged<TKey, TValue*>(uint32_t size, HashFunc hash_func);
    ~HashMapManaged<TKey, TValue*>();

    inline uint32_t size() { return size_; }
    inline uint32_t count() { return count_; }
    inline float load() { return load_factor_; }
    bool insert(const TKey& key, TValue* value);
    bool remove(const TKey& key);
    bool lookup(const TKey& key, TValue*& value);

  private:
    uint32_t size_;
    uint32_t count_;
    Pair<TKey, TValue*>* table_;
    bool* bucket_full_;  // Array of booleans telling us if an item exists there
    float load_factor_;
    static const float max_load_;

    HashFunc hash_func_;
    uint32_t linearProbeFunc(uint32_t hash, uint32_t probe_index);
    void rehash();

    // Non-copyable, non-assignable.
    HashMapManaged(HashMapManaged&);
    HashMapManaged& operator=(const HashMapManaged&);
  };

  template <class TKey, class TValue>
  const float HashMapManaged<TKey, TValue*>::max_load_ = 0.5f;

  template <class TKey, class TValue>
  uint32_t HashMapManaged<TKey, TValue*>::linearProbeFunc(uint32_t hash, 
    uint32_t probe_index) {
    return (hash + probe_index) % size_;
  };

  template <class TKey, class TValue>
  HashMapManaged<TKey, TValue*>::HashMapManaged(uint32_t size, 
                                                HashFunc hash_func) {
    hash_func_ = hash_func;
    load_factor_ = 0;
    count_ = 0;
    size_ = size;
    if (size_ < 1) {
      throw std::wruntime_error(std::wstring(L"HashMapManaged<TKey, TValue*>") +
        std::wstring(L"::HashMapManaged: size < 1"));
    }
    table_ = new Pair<TKey, TValue*>[size_];
    for (uint32_t i = 0; i < size_; i ++) {
      table_[i].second = NULL;
    }
    bucket_full_ = new bool[size_];
    memset(bucket_full_, false, size_*sizeof(bucket_full_[0]));
  };

  template <class TKey, class TValue>
  void HashMapManaged<TKey, TValue*>::rehash() {
    // printf("HashMapManaged rehash\n");
    uint32_t new_size = size_*2;
    Pair<TKey, TValue*>* new_table = new Pair<TKey, TValue*>[new_size];
    for (uint32_t i = 0; i < new_size; i ++) {
      new_table[i].second = NULL;
    }
    bool* new_bucket_full = new bool[new_size];
    memset(new_bucket_full, false, new_size*sizeof(new_bucket_full[0]));
    // manually insert all the old key/value pairs into the hash table
    for (uint32_t j = 0; j < size_; j ++) {
      if (bucket_full_[j]) {
        TKey curKey = table_[j].first;
        TValue* curValue = table_[j].second;
        bool value_inserted = false;
        // Now insert this key/value pair
        for (uint32_t i = 0; i < size_; i ++) {
          uint32_t hash = linearProbeFunc(hash_func_(size_, curKey), i);
          if (!new_bucket_full[hash]) {
            new_bucket_full[hash] = true;
            new_table[hash].first = curKey;
            new_table[hash].second = curValue;
            value_inserted = true;
            break;
          }
        }  // end for (uint32_t i = 0; i < size_; i ++)
        if (!value_inserted) {
          printf("HashMapManaged<TKey, TValue*>::rehash - Couldn't insert!  ");
          printf("This shouldn't happen.  Check hash table logic.\n");
          throw std::wruntime_error(
            "HashMapManaged<TKey, TValue*>::rehash: insert failed");
        }
      }  // end if (bucket_full_[i])
    }
    size_ = new_size;
    delete[] table_;
    delete[] bucket_full_;
    table_ = new_table;
    bucket_full_ = new_bucket_full;
  };

  template <class TKey, class TValue>
  HashMapManaged<TKey, TValue*>::~HashMapManaged() {
    if (table_ && bucket_full_) {
      for (uint32_t i = 0; i < size_; i ++) {
        if (bucket_full_[i]) {
          if (table_[i].second != NULL) {
            delete table_[i].second;
            table_[i].second = NULL;
          }
        }
      }
    }
    if (table_) {
      delete[] table_;
    }
    if (bucket_full_) {
      delete[] bucket_full_;
    }
  };

  template <class TKey, class TValue>
  bool HashMapManaged<TKey, TValue*>::insert(const TKey& key, TValue* value) {
    // keep trying to insert until the table is full.  We're doing linear probes
    // http://en.wikipedia.org/wiki/Linear_probing
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(hash_func_(size_, key), i);
      if (!bucket_full_[hash]) {
        bucket_full_[hash] = true;
        table_[hash].first = key;
        table_[hash].second = value;
        count_++;
        load_factor_ = static_cast<float>(count_) / static_cast<float>(size_);
        while (load_factor_ > max_load_) {
          rehash();
          load_factor_ = static_cast<float>(count_) / 
            static_cast<float>(size_);
        }
        return true;
      } else {
        if (table_[hash].first == key)  // Key already exists
          return false;
      }
    }
    return false;
  };

  template <class TKey, class TValue>
  bool HashMapManaged<TKey, TValue*>::remove(const TKey& key) {
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(hash_func_(size_, key), i);
      if (!bucket_full_[hash]) {
        return false;
      } else {
        if (table_[hash].first == key) {
          bucket_full_[hash] = false;
          count_--;
          if (table_[hash].second != NULL) {
            delete table_[hash].second;
            table_[hash].second = NULL;
          }
          load_factor_ = static_cast<float>(count_) / 
            static_cast<float>(size_);
          return true;
        }
      }
    }
    return false;
  };

  template <class TKey, class TValue>
  bool HashMapManaged<TKey, TValue*>::lookup(const TKey& key, TValue*& value) {
    for (uint32_t i = 0; i < size_; i ++) {
      uint32_t hash = linearProbeFunc(hash_func_(size_, key), i);
      if (!bucket_full_[hash]) {
        return false;
      } else {
        if (table_[hash].first == key) {
          value = table_[hash].second;
          return true;
        }
      }
    }
    return false;
  };
  

};  // data_str namespace

#endif  // DATA_STR_HASH_MAP_MANAGED_HEADER
