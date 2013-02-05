//
//  hash_funcs.cpp
//
//  Created by Jonathan Tompson on 4/26/12.

#include <cstring>  // For strlen
#include "data_str/hash_funcs.h"  // for uint

namespace data_str {
  // Not very good!  Only even slightly reasonable if size is a good prime.
  // Mostly for testing HashSet and HashMap and HashMapManaged classes.
  uint32_t HashUint(uint32_t size, uint32_t key) {
    return key % (size - 1); 
  }

  // Dynamic string hash version
  // This example is from: http://www.altdevblogaday.com/2011/10/27/quasi-compile-time-string-hashing/
  unsigned int CalculateFNV(const char* str) {
    const size_t length = strlen(str) + 1;
    unsigned int hash = 2166136261u;
    for (size_t i = 0; i < length; ++i) {
      hash ^= *str++;
      hash *= 16777619u;
    }

    return hash;
  }


}  // data_str namespace
