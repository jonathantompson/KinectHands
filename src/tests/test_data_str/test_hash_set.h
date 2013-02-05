//
//  test_circular_buffer.h
//
//  Created by Jonathan Tompson on 4/26/12.
//

#include "data_str/hash_set.h"
#include "tests/test_unit/test_unit.h"
#include "data_str/hash_funcs.h"

#define TEST_HS_START_SIZE 101  // A "bigish" prime
#define TEST_HS_NUM_VALUES 2048

namespace tests {

  using data_str::HashSet;
  using data_str::HashUint;

  // TEST 1: Create a hash table, insert items from 1:N
  TEST(HashSet, CreationAndInsertion) {
    HashSet<uint32_t> ht(TEST_HS_START_SIZE, &HashUint);

    for (int i = 0; i < TEST_HS_NUM_VALUES; i += 1) {
      EXPECT_EQ(ht.insert(i), true);  // Insert key = i, value = i*2
    }
    // Now check that they're all there
    for (uint32_t i = 0; i < TEST_HS_NUM_VALUES; i += 1) {
      EXPECT_TRUE(ht.lookup(i));  // find key = i (value = i)
    }
    for (uint32_t i = TEST_HS_NUM_VALUES; i < 2*TEST_HS_NUM_VALUES; i += 1) {
      EXPECT_FALSE(ht.lookup(i));  // find key = i (value = i)
    }
    EXPECT_EQ(ht.count(), TEST_HS_NUM_VALUES);
  }

  // TEST 2: Create a hash table, insert items from 1:N and delete some
  TEST(HashSet, CreationAndDeletion) {
    HashSet<uint32_t> ht(TEST_HS_START_SIZE, &HashUint);
    EXPECT_FALSE(ht.remove(1));  // Nothing there: should fail
    EXPECT_TRUE(ht.insert(0));
    EXPECT_FALSE(ht.remove(1));  // 0 is there, but 1 is not: should fail
    EXPECT_TRUE(ht.insert(1));
    EXPECT_TRUE(ht.remove(1));  // should work this time
    EXPECT_FALSE(ht.remove(1));  // Make sure the value is actually gone
    EXPECT_TRUE(ht.lookup(0));  // key=0 should still be there
  }

  // TEST 3: Duplicate insertion
  TEST(HashSet, DuplicateInsertion) {
    HashSet<uint32_t> ht(TEST_HS_START_SIZE, &HashUint);
    EXPECT_TRUE(ht.insert(0));
    EXPECT_FALSE(ht.insert(0));  // Try twice with two different values
    EXPECT_FALSE(ht.insert(0));  // (since value shouldn't matter)
  }

}  // unnamed namespace
