//
//  test_circular_buffer.h
//
//  Created by Jonathan Tompson on 4/26/12.
//

#include "data_str/hash_map.h"
#include "tests/test_unit/test_unit.h"
#include "data_str/hash_funcs.h"

#define TEST_HM_START_SIZE 101  // A "bigish" prime
#define TEST_HM_NUM_VALUES 2048

namespace tests {

  using data_str::HashMap;
  using data_str::HashUint;

  // TEST 1: Create a hash table, insert items from 1:N
  TEST(HashMap, CreationAndInsertion) {
    HashMap<uint32_t, uint32_t> ht(TEST_HM_START_SIZE, &HashUint);

    for (uint32_t i = 0; i < TEST_HM_NUM_VALUES; i += 1) {
      uint32_t val = i*2;
      EXPECT_EQ(ht.insert(i, val), true);  // Insert key = i, value = i*2
    }
    // Now check that they're all there
    uint32_t val;
    for (uint32_t i = 0; i < TEST_HM_NUM_VALUES; i += 1) {
      EXPECT_TRUE(ht.lookup(i, val));  // find key = i (value = i)
      EXPECT_EQ(i*2, val);
    }
    for (uint32_t i = TEST_HM_NUM_VALUES; i < 2*TEST_HM_NUM_VALUES; i += 1) {
      EXPECT_FALSE(ht.lookup(i, val));  // find key = i (value = i)
    }
    EXPECT_EQ(ht.count(), TEST_HM_NUM_VALUES);
  }

  // TEST 2: Create a hash table, insert items from 1:N and delete some
  TEST(HashMap, CreationAndDeletion) {
    HashMap<uint32_t, uint32_t> ht(TEST_HM_START_SIZE, &HashUint);
    EXPECT_FALSE(ht.remove(1));  // Nothing there: should fail
    EXPECT_TRUE(ht.insert(0, 1));
    EXPECT_FALSE(ht.remove(1));  // 0 is there, but 1 is not: should fail
    EXPECT_TRUE(ht.insert(1, 0));
    EXPECT_TRUE(ht.remove(1));  // should work this time
    EXPECT_FALSE(ht.remove(1));  // Make sure the value is actually gone
    uint32_t val;
    EXPECT_TRUE(ht.lookup(0, val));  // key=0 should still be there
    EXPECT_EQ(val, 1);  // key=0's value shouldn't have been corrupted
  }

  // TEST 3: Duplicate insertion
  TEST(HashMap, DuplicateInsertion) {
    HashMap<uint32_t, uint32_t> ht(TEST_HM_START_SIZE, &HashUint);
    EXPECT_TRUE(ht.insert(0, 1));
    EXPECT_FALSE(ht.insert(0, 0));  // Try twice with two different values
    EXPECT_FALSE(ht.insert(0, 1));  // (since value shouldn't matter)
  }

  // TEST 1Ptr: Create a hash table, insert items from 1:N
  TEST(HashMapPtr, CreationAndInsertion) {
    HashMap<uint32_t, uint32_t*> ht(TEST_HM_START_SIZE, &HashUint);
    uint32_t* cur_val;

    for (uint32_t i = 0; i < TEST_HM_NUM_VALUES; i += 1) {
      cur_val = new uint32_t[1];
      *cur_val = i*2;
      // Insert key = i, value = i*2
      EXPECT_EQ(ht.insert(i, cur_val), true);  
    }
    // Now check that they're all there
    uint32_t* val;
    for (uint32_t i = 0; i < TEST_HM_NUM_VALUES; i += 1) {
      EXPECT_TRUE(ht.lookup(i, val));  // find key = i (value = i)
      EXPECT_EQ(i*2, *val);
    }
    for (uint32_t i = TEST_HM_NUM_VALUES; i < 2*TEST_HM_NUM_VALUES; i += 1) {
      EXPECT_FALSE(ht.lookup(i, val));  // find key = i (value = i)
    }
    EXPECT_EQ(ht.count(), TEST_HM_NUM_VALUES);
  }

  // TEST 2Ptr: Create a hash table, insert items from 1:N and delete some
  TEST(HashMapPtr, CreationAndDeletion) {
    HashMap<uint32_t, uint32_t*> ht(TEST_HM_START_SIZE, &HashUint);
    uint32_t* cur_val;
    EXPECT_FALSE(ht.remove(1));  // Nothing there: should fail
    cur_val = new uint32_t[1]; 
    *cur_val = 1;
    EXPECT_TRUE(ht.insert(0, cur_val));
    EXPECT_FALSE(ht.remove(1));  // 0 is there, but 1 is not: should fail
    cur_val = new uint32_t[1]; 
    *cur_val = 0;
    EXPECT_TRUE(ht.insert(1, cur_val));
    EXPECT_TRUE(ht.remove(1));  // should work this time
    EXPECT_FALSE(ht.remove(1));  // Make sure the value is actually gone
    EXPECT_TRUE(ht.lookup(0, cur_val));  // key=0 should still be there
    EXPECT_EQ(*cur_val, 1);  // key=0's value shouldn't have been corrupted
  }

  // TEST 3Ptr: Duplicate insertion
  TEST(HashMapPtr, DuplicateInsertion) {
    HashMap<uint32_t, uint32_t*> ht(TEST_HM_START_SIZE, &HashUint);
    uint32_t* cur_val;
    cur_val = new uint32_t[1]; 
    *cur_val = 1;
    EXPECT_TRUE(ht.insert(0, cur_val));
    cur_val = new uint32_t[1]; 
    *cur_val = 0;
    EXPECT_FALSE(ht.insert(0, cur_val));  // Try twice with two different values
    delete cur_val;
    cur_val = new uint32_t[1]; 
    *cur_val = 1;
    EXPECT_FALSE(ht.insert(0, cur_val));  // (since value shouldn't matter)
    delete cur_val;
  }


}  // unnamed namespace
