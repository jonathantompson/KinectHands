//
//  test_data_str.cpp
//
//  Created by Jonathan Tompson on 4/26/12.
//
//  LOTS of tests here.  There's a lot of code borrowed from Prof 
//  Alberto Lerner's code repository (I took his distributed computing class, 
//  which was amazing), particularly the test unit stuff.

#include "tests/test_data_str/test_circular_buffer.h"
#include "tests/test_data_str/test_hash_map.h"
#include "tests/test_data_str/test_hash_set.h"
#include "tests/test_data_str/test_vector.h"
#include "tests/test_data_str/test_pair.h"
#include "tests/test_data_str/test_min_heap.h"

int main(int argc, char* argv[]) {
#if defined(_WIN32) && defined(_DEBUG)
  _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif
  int ret_val = RUN_TESTS(argc, argv);
#ifdef _WIN32
  system("PAUSE");
#endif
  return ret_val;
}
