//
//  test_math.cpp
//
//  Created by Jonathan Tompson on 3/14/12.
//
//  C++ code to implement test_math.m --> Use matlab to check C results
//  Typical code will not use templates directly, but I do so here so
//  that I can switch out float for doubles and test both cases
//
//  LOTS of tests here.  There's a lot of code borrowed from Prof 
//  Alberto Lerner's code repository (I took his distributed computing class, 
//  which was amazing), particularly the test unit stuff.

#include "tests/test_math/test_vec2_mat2x2.h"
#include "tests/test_math/test_vec3_mat3x3.h"
#include "tests/test_math/test_vec4_mat4x4.h"
#include "tests/test_math/test_quaternion.h"

int main(int argc, char* argv[]) {
#ifdef _WIN32
  _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF | 
    _CRTDBG_CHECK_ALWAYS_DF);
#endif

  int ret_val = RUN_TESTS(argc, argv);

#ifdef _WIN32
  system("PAUSE");
#endif
  return ret_val;
}

