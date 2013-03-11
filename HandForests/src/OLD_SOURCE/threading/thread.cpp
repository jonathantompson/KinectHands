//
//  thread.cpp
//
//  Created by Jonathan Tompson on 7/10/12.
//
//  A simple wrapper around std::thread for spawning threads from a Callback

#include <stdio.h>  // perror
#include <stdlib.h>  // exit
#include <thread>
#include <sstream>
#include "threading/thread.h"

using std::thread;

namespace threading {
  
  static void* threadFunction(void* arg) {
    (* reinterpret_cast<Callback<void>*>(arg))();
    return 0;
  }
  
  thread MakeThread(Callback<void>* body) {
    void* arg = reinterpret_cast<void*>(body);
    thread tid = thread(threadFunction, arg);
    return tid;
  }
  
  void* GetThreadID(std::thread* thread) {
    std::stringstream ios; 
    ios << thread->get_id();
    void* tid;
    ios >> tid;
    return tid;
  }
  
}  // namespace threading
