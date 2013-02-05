//
//  thread.h
//
//  Created by Jonathan Tompson on 7/10/12.
//
//  A simple wrapper around std::thread for spawning threads from a Callback<>

#ifndef THREADING_THREAD_HEADER
#define THREADING_THREAD_HEADER

#include <thread>
#include "threading/callback.h"

namespace threading {

  // Returns the thread id of the thread create to run 'body'. The
  // latter ownership is determined by whether the callback is call-once
  // or not. Internally, 'body' is only invoked once.
  // If a thread cannot be created, this call will exit()
  std::thread MakeThread(threading::Callback<void>* body);
  
  // Creates a void* equivilient of the std::thread::id class
  void* GetThreadID(std::thread* thread);

}  // namespace base

#endif  // THREADING_THREAD_HEADER
