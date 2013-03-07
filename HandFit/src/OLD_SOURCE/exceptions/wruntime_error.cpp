//
//  wruntime_error.cpp
//
//  Created by Jonathan Tompson on 5/27/12.
//

#include <string>
#include "exceptions/wruntime_error.h"
#include "string_util/string_util.h"

using string_util::ToNarrowString;
using string_util::ToWideString;

namespace std {

  wruntime_error::wruntime_error(const wstring& errorMsg)
    : runtime_error(ToNarrowString(errorMsg)), mErrorMsg(errorMsg) {
    // NOTE: We give the runtime_error base the narrow version of the 
    //  error message. This is what will get shown if what() is called.
    //  The wruntime_error inserter or errorMsg() should be used to get 
    //  the wide version.
#if defined(_WIN32) && defined(_DEBUG) && defined(BREAK_ON_EXCEPTION)
    if (IsDebuggerPresent()) {
      _CrtDbgBreak();
    }
#endif
  }

  wruntime_error::wruntime_error(const string& errorMsg)
    : runtime_error(errorMsg), mErrorMsg(ToWideString(errorMsg)) {
    // NOTE: We give the runtime_error base the narrow version of the 
    //  error message. This is what will get shown if what() is called.
    //  The wruntime_error inserter or errorMsg() should be used to get 
    //  the wide version.
#if defined(_WIN32) && defined(_DEBUG) && defined(BREAK_ON_EXCEPTION)
    if (IsDebuggerPresent()) {
      _CrtDbgBreak();
    }
#endif
  }

  wruntime_error::wruntime_error(const wruntime_error& rhs)
    : runtime_error(ToNarrowString(rhs.errorMsg())), mErrorMsg(rhs.errorMsg()) {
  }

  wruntime_error& wruntime_error::operator=(const wruntime_error& rhs) {
    // copy the wruntime_error
    runtime_error::operator=(rhs); 
    mErrorMsg = rhs.mErrorMsg; 

#if defined(_WIN32) && defined(_DEBUG) && defined(BREAK_ON_EXCEPTION)
    if (IsDebuggerPresent()) {
      _CrtDbgBreak();
    }
#endif

    return *this; 
  }

#if defined(WIN32) || defined(_WIN32) || defined(__APPLE__)
  wruntime_error::~wruntime_error() {
  }
#else
  wruntime_error::~wruntime_error() _GLIBCXX_USE_NOEXCEPT {
  }
#endif

  const wstring& wruntime_error::errorMsg() const { 
    return mErrorMsg; 
  }
}
