//
//  wruntime_error.h
//
//  Created by Jonathan Tompson on 5/27/12.
//
//  A simple wide version of std::runtime_error exception with break points for
//  visual studio.

#ifndef STD_WRUNTIME_ERROR_HEADER
#define STD_WRUNTIME_ERROR_HEADER

#include <stdexcept>  // for runtime_error
#include <string>

namespace std {

  class wruntime_error : public std::runtime_error {
  public:       
    explicit wruntime_error(const std::string& errorMsg);
    explicit wruntime_error(const std::wstring& errorMsg);
    wruntime_error(const wruntime_error& rhs);
    wruntime_error& operator=(const wruntime_error& rhs);
    virtual ~wruntime_error();
    const std::wstring& errorMsg() const;

  private:
    std::wstring mErrorMsg;
  };
};  // std namespace

#endif  // NAMESPACE_CLASS_TEMPLATE_HEADER
