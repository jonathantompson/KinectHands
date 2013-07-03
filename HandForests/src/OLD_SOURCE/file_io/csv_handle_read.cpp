//
//  csv_handle_read.cpp
//
//  Created by Jonathan Tompson on 5/1/12.
//

#include <stdio.h>  // snprintf
#include <sstream>
#include <string>
#include "file_io/csv_handle_read.h"
#include "math/math_types.h"  // for uint
#include "data_str/vector_managed.h"
#include "exceptions/wruntime_error.h"

using data_str::VectorManaged;
using std::string;
using std::wruntime_error;
using std::stringstream;

#if _WIN32
#define snprintf _snprintf
#endif

namespace file_io {

  CSVHandleRead::CSVHandleRead(string filename) {
    filename_.append(filename);

    // See if the file exists
    reader_.open(filename_.c_str(), std::ios::in);
    if (!reader_.is_open()) {
      throw wruntime_error(string(
        "CSVHandleRead::CSVHandleRead - cannot open file: ") + filename);
    }
    open_ = true;
  }

  CSVHandleRead::~CSVHandleRead() {
    if (reader_.is_open())
      reader_.close();
  }

  void CSVHandleRead::close() {
    if (!reader_.is_open())
      throw wruntime_error("CSVHandleRead::close - File not open.");
    reader_.close();
    open_ = false;
  }

  bool CSVHandleRead::readNextToken(VectorManaged<char*>* token_array, 
    bool include_whitespace) {
    if (!open_ || !reader_.is_open())
      throw wruntime_error("CSVHandleRead::readNextToken - File not open.");

    token_array->clear();

    if (reader_.bad())
      return false;  // eof, failbit or badbit flags are set

    string cur_line;
    getline(reader_, cur_line);

    if (cur_line.empty())
      return false;

    // We sucessfully read a token -> go through each character, parsing out the
    // csv and removing whitespace
    stringstream token;
    string cur_token_string;
    char* cur_token_c_str = NULL;
    bool quotes = false;
    for (uint32_t i = 0; i < cur_line.size(); i ++) {
      if (cur_line.at(i) == ',') {
        cur_token_string = token.str(); 
        cur_token_c_str = new char[ cur_token_string.size() + 1]; 
        snprintf(cur_token_c_str, cur_token_string.size()+1, "%s", 
          cur_token_string.c_str());

        // Check that all quotes were closed
        if (quotes) {
          string error = string("CSVHandleRead::readNextToken") + 
            string("- Error reading csv file, a set of quotes was not closed") +
            string(" at the end of the token:") + string(cur_token_c_str);
          throw wruntime_error(error);
        }

        token_array->pushBack(cur_token_c_str);
        token.clear();
        token.str("");
      } else if (include_whitespace) {
        token << cur_line.at(i);
      } else {
        // Parse out the whitespace quotes and tabs
        if (cur_line.at(i) == '"') {
          // If we're quoting, keep the spaces and tabs
          quotes = !quotes;
        } else if (cur_line.at(i) == ' ' && !quotes) {
          // Do nothing for black spaces
        } else if (cur_line.at(i) == '\t' && !quotes) {
          // Do nothing for tab characters
        } else {
          token << cur_line.at(i);
        }
      }
    }

    // Now add the last element
    cur_token_string = token.str();
    cur_token_c_str = new char[ cur_token_string.size() + 1];
    snprintf(cur_token_c_str, cur_token_string.size()+1, "%s", 
      cur_token_string.c_str());

    // Check that all quotes were closed
    if (quotes) {
          string error = string("CSVHandleRead::readNextToken") + 
            string("- Error reading csv file, a set of quotes was not closed") +
            string(" at the end of the token:") + string(cur_token_c_str);
      throw wruntime_error(error);
    }

    token_array->pushBack(cur_token_c_str);

    return true;
  }

}  // namespace file_io
