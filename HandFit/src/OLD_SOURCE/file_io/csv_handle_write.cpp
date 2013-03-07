#include <sstream>
#include <string>
#include "file_io/csv_handle_write.h"
#include "math/math_types.h"  // for uint
#include "data_str/vector_managed.h"
#include "exceptions/wruntime_error.h"

using data_str::VectorManaged;
using std::string;
using std::wruntime_error;
using std::stringstream;

namespace file_io {

  CSVHandleWrite::CSVHandleWrite(string filename) {
    filename_.append(filename);

    // See if the file exists
    writer_.open(filename_.c_str(), std::ios::out);
    if (!writer_.is_open())
      throw wruntime_error("CSVHandleWrite::CSVHandleWrite - cannot open file");
    open_ = true;
  }

  CSVHandleWrite::~CSVHandleWrite() {
    if (writer_.is_open())
      writer_.close();
  }

  void CSVHandleWrite::close() {
    if (!writer_.is_open())
      throw wruntime_error("CSVHandleWrite::close - File not open.");
    writer_.close();
    open_ = false;
  }

  void CSVHandleWrite::writeLine(char * line) {
    if (!writer_.is_open())
      throw wruntime_error("CSVHandleWrite::writeLine - File not open.");

    writer_ << line;
  }

  void CSVHandleWrite::writeToken(VectorManaged<char*>* token_array ) {
    if (!writer_.is_open())
      throw wruntime_error("CSVHandleWrite::writeToken - File not open.");

    for (uint32_t i = 0; i < token_array->size(); i ++) {
      writer_ << *token_array->at(i);
      if (i < (token_array->size()-1))
        writer_ << ",";  // Don't start a new tab on the last entry
    }
    writer_ << "\n";
  }
  
  void CSVHandleWrite::flush() {
    if (!writer_.is_open())
      throw wruntime_error("CSVHandleWrite::flush - File not open.");
    writer_.flush();
  }

}  // namespace file_io
