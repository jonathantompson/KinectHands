//
//  csv_handle_read.h
//
//  Created by Jonathan Tompson on 5/1/12.
//

#ifndef FILE_IO_CSV_HANDLE_READ_HEADER
#define FILE_IO_CSV_HANDLE_READ_HEADER

#include <string>
#include <fstream>
#include "file_io/csv_handle.h"

namespace data_str { template <typename T> class VectorManaged; }

namespace file_io {

  class CSVHandleRead : public CSVHandle {
  public:
    explicit CSVHandleRead(std::string filename);
    ~CSVHandleRead();

    void close();
    bool readNextToken(data_str::VectorManaged<char*> * token_array, 
      bool include_whitespace);
    inline bool checkEOF( ) { return reader_.eof(); }

  private:
    std::ifstream reader_;

    // Non-copyable, non-assignable.
    CSVHandleRead(CSVHandleRead&);
    CSVHandleRead& operator=(const CSVHandleRead&);
  };

};  // namespace file_io

#endif  // FILE_IO_CSV_HANDLE_READ_HEADER
