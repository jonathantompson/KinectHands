//
//  csv_handle_write.h
//
//  Created by Jonathan Tompson on 5/1/12.
//

#ifndef FILE_IO_CSV_HANDLE_WRITE_HEADER
#define FILE_IO_CSV_HANDLE_WRITE_HEADER

#include <string>
#include <fstream>
#include "file_io/csv_handle.h"

namespace data_str { template <typename T> class VectorManaged; }

namespace file_io {

  class CSVHandleWrite : public CSVHandle {
  public:
    explicit CSVHandleWrite(std::string filename);
    ~CSVHandleWrite();

    // Access and Modifier Functions
    void close();
    bool readNextToken(data_str::VectorManaged<char*>* token_array);
    inline bool checkEOF( ) { return writer_.eof(); }
    void writeLine(char* line);
    void writeToken(data_str::VectorManaged<char*>* token_array);
    void flush();

  private:
    std::ofstream writer_;

    // Non-copyable, non-assignable.
    CSVHandleWrite(CSVHandleWrite&);
    CSVHandleWrite& operator=(const CSVHandleWrite&);
  };

};  // namespace file_io

#endif  // FILE_IO_CSV_HANDLE_WRITE_HEADER
