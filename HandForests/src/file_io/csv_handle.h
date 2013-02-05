//
//  csv_handle.h
//
//  Created by Jonathan Tompson on 5/1/12.
//

#ifndef FILE_IO_CSV_HANDLE_HEADER
#define FILE_IO_CSV_HANDLE_HEADER

#include <string>

namespace data_str { template <typename T> class VectorManaged; }

namespace file_io {
  class CSVHandle { 
  public:
    CSVHandle();

    inline bool open() { return open_; }
    inline std::string filename() { return filename_; }

    // flattenToken - used to generate error strings from the Vector token
    static std::string flattenToken(data_str::VectorManaged<char*>* cur_token);

    // public since they're used by ui_elem_setting.h
    static void seperateWhiteSpace(char* cur_token, 
      data_str::VectorManaged<char*>* return_token);
    static char* joinWhiteSpace(data_str::VectorManaged<char*>* cur_token);
    static int getDoublePrecision(const char* num);

  protected:
    std::string filename_;
    bool open_;

    // Non-copyable, non-assignable.
    CSVHandle(CSVHandle&);
    CSVHandle& operator=(const CSVHandle&);
  };

};  // namespace file_io

#endif  // FILE_IO_CSV_HANDLE_HEADER
