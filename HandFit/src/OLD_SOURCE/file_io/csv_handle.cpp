#include <sstream>
#include <string>
#include "file_io/csv_handle.h"
#include "math/math_types.h"  // for uint
#include "data_str/vector_managed.h"
#include "exceptions/wruntime_error.h"

using data_str::VectorManaged;
using std::wruntime_error;

#if _WIN32
  #ifndef snprintf
    #define snprintf _snprintf
  #endif
#endif

namespace file_io {

  CSVHandle::CSVHandle() {
    open_ = false;
  }

  // FlattenToken - Flatten vector of strings into one string.
  std::string CSVHandle::flattenToken(VectorManaged<char*>* cur_token ) {
    // We sucessfully got a token, so go through each character, parsing out 
    // the csv and removing whitespace.
    std::stringstream token;

    for (uint32_t i = 0; i < cur_token->size(); i ++) {
      token << *cur_token->at(i);
      if (i < (cur_token->size()-1))
        token << ", ";
    }

    return token.str();
  }

  // SeperateWhiteSpace, Seperate input string into left_hand whitespace, the 
  // data and the righthand whitespace.
  void CSVHandle::seperateWhiteSpace(char* cur_token, 
    VectorManaged<char*>* return_token ) {
    if (return_token == NULL)
      throw wruntime_error(L"CSVHandle::seperateWhiteSpace: token is NULL!");
    return_token->clear();

    // Go through input string and seperate out whitespace
    std::stringstream token;
    token.clear(); 
    token.str("");  // Clear it to an empty string
    std::string cur_token_string;
    char* cur_token_c_str = NULL;

    // Parse out the beginning whitespace
    uint32_t i = 0;
    bool begin_quote = false;
    while (i < strlen(cur_token)) {
      if (cur_token[i] == ' ' || cur_token[i] == '\t') {
        token << cur_token[i];
        i++;
      } else if (cur_token[i] == '"') {
        token << cur_token[i];
        i++;
        begin_quote = true;
        break;
      } else {  // We're at the end of the section
        break;
      }
    }
    cur_token_string = token.str();
    cur_token_c_str = new char[ cur_token_string.size() + 1];
    snprintf(cur_token_c_str, cur_token_string.size()+1, "%s", 
      cur_token_string.c_str());
    return_token->pushBack(cur_token_c_str);
    token.clear();
    token.str("");

    // Parse out the data block
    bool end_quote = false;
    while (i < strlen(cur_token)) {
      if (begin_quote) { 
        // We're in a quote block, the only thing that ends a quote block is 
        // another quote
        if (cur_token[i] == '"') {
          end_quote = true;
          break;
        } else {
          token << cur_token[i];
          i++;
        }
      } else { 
        // We're not in a quote block, a space or tab will end the section
        if (cur_token[i] == ' ' || cur_token[i] == '\t') {
          break;
        } else {
          token << cur_token[i];
          i++;
        }
      }
    }
    if (begin_quote == true && end_quote == false)
      throw std::wruntime_error(
      L"CSVHandle::seperateWhiteSpace - A quote block was not finished!");
    cur_token_string = token.str(); 
    cur_token_c_str = new char[ cur_token_string.size() + 1]; 
    snprintf(cur_token_c_str, cur_token_string.size()+1, "%s", 
      cur_token_string.c_str());
    return_token->pushBack(cur_token_c_str); 
    token.clear();
    token.str("");

    if (begin_quote == true)
      token << '"';

    // Now parse the last section of whitespace
    while (i < strlen(cur_token)) {
      token << cur_token[i];
      i++;
    }
    cur_token_string = token.str();
    cur_token_c_str = new char[ cur_token_string.size() + 1];
    snprintf(cur_token_c_str, cur_token_string.size()+1, "%s", 
      cur_token_string.c_str());
    return_token->pushBack(cur_token_c_str);
    token.clear();

    if (return_token->size() != 3)
      throw std::wruntime_error(
      L"CSVHandle::SeperateWhiteSpace() - there weren't 3 blocks to parse");
  }

  // JoinWhiteSpace - Add the data and the whitespace back together
  char* CSVHandle::joinWhiteSpace(VectorManaged<char*>* cur_token ) {
    // Get the total size of the return string
    uint32_t size = 0;
    for (uint32_t i = 0; i < cur_token->size(); i ++)
      size += static_cast<uint32_t>(strlen(*cur_token->at(i)));

    char* retval = new char[size + 1];

    uint32_t cur_index =  0;
    for (uint32_t i = 0; i < cur_token->size(); i ++) {
      for (uint32_t j = 0; j < strlen(*cur_token->at(i)); j ++) {
        retval[cur_index] = (*cur_token->at(i))[j];
        cur_index++;
      }
    }
    retval[size] = '\0';  // Add a null terminator at the end

    return retval;
  }

  // GetDoublePrecision - Work out the precision of the number after the point
  int CSVHandle::getDoublePrecision(const char* num ) {
    int retval = 0;
    bool hit_decimal_point = false;
    for (uint32_t i = 0; i < strlen(num); i ++) {
      if (hit_decimal_point)
        retval++;
      else
        if (num[i] == '.')
          hit_decimal_point = true;
    }
    return retval;
  }

}  // namespace file_io
