//
//  int_par.h
//
//  Created by Jonathan Tompson on 4/26/12.
//

#ifndef data_str_INT_PAIR_HEADER
#define data_str_INT_PAIR_HEADER

namespace data_str {

  template <typename TFirst, typename TSecond>
  class Pair {
  public:
    Pair() { }
    Pair(TFirst _first, TSecond _second) : first(_first), second(_second) { }

    bool operator == (const Pair<TFirst, TSecond> &a);
    void operator = (const Pair<TFirst, TSecond> &a);

    TFirst first;
    TSecond second;
  };

  template <typename TFirst, typename TSecond>
  bool Pair<TFirst, TSecond>::operator== (const Pair<TFirst, TSecond>& a) {
    if (this == &a) {  // if both point to the same memory
      return true; 
    }
    if (first == a.first && second == a.second) { 
      return true; 
    } else {
      return false;
    }
  };

  template <typename TFirst, typename TSecond>
  void Pair<TFirst, TSecond>::operator= (const Pair<TFirst, TSecond>& a) {
    if (this == &a) {  // if both point to the same memory
      return; 
    }
    first = a.first;
    second = a.second;
  };

};  // data_str namespace

#endif  // data_str_INT_PAIR_HEADER
