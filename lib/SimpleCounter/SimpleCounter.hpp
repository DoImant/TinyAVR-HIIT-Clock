#pragma once

//////////////////////////////////////////////////////////////////////////////
/// \brief  Class for creating a counter. This can be increased up to
///         a predefined limit value. Once the limit value is reached,
///         the counter is automatically reset to 0.
///
///         Negative values are not possible.
///
//////////////////////////////////////////////////////////////////////////////
class SimpleCounter {
public:
  SimpleCounter(unsigned int max_counter) : max_counter {max_counter} {}
  ~SimpleCounter() {}

  // SimpleCounter& operator=(const SimpleCounter& rValue) {
  //   if (rValue.out_of_range && this != &rValue) {
  //     rValue.is_count_up ? operator++() : operator--();
  //   } else {
  //     out_of_range = false;
  //   }
  //   return *this;
  // }

  constexpr operator bool() const { return out_of_range; }

  constexpr unsigned int operator()() const { return counter; }
  unsigned int scMax() const { return max_counter; }

  SimpleCounter& operator++() {   // pre increment (++x)
    is_count_up = true;
    ++counter;
    if (counter >= max_counter) {
      counter = 0;
      out_of_range = true;
    } else {
      out_of_range = false;
    }
    return *this;
  }

  // post increment (x++): int needed (other method call pattern than operator++()) but not used
  const SimpleCounter operator++(int) {
    SimpleCounter temp {*this};
    operator++();
    return temp;
  }

  SimpleCounter& operator--() {   // pre increment (++x)
    is_count_up = false;
    --counter;
    if (counter > max_counter) {
      counter = max_counter - 1;
      out_of_range = true;
    } else {
      out_of_range = false;
    }
    return *this;
  }

  // post increment (x++): int needed (other method call pattern than operator++()) but not used
  const SimpleCounter operator--(int) {
    SimpleCounter temp {*this};
    operator--();
    return temp;
  }

private:
  unsigned int max_counter;
  unsigned int counter {0};
  bool out_of_range {false};
  bool is_count_up {true};
};