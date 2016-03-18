#ifndef _exceptions_h
#define _exceptions_h

#include <stdexcept>

namespace xamla {

class NotImplementedException
  : public std::logic_error {
public:
  NotImplementedException()
    : logic_error("Function not implemented.") {
  }

  NotImplementedException(const std::string &msg)
    : logic_error(msg) {
  }
};

class InvalidTypeException
  : public std::runtime_error {
public:
  InvalidTypeException(const std::string& reason)
    : runtime_error(reason) {
  }
};

} // xamla

#endif
