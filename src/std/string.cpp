#include "torch-std.h"
#include <string>

STDIMP(std::string*, string, new)(const char* s = 0, size_t len = 0)
{
  if (!s || !len)
    return new std::string();
  else
    return new std::string(s, len);
}

STDIMP(void, string, delete)(std::string *self)
{
  delete self;
}

STDIMP(std::string*, string, clone)(std::string *self)
{
  return new std::string(*self);
}

STDIMP(void, string, assign)(std::string *self, const char *s, size_t len)
{
  self->assign(s, len);
}

STDIMP(int, string, length)(std::string *self)
{
  return self->length();
}

STDIMP(const char*, string, c_str)(std::string *self)
{
  return self->c_str();
}
