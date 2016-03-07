#include "torch-std.h"

STDIMP(StringVector *, StringVector, new)() {
  return new StringVector();
}

STDIMP(StringVector *, StringVector, clone)(StringVector *self) {
  return new StringVector(*self);
}

STDIMP(void, StringVector, delete)(StringVector *ptr) {
  delete ptr;
}

STDIMP(int, StringVector, size)(StringVector *self) {
  return static_cast<int>(self->size());
}

STDIMP(const char*, StringVector, getAt)(StringVector *self, size_t pos) {
  StringVector& v = *self;
  return v[pos].c_str();
}

STDIMP(void, StringVector, setAt)(StringVector *self, size_t pos, const char *value) {
  StringVector& v = *self;
  v[pos] = value;
}

STDIMP(void, StringVector, push_back)(StringVector *self, const char *value) {
  self->push_back(value);
}

STDIMP(void, StringVector, pop_back)(StringVector *self) {
  self->pop_back();
}

STDIMP(void, StringVector, clear)(StringVector *self) {
  self->clear();
}

STDIMP(void, StringVector, insert)(StringVector *self, size_t pos, size_t n, const char *value) {
  StringVector& v = *self;
  StringVector::iterator i = pos >= v.size() ? v.end() : v.begin() + pos;
  v.insert(i, n, value);
}

STDIMP(void, StringVector, erase)(StringVector *self, size_t begin, size_t end) {
  if (begin >= end)
    return;
    
  StringVector& v = *self;
  StringVector::iterator b = begin >= v.size() ? v.end() : v.begin() + begin;
  StringVector::iterator e = end >= v.size() ? v.end() : v.begin() + end;
  v.erase(b, e);
}

STDIMP(bool, StringVector, empty)(StringVector *self) {
  return self->empty();
}
