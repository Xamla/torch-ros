#include "torch-std.h"
#include "variable.h"

using namespace xamla;

#define VariableVector_Ptr xamla::VariableVector_ptr

STDIMP(VariableVector_Ptr *, VariableVector, new)() {
  return new VariableVector_Ptr(new VariableVector());
}

STDIMP(void, VariableVector, delete)(VariableVector_Ptr *self) {
  delete self;
}

STDIMP(VariableVector_Ptr *, VariableVector, clone)(VariableVector_Ptr *self) {
  return new VariableVector_Ptr(new VariableVector(**self));
}

STDIMP(int, VariableVector, size)(VariableVector_Ptr *self) {
  return static_cast<int>((*self)->size());
}

STDIMP(void, VariableVector, getAt)(VariableVector_Ptr *self, size_t pos, xamla::Variable *result) {
  VariableVector& v = **self;
  *result = v[pos];
}

STDIMP(void, VariableVector, setAt)(VariableVector_Ptr *self, size_t pos, xamla::Variable *value) {
  VariableVector& v = **self;
  v[pos] = *value;
}

STDIMP(void, VariableVector, push_back)(VariableVector_Ptr *self, xamla::Variable *value) {
  (*self)->push_back(*value);
}

STDIMP(void, VariableVector, pop_back)(VariableVector_Ptr *self) {
  (*self)->pop_back();
}

STDIMP(void, VariableVector, clear)(VariableVector_Ptr *self) {
  (*self)->clear();
}

STDIMP(void, VariableVector, insert)(VariableVector_Ptr *self, size_t pos, size_t n, xamla::Variable *value) {
  VariableVector& v = **self;
  VariableVector::iterator i = pos >= v.size() ? v.end() : v.begin() + pos;
  v.insert(i, n, *value);
}

STDIMP(void, VariableVector, erase)(VariableVector_Ptr *self, size_t begin, size_t end) {
  if (begin >= end)
    return;

  VariableVector& v = **self;
  VariableVector::iterator b = begin >= v.size() ? v.end() : v.begin() + begin;
  VariableVector::iterator e = end >= v.size() ? v.end() : v.begin() + end;
  v.erase(b, e);
}

STDIMP(bool, VariableVector, empty)(VariableVector_Ptr *self) {
  return (*self)->empty();
}
