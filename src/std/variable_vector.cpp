#include "torch-std.h"
#include "variable.h"

typedef std::vector<xamla::Variable> VariableVector;

STDIMP(VariableVector *, VariableVector, new)() {
  return new VariableVector();
}

STDIMP(VariableVector *, VariableVector, clone)(VariableVector *self) {
  return new VariableVector(*self);
}

STDIMP(void, VariableVector, delete)(VariableVector *ptr) {
  delete ptr;
}

STDIMP(int, VariableVector, size)(VariableVector *self) {
  return static_cast<int>(self->size());
}

STDIMP(void, VariableVector, getAt)(VariableVector *self, size_t pos, xamla::Variable *result) {
  VariableVector& v = *self;
  *result = v[pos];
}

STDIMP(void, VariableVector, setAt)(VariableVector *self, size_t pos, xamla::Variable *value) {
  VariableVector& v = *self;
  v[pos] = *value;
}

STDIMP(void, VariableVector, push_back)(VariableVector *self, xamla::Variable *value) {
  self->push_back(*value);
}

STDIMP(void, VariableVector, pop_back)(VariableVector *self) {
  self->pop_back();
}

STDIMP(void, VariableVector, clear)(VariableVector *self) {
  self->clear();
}

STDIMP(void, VariableVector, insert)(VariableVector *self, size_t pos, size_t n, xamla::Variable *value) {
  VariableVector& v = *self;
  VariableVector::iterator i = pos >= v.size() ? v.end() : v.begin() + pos;
  v.insert(i, n, *value);
}

STDIMP(void, VariableVector, erase)(VariableVector *self, size_t begin, size_t end) {
  if (begin >= end)
    return;

  VariableVector& v = *self;
  VariableVector::iterator b = begin >= v.size() ? v.end() : v.begin() + begin;
  VariableVector::iterator e = end >= v.size() ? v.end() : v.begin() + end;
  v.erase(b, e);
}

STDIMP(bool, VariableVector, empty)(VariableVector *self) {
  return self->empty();
}
