#include "torch-std.h"
#include "variable.h"
#include <map>

typedef std::map<std::string, xamla::Variable> VariableMap;
typedef std::vector<xamla::Variable> VariableVector;

STDIMP(VariableMap *, VariableMap, new)() {
  return new VariableMap();
}

STDIMP(VariableMap *, VariableMap, clone)(VariableMap *self) {
  return new VariableMap(*self);
}

STDIMP(void, VariableMap, delete)(VariableMap *ptr) {
  delete ptr;
}

STDIMP(int, VariableMap, size)(VariableMap *self) {
  return (int)self->size();
}

STDIMP(void, VariableMap, clear)(VariableMap *self) {
  self->clear();
}

STDIMP(bool, VariableMap, getAt)(VariableMap *self, const char *key, xamla::Variable *result) {
  VariableMap::iterator i = self->find(key);
  if (i == self->end())
    return false;

  *result = i->second;
  return true;
}

STDIMP(void, VariableMap, setAt)(VariableMap *self, const char *key, xamla::Variable *value) {
  (*self)[key] = *value;
}

STDIMP(bool, VariableMap, erase)(VariableMap *self, const char *key) {
  return self->erase(std::string(key)) == 1;
}

STDIMP(bool, VariableMap, exists)(VariableMap *self, const char *key) {
  return self->count(key) > 0;
}

STDIMP(void, VariableMap, keys)(VariableMap *self, StringVector *result) {
  result->clear();
  for (VariableMap::const_iterator i = self->begin(); i != self->end(); ++i) {
    result->push_back(i->first);
  }
}

STDIMP(void, VariableMap, values)(VariableMap *self, VariableVector *result) {
  result->clear();
  for (VariableMap::const_iterator i = self->begin(); i != self->end(); ++i) {
    result->push_back(i->second);
  }
}
