#include "torch-std.h"
#include "variable.h"

using namespace xamla;

#define VariableVector_Ptr xamla::VariableVector_ptr
#define VariableTable_Ptr xamla::VariableTable_ptr

STDIMP(VariableTable_Ptr *, VariableTable, new)() {
  return new VariableTable_Ptr(new VariableTable());
}

STDIMP(void, VariableTable, delete)(VariableTable_Ptr *handle) {
  delete handle;
}

STDIMP(VariableTable_Ptr *, VariableTable, clone)(VariableTable_Ptr *self) {
  return new VariableTable_Ptr(new VariableTable(**self));
}

STDIMP(int, VariableTable, size)(VariableTable_Ptr *self) {
  return (int)(*self)->size();
}

STDIMP(void, VariableTable, clear)(VariableTable_Ptr *self) {
  (*self)->clear();
}

STDIMP(bool, VariableTable, getField)(VariableTable_Ptr *self, const char *key, Variable *result) {
  VariableTable& t = **self;
  VariableTable::iterator i = t.find(key);
  if (i == t.end())
    return false;

  *result = i->second;
  return true;
}

STDIMP(void, VariableTable, setField)(VariableTable_Ptr *self, const char *key, Variable *value) {
  VariableTable& t = **self;
  t[key] = *value;
}

STDIMP(bool, VariableTable, erase)(VariableTable_Ptr *self, const char *key) {
  return (*self)->erase(std::string(key)) == 1;
}

STDIMP(bool, VariableTable, exists)(VariableTable_Ptr *self, const char *key) {
  return (*self)->count(key) > 0;
}

STDIMP(void, VariableTable, keys)(VariableTable_Ptr *self, StringVector *result) {
  VariableTable& t = **self;
  result->clear();
  for (VariableTable::const_iterator i = t.begin(); i != t.end(); ++i) {
    result->push_back(i->first);
  }
}

STDIMP(void, VariableTable, values)(VariableTable_Ptr *self, VariableVector_Ptr *result) {
  VariableTable& t = **self;
  VariableVector& r = **result;
  r.clear();
  for (VariableTable::const_iterator i = t.begin(); i != t.end(); ++i) {
    r.push_back(i->second);
  }
}
