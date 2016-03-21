#include "torch-std.h"
#include "variable.h"
#include <map>

typedef std::map<std::string, xamla::Variable> VariableTable;
typedef std::vector<xamla::Variable> VariableVector;

STDIMP(VariableTable *, VariableTable, new)() {
  return new VariableTable();
}

STDIMP(VariableTable *, VariableTable, clone)(VariableTable *self) {
  return new VariableTable(*self);
}

STDIMP(void, VariableTable, delete)(VariableTable *ptr) {
  delete ptr;
}

STDIMP(int, VariableTable, size)(VariableTable *self) {
  return (int)self->size();
}

STDIMP(void, VariableTable, clear)(VariableTable *self) {
  self->clear();
}

STDIMP(bool, VariableTable, getField)(VariableTable *self, const char *key, xamla::Variable *result) {
  VariableTable::iterator i = self->find(key);
  if (i == self->end())
    return false;

  *result = i->second;
  return true;
}

STDIMP(void, VariableTable, setField)(VariableTable *self, const char *key, xamla::Variable *value) {
  (*self)[key] = *value;
}

STDIMP(bool, VariableTable, erase)(VariableTable *self, const char *key) {
  return self->erase(std::string(key)) == 1;
}

STDIMP(bool, VariableTable, exists)(VariableTable *self, const char *key) {
  return self->count(key) > 0;
}

STDIMP(void, VariableTable, keys)(VariableTable *self, StringVector *result) {
  result->clear();
  for (VariableTable::const_iterator i = self->begin(); i != self->end(); ++i) {
    result->push_back(i->first);
  }
}

STDIMP(void, VariableTable, values)(VariableTable *self, VariableVector *result) {
  result->clear();
  for (VariableTable::const_iterator i = self->begin(); i != self->end(); ++i) {
    result->push_back(i->second);
  }
}
