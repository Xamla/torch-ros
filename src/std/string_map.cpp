#include "torch-std.h"

STDIMP(StringMap *, StringMap, new)() {
  return new StringMap();
}

STDIMP(StringMap *, StringMap, clone)(StringMap *self) {
  return new StringMap(*self);
}

STDIMP(void, StringMap, delete)(StringMap *ptr) {
  delete ptr;
}

STDIMP(int, StringMap, size)(StringMap *self) {
  return static_cast<int>(self->size());
}

STDIMP(void, StringMap, clear)(StringMap *self) {
  self->clear();
}

STDIMP(const char *, StringMap, getAt)(StringMap *self, const char *key) {
  return (*self)[key].c_str();
}

STDIMP(const char *, StringMap, setAt)(StringMap *self, const char *key, const char *value) {
  (*self)[key] = value;
}

STDIMP(bool, StringMap, insert)(StringMap *self, const char *key, const char *value) {
  return self->insert(StringMap::value_type(key, value)).second;
}

STDIMP(bool, StringMap, erase)(StringMap *self, const char *key) {
  return self->erase(key) > 0;
}

STDIMP(bool, StringMap, exists)(StringMap *self, const char *key) {
  return self->count(key) > 0;
}

STDIMP(void, StringMap, keys)(StringMap *self, StringVector *result) {
  result->clear();
  for (StringMap::const_iterator i = self->begin(); i != self->end(); ++i) {
    result->push_back(i->first);
  }
}

STDIMP(void, StringMap, values)(StringMap *self, StringVector *result) {
  result->clear();
  for (StringMap::const_iterator i = self->begin(); i != self->end(); ++i) {
    result->push_back(i->second);
  }
}
